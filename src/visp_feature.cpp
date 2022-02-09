#include <ros/ros.h>

#include <iostream>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp/vpImage.h> 
#include "visp_bridge/image.h"
#include <visp3/gui/vpDisplayX.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp_bridge/camera.h>

class VIS
{
 public:
  VIS();
  void timerCallback(const ros::TimerEvent& event);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                     const sensor_msgs::CameraInfoConstPtr& camera_info);
  void displayAllTagID();
  int tagIdExist(int tagID);

 private:
  ros::NodeHandle nh_;
  ros::Publisher  pubTwistRobot_; // cmd_vel
  // ros::Timer controller_timer_;   // controller timer

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber camera_image_subscriber_;
  
  vpDetectorAprilTag::vpAprilTagFamily tagFamily_;
  vpDetectorAprilTag detector_;
  double tagSize_;
  int tagID_;

  vpImage<unsigned char> I_;
  vpCameraParameters camInfo_;

  vpServo task_;

  vpDisplayX display_;
  bool streamStarted_;
  bool displayImage_;

  vpFeaturePoint p_[4], pd_[4];         // current and desired feature points
  std::vector<vpPoint> point_;          // current points in image plane (x,y)
  std::vector<vpImagePoint> img_pd_;  // desired points in image plane (x,y)
};

VIS::VIS() : 
  tagFamily_(vpDetectorAprilTag::TAG_36h11),
  detector_(tagFamily_),
  tagSize_(0.096),
  tagID_(8),
  // I_(480, 640, 0),
  I_(400, 640, 0),
  // I_(960, 1280, 0),
  streamStarted_(false),
  displayImage_(true),
  point_(4),
  img_pd_(4)
{
  pubTwistRobot_  = nh_.advertise<geometry_msgs::Twist>("/bluerov/twist", 1);

  std::string transport_hint;
  nh_.param<std::string>("transport_hint", transport_hint, "raw");
                                     
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh_));

  ///bluerov/vertical_camera/image_raw

  camera_image_subscriber_ =
      it_->subscribeCamera("/bluerov/vertical_camera/image_raw", 1,
                          &VIS::imageCallback, this,
                          image_transport::TransportHints(transport_hint));

  detector_.setAprilTagQuadDecimate(1.0); // default: 1.0
  detector_.setAprilTagNbThreads(2);    // default: 2
  detector_.setAprilTagRefineEdges(1);  // default: 1
  detector_.setDisplayTag(true);

  // controller_timer_ = nh_.createTimer(
  //                       ros::Duration(1/10.0), 
  //                       &VIS::timerCallback, this);
  
  // VISP display settings
  if (displayImage_)
  {
    #if defined VISP_HAVE_X11
      vpDisplayX display(I_);
    #else
      std::cout << "No image viewer is available..." << std::endl;
    #endif
  }
  
  // Visual servo task parameters
  task_.setServo(vpServo::EYEINHAND_CAMERA);
  task_.setInteractionMatrixType(vpServo::CURRENT);
  task_.setLambda(5);

  // Desired camera position from object
  vpHomogeneousMatrix cdMo( vpTranslationVector(0, 0, tagSize_ * 5), // 3 times tag with along camera z axis
                            vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );

  // Set coordinates of features to be track by VIS 
  double tag_point = tagSize_/2;
  point_[0].setWorldCoordinates(-tag_point, -tag_point, 0); // 0 - bottom left
  point_[1].setWorldCoordinates( tag_point, -tag_point, 0); // 1 - bottom right
  point_[2].setWorldCoordinates( tag_point,  tag_point, 0); // 2 - top right
  point_[3].setWorldCoordinates(-tag_point,  tag_point, 0); // 3 - top left
  
  for (unsigned int i = 0; i < 4; i++) {
    // Create the points where the desired feature will be located when viewed from the desired camera position 
    // Computes the point coordinates in the camera frame and its 2D coordinates
    point_[i].track(cdMo);
    vpFeatureBuilder::create(pd_[i], point_[i]);

    // Create the points where the desired features are currently located based on the camera's current position 
    point_[i].track(cdMo);  
    // ^cdMo here should be the camera's current position; set here for convenience
    vpFeatureBuilder::create(p_[i], point_[i]);
    task_.addFeature(p_[i], pd_[i]);
  }

  // Set velocity twist matrix to account for offset between base and camera frame
  // Rotation from base to camera frame in zyx sequence
  vpRzyxVector vec_rzyx(vpMath::rad(-90.f), 0, vpMath::rad(-90.f)) ;
  //vpRzyxVector vec_rzyx(0, 0, 0) ;
  vpRotationMatrix Rzyx(vec_rzyx);

  // Translation from base to camera frame
  vpTranslationVector pvec_baseXcamera(-0.33, 0.12, 0.16);

  // Put together into SE(3)
  vpHomogeneousMatrix eMc( pvec_baseXcamera, Rzyx);
  
  // Compute velocity twist matrix and set the jacobian
  // Since transform is static, only need to do it once
  vpVelocityTwistMatrix cVe(eMc.inverse());
  task_.set_cVe(cVe);
}

void VIS::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  if (!streamStarted_)
  {
    //Convert the image to Visp format
    I_ = visp_bridge::toVispImage(*image_rect);
    I_.resize(image_rect->height,image_rect->width);
    camInfo_ = visp_bridge::toVispCameraParameters(*camera_info);

    // Projection to determine where the desired features should lie on image plane
    for (size_t i = 0; i < 4; i++)
      vpMeterPixelConversion::convertPoint(camInfo_, pd_[i].get_x(), pd_[i].get_y(), img_pd_[i]);

    // Initialize display
    if (displayImage_)
      display_.init(I_, 0, 0, "Camera view");
    streamStarted_ = true;
    ROS_INFO("Stream started!");
  }

  // Convert to visp type
  I_ = visp_bridge::toVispImage(*image_rect);
  camInfo_ = visp_bridge::toVispCameraParameters(*camera_info);

  // Detect AprilTags: could have multiple, so store in std::vector
  std::vector<vpHomogeneousMatrix> cMo_vec;   // Transform from camera to object (tag)
  detector_.detect(I_, tagSize_, camInfo_, cMo_vec); 

  // Displays tag ID on each detected tags (for visual/debugging purpose only)
  if (displayImage_)
  {
    vpDisplay::display(I_);
    displayAllTagID();
  }

  // Check if desired tagID exist
  int tag_index = tagIdExist(tagID_);
  if (tag_index == -1)
  {
    if (displayImage_)
      vpDisplay::flush(I_);
    return;
  }

  ROS_INFO_THROTTLE(1, "Tracking tag ID: %i found!", tag_index);
  // std::cout <<"Index of element in vector : "<<index<<std::endl;

  // Get the four points of the square encompassing the tag 
  std::vector<vpImagePoint> vec_ip = detector_.getPolygon(tag_index);
  
  // Update feature points
  for (size_t i = 0; i < vec_ip.size(); i++) // size = 4
  {
    // Using the estimated pose of the tag relative to the camera, computes
    // the point coordinates in the camera frame and its 2D coordinates
    point_[i].track(cMo_vec[0]);

    // Retrieve x,y and Z of the vpPoint structure (point_) into the feature point (p_)
    vpFeatureBuilder::create(p_[i], point_[i]); 
    
    if (displayImage_)
    {
      // Display desired and current features
      vpServoDisplay::display(task_, camInfo_, I_);
      // Show the feature index of desired and current features
      std::stringstream ss;
      ss << i;
      vpDisplay::displayText(I_, vec_ip[i] + vpImagePoint(15, 15), ss.str(), vpColor::green);
      vpDisplay::displayText(I_, img_pd_[i]+vpImagePoint(15, 15), ss.str(), vpColor::red);
    }
  }
  
  // now Compute (velocity in base frame since Jacobian for offset added in initialization already)! 
  vpColVector v = task_.computeControlLaw();
  
  // publish velocity!
  geometry_msgs::Twist twist;
  twist.linear.x = v[0];
  twist.linear.y = v[1];
  twist.linear.z = v[2];
  twist.angular.x = v[3];
  twist.angular.y = v[4];
  twist.angular.z = v[5];
  pubTwistRobot_.publish(twist);
  
  if (displayImage_)
  {
    vpDisplay::displayFrame(I_, cMo_vec[0], camInfo_, tagSize_ / 2, vpColor::none, 3);  // Display estimated pose
    vpDisplay::flush(I_);
  }
}

int VIS::tagIdExist(int tagID)
{
  // Search for ID using STL iterator
  std::vector<int> taglist = detector_.getTagsId();
  std::vector<int>::iterator it = std::find(taglist.begin(), taglist.end(), tagID);

  // If reach of iterator means tagID not found, so publish null twist and exit function
  if (it != taglist.end())
  {
    return std::distance(taglist.begin(), it); // Get index of element from iterator
  }
  else
  {
    geometry_msgs::Twist twist;
    pubTwistRobot_.publish(twist);
    ROS_WARN_THROTTLE(1, "Tracking tag not found");
    return -1;
  }
}

void VIS::displayAllTagID()
{
  std::stringstream ss;
  for (int i=0; i<detector_.getNbObjects(); i++)
  {
    std::string message = detector_.getMessage(i);
    std::size_t tag_id_pos = message.find("id: ");
    if (tag_id_pos != std::string::npos) {
      vpRect bbox = detector_.getBBox(i);
      int tag_id = atoi(message.substr(tag_id_pos + 4).c_str());
      ss.str("");
      ss << "Tag id: " << tag_id;
      vpDisplay::displayText(I_, (int)(bbox.getTop() - 10), (int)bbox.getLeft(), ss.str(), vpColor::red);
    }
  }
}  
  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_vis");

  VIS vis;

  ros::spin();

  return 0;
}


