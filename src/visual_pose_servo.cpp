#include <ros/ros.h>

#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
// #include <std_msgs/Int8.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <visp_bridge/3dpose.h>
// #include <visp_bridge/camera.h>
// #include <sensor_msgs/CameraInfo.h>

class VIS
{
 public:
  VIS();
  void timerCallback(const ros::TimerEvent& event);

 private:
  ros::NodeHandle nh_;
  ros::Publisher  pubTwistRobot_; // cmd_vel
  ros::Timer controller_timer_;   // controller timer

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf2Listener_; // how to use this in class?
};

VIS::VIS() : tf2Listener_(tfBuffer_)
{
  pubTwistRobot_  = nh_.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);

  controller_timer_ = nh_.createTimer(
                        ros::Duration(1/30.0), 
                        &VIS::timerCallback, this);
}

void VIS::timerCallback(const ros::TimerEvent& event)
{
  double KX = 1, KY = 1, KQ = 1;

  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer_.lookupTransform("fake_base_link","tag_0",
                                                ros::Time::now(),
                                                ros::Duration(0.5));
                                                // ros::Time(0));
    
    double px, py, pz;
    px = transformStamped.transform.translation.x;
    py = transformStamped.transform.translation.y;
    pz = transformStamped.transform.translation.z;
    
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(transformStamped.transform.rotation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // yaw = tf::getYaw(transform.getRotation());
    std::cout << "x: " << px << std::endl;
    std::cout << "y: " << py << std::endl;
    std::cout << "z: " << pz << std::endl;
    std::cout << "yaw: " << yaw << std::endl;
    std::cout << "------------------------" << std::endl;

    geometry_msgs::Twist twist;
    twist.linear.x = KX*px;
    twist.linear.y = KY*py;
    twist.angular.z = KQ*yaw;
    pubTwistRobot_.publish(twist);
  }
  // catch (tf::TransformException ex){
  catch (tf2::TransformException ex){
    ROS_ERROR("%s",ex.what());
    geometry_msgs::Twist twist;
    pubTwistRobot_.publish(twist);
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_vis");

  VIS vis;

  ros::spin();

  return 0;
}


