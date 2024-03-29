# rov_visp

To run feature tracking ViSP (tracks the corners of the AprilTag):

    roslaunch rov_visp vis.launch

Tracking parameters such as tag ID, size, visual servo controller gain, can be set in the launch file.

The static transform from base to camera frame can be set in the launch file. TF lookup is used inside the code to determine the transform between the parent ("base_link") and child ("camera_link"). The frame names in static_transform_publisher in the launch file needs to be set manually to because ROS parameters results in the tf names having "/" which is invalid.

To run pose tracking (6D pose estimated from AprilTag and tracked with a P controller):

    rosrun rov_visp main
    
    
## References
1. [Example code](https://github.com/jokla/visp_cam_vs)
2. [ViSP bridge](https://docs.ros.org/en/api/visp_bridge/html/namespacemembers.html)
3. [Feature Point](https://visp-doc.inria.fr/doxygen/visp-2.9.0/classvpFeaturePoint.html#ac495bc37d8cf655a97cd095d62a8884f)
4. [Tutorial: IBVS](https://visp-doc.inria.fr/doxygen/visp-2.9.0/tutorial-ibvs.html)
5. [Tutorial: ViSP](https://visp-doc.inria.fr/doxygen/visp-daily/classvpServo.html)
