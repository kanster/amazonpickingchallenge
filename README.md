#How to build uts recogniser?
* External dependencies installation
* pr_msgs for communication

- - -
#How to run uts recogniser?
Two kinds of sensors are involved in uts recogniser: a rgb camera 
and a rgbd sensor such as Kinect and Xtion.

    roscore
    rosrun uvc_camera uvc_camera_node 
    roslaunch openni_launch openni.launch

You can test the code using `<pseudo_target`>


