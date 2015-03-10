#Amazon Picking Challenge

## How to compile it?

1.  Compile the **apc_msgs** which is a must in **recogniser** and **data_publisher**
2.  **data_writer** is not necessary using **offline_recogniser**
3.  Compile **data_publisher**, for the detail of usage of data_publisher, go the sub-directory.
4.  Compile **recogniser**, if you are using ubunt 12.04 64 bit system with ROS Hydro, the libraries in **dependencies** folder should work, otherwise, compile the packages in **externals** folder and put the library in **dependencies** folder. For the detail explanation, go to sub-directory


## How to run it?

Normally, I am more used to run ros package directly using binray file instead of `rosrun <package name> <binary name>`.
1. Open a terminal, go to **data_publisher**, `./bin/data_publisher -d ../data/ -n 2`
2. Open another terminal, go to **recogniser**, `./bin/offline_recogniser -dir ../data/`

### Simple explanation




##Development Log:

  08/03/2015: 
  
  `data_writer` can save following topic information to disk by Press `c` button:
  
    * rgb image from xtion
    * rgb camera info from xtion, only projection matrix and distortion vector are saved to *yml* file, also for other camera info
    * depth image from xtion, 16bit
    * depth camera info from xtion
    * rgb point cloud from xtion
    * rgb image from rgb camera
    * rgb camera info from rgb camera
    
  `data_publisher` can publish saved data to specific topics. This is used as the bridge between sensor data and `recogniser`, the topic name are
  
    * /xtion/rgb/image
    * /xtion/rgb/camera_info
    * /xtion/depth/image
    * /xtion/depth/camera_info
    * /xtion/depth/points
    * /camera/image
    * /camera/camera_info
    
  ~~Current problem in `data_publisher` is the communication between `data_publisher` and `recogniser`. To be specific, **New frame of data should be published only after the recogniser has finished processing previous frame**.~~
  
  09/03/2015
  
  Communication problem between `data_publisher` and `recogniser` is solved.
  The next frame of sensor data input will only be published if the `recogniser` send a feedback.
  

  ~~Load the mask images and other information in the `recogniser`~~
  
  10/03/2015
  
  Successful tested using offline collected data


