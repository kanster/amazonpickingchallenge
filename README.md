#Amazon Picking Challenge

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
  
  **todo**
  
  Load the mask images and other information in the `recogniser`
  
  
  
  `recogniser` remains the same with online sensor data input

##How to run it?
