# amazonpickingchallenge
Amazon Picking Challenge

Development Log:

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
  
    
