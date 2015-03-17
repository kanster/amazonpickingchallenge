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

The **data_publisher** will load the collected data in the given directory frame by frame iteratively. In the repo, there are only two frame of data, here we assume the frame 1 is from `bin_A` and frame 2 is from `bin_B`, check the **recogniser/data/amazon.json** for detail environment configuration of the shelf. Based on the sending request from **data_publisher**, to be specifically, the frame index in the request, **offline_recogniser** is able to know what the id of the bin and further obtain the items in the bin. Based on the **recogniser/data/methods.txt**, the **offline_recogniser** which **recogniser** (here we mean rgb or rgbd) it will use to recogniser the item.

Some screenshot of the results are:
![](http://d.pcs.baidu.com/thumbnail/f3bb28ddf7cd61a44f64d2fc4cb5fcbb?fid=2587132861-250528-882813956686291&time=1425978000&rt=pr&sign=FDTAER-DCb740ccc5511e5e8fedcff06b081203-1VUSZ%2bnN1oNBnoFcsCKDIkt84O4%3d&expires=8h&prisign=unknow&chkbd=0&chkv=0&size=c10000_u10000&quality=90)
![](http://d.pcs.baidu.com/thumbnail/f9215859a53006193a8448e093a8bd98?fid=2587132861-250528-526655755935820&time=1425978000&rt=pr&sign=FDTAER-DCb740ccc5511e5e8fedcff06b081203-Bvh7BgnZMJF7RSER18P41QCuldM%3d&expires=8h&prisign=unknow&chkbd=0&chkv=0&size=c10000_u10000&quality=90)


##Development Log:
	Object recognition status:

		* R = Recognisable, correct
		* NR = Not Recognisable, will be recognised using Machine Learning
		* NA = Not available, cannot be purchased in Australia
		* O = Ordered, no available yet


| Object name    | Status |
|----------------|--------|
| oreo_mega_stuf | NA     |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |
|                |        |


	


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


