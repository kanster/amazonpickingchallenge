# data_publisher

## What are the published data

- xtion rgb image, with prefix *xtion_rgb_*
- xtion depth image, with prefix *xtion_depth_*
- xtion rgb point cloud, with prefix *xtion_cloud_*
- camera rgb image, with prefix *camera_rgb_*
- camera info of xtion rgb image, saved in yml, generated from opencv, check **data_writer** for detail, with prefix *xtion_rgb_info_*
- camera info of xtion depth image, saved in yml, generated from opencv, check **data_writer** for detail, with prefix *xtion_depth_info_*
- camera info of camera rgb image, saved in yml, generated from opencv, check **data_writer** for detail, with prefix *camera_rgb_info_*
 

There are another 3 types of image saved in the **../data** directory which are not published:
- mask image of xtion rgb image, which only contains the bin
- mask image of xtion depth image, which only contains the bin
- mask image of camera rgb image, which only contains the bin

The reason why the mask images are not published, but will be loaded in **offline_recogniser** is because in the actual competition, the mask images are generated from prepration phase therefore won't be able to published in real time.

## What are the args for publisher
Go to `print_usage` in **src/main.cpp**.
