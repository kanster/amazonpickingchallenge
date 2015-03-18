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
![](http://s5.sinaimg.cn/middle/001WoJ8ogy6QNoF9qyo44&690)
![](http://s3.sinaimg.cn/middle/001WoJ8ogy6QNoF8m7Ec2&690)


##Development Log:

17/03/2015

Object recognition status:

	* R = Recognisable, correct
	* NR = Not Recognisable, will be recognised using Machine Learning
	* NA = Not available, cannot be purchased in Australia
	* O = Ordered, no available yet

| Object name                              | Status |
|------------------------------------------|--------|
| oreo_mega_stuf                           | NA     |
| champion_copper_plus_spark_plug          | O      |
| expo_dry_erase_board_eraser              | R      |
| genuine_joe_plastic_stir_sticks          | R      |
| munchkin_white_hot_duck_bath_toy         | NR     |
| crayola_64_ct                            | O      |
| mommys_helper_outlet_plugs               | NR     |
| sharpie_accent_tank_style_highlighters   | NR/R<sup>1</sup>   |
| stanley_66_052                           | NR     |
| safety_works_safety_glasses              | NR     |
| cheezit_big_original                     | NA     |
| paper_mate_12_count_mirado_black_warrior | R      |
| feline_greenies_dental_treats            | R<sup>2</sup>   |
| elmers_washable_no_run_school_glue       | R<sup>3</sup>   |
| mead_index_cards                         | R      |
| rolodex_jumbo_pencil_cup                 | NR     |
| first_years_take_and_toss_straw_cup      | NR     |
| highland_6539_self_stick_notes           | R      |
| mark_twain_huckleberry_finn              | R      |
| kyjen_squeakin_eggs_plush_puppies        | NR     |
| kong_sitting_frog_dog_toy                | NR     |
| kong_air_god_squeakair_tennis_ball       | NR     |
| dr_browns_bottle_brush                   | NR/R<sup>4</sup>   |
| kong_duck_dog_toy                        | NR     |
| laugh_out_loud_joke_book                 | R      |

Detailed explanations:
	
	1. *sharpie_accent_tank_style_highlighters* can be recognised only using the front view, using the label in the front.
	2. *feline_greenies_dental_treats* is non-rigid, however, this item can be recognised only if we don't bend the item too much. **model updated**
	3. *elmers_washable_no_run_school_glue* can be recognised, the model will be updated lated this week. **model updated**
	4. *dr_browns_bottle_brush* can roughly recognised using the texture on the paper board

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

09/03/2015

Communication problem between `data_publisher` and `recogniser` is solved.
The next frame of sensor data input will only be published if the `recogniser` send a feedback.



