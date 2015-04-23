#Amazon Picking Challenge

## How to install EBlearn
The eblearn package is in *uts_recogniser/externals/*. The installation instruction is [here](http://eblearn.sourceforge.net/install.html). Both eblearn core and tools are required to be installed. EBlearn does NOT provide default link file. If EBlearn is installed correct to */usr/include* and */usr/lib*, *eblearn.pc* is able to find EBlearn on your PC.

Please check the EBlearn is installed and tested successfully before compling the updated **uts_recogniser**.

## How to run eblearn test?

`../bin/eblearn_test ../data/eblearn/safety_works_safety_glasses.conf ../data/eblearn/images/boot-fc2_save_2015-04-11-220529-0000.bmp`

## How to run it?

Normally, I am more used to run ros package directly using binray file instead of `rosrun <package name> <binary name>`.

1. Test Kernel Descriptor at first:

	`../bin/kd_test ../data/201504174639/xtion_rgb_1.png ../data/201504174639/bin_A_empty.png ../data/201504174639/mask_xtion_rgb_bin_A.png feline_greenies_dental_treats mead_index_cards expo_dry_erase_board_eraser kong_duck_dog_toy`
	 

2. Test offline recogniser:

	`../bin/offline_recogniser -j ../data/201504171107.json -mask ../data/201504174639/ -method ../data/method.txt -kd ../data/kd_models/`
	
	`../bin/pseudo_request -d ../data/201504174639/ -n 12 -j ../data/201504171107.json`


Object recognition status:

	* R = Recognisable, correct
	* NR = Not Recognisable, will be recognised using Machine Learning
	* NA = Not available, cannot be purchased in Australia
	* O = Ordered, no available yet

| Object name                              | Status |
|------------------------------------------|--------|
| oreo_mega_stuf                           | NA     |
| champion_copper_plus_spark_plug          | R      |
| expo_dry_erase_board_eraser              | R      |
| genuine_joe_plastic_stir_sticks          | R      |
| munchkin_white_hot_duck_bath_toy         | NR     |
| crayola_64_ct                            | R      |
| mommys_helper_outlet_plugs               | NR     |
| sharpie_accent_tank_style_highlighters   | NR/R<sup>1</sup>   |
| stanley_66_052                           | NR     |
| safety_works_safety_glasses              | NR     |
| cheezit_big_original                     | R     |
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
