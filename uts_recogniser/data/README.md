# Instruction for data

## JSON

This file illustrates the environment set-ups in each bin

## Amazon Models

XML models for *RGB recogniser* and *RGBD recogniser*, not useful for current pipeline.

## Kernel Descriptor Models

Two MAT files are provided for kernel descriptor recogniser. 

**If the MAT files are loaded with error, the MAT files will be cleared. Keep a backup for these two files**

## Method

The suitable method for the items suitable for *RGBD recogniser* and *RGB recogniser*

## Sensor Data

1. Sensor information is generated from `apc_data_writer`
2. Mask images for each bin are
	* mask_camera_rgb_bin_<>.png
	* mask_xtion_depth_bin_<>.png
	* mask_xtion_rgb_bin_<>.png
3. Empty bin image
	* bin_<>_empty.png

index 1 is corresponding with bin_A and later follows the same pattern.
