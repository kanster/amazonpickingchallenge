# How to RUN apc_data_writer

Start an openni device and another PointGrey RGB camera.

1. roslaunch openni_launch openni.launch
2. roslaunch firefly_pgr firefly_pgr.launch

The RGB image and depth image from Xtion should be syncronized.

`./bin/apc_data_writer -d tmp/ -c 0`

Try `./bin/apc_data_writer -h` for detail information of the command line.

Press **s**  to save current frame.

This simple recorder is able to save sensor information to disk, used for `pseudo_request` in `uts_recogniser`.

## Important Notice

Except for the information collected in this program, you should also prepare:

1. mask images for each bin
2. sensor information for empty shelf

The filenames for above files, please go to *./uts_recogniser/data/* for more instructions.
