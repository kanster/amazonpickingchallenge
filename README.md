#Compilation
* **Install external libraries** If you are using Ubuntu 12.04 64 bit system with ROS hydro, the pre-compiled version of linked libraries in `dependencies` should be work. In other cases, go to `external` directory, unzip the compressed documents and copy the compiled libraries into `dependencies`. Please using the static library in order to correctly compile the code
* **denpendencies** In order to correctly compile `levmar`, `f2c` package needs to be installed. Other required packages include: [BLAS](http://www.netlib.org/blas/) and [LAPACK](http://www.netlib.org/lapack/). Both of them can be easily installed using `apt-get` on Ubuntu.

- - -
#How to run uts recogniser?
Two kinds of sensors are involved in uts recogniser: a rgb camera(firewire camera in this case) 
and a rgbd sensor such as Kinect and Xtion. Start the two sensors before running this package.

You can test the code using `pseudo_target` which sends pseudo request to the `uts_recogniser`