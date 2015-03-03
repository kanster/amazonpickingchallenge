#Compilation
* **Install external libraries** If you are using Ubuntu 12.04 64 bit system with ROS hydro, the pre-compiled version of linked libraries in `dependencies` should be work. In other cases, go to `external` directory, unzip the compressed documents and copy the compiled libraries into `dependencies`. Please using the static library in order to correctly compile the code
* **Dependencies** In order to correctly compile `levmar`, `f2c` package needs to be installed. Other required packages include: [BLAS](http://www.netlib.org/blas/) and [LAPACK](http://www.netlib.org/lapack/). Both of them can be easily installed using `apt-get` on Ubuntu.

- - -
#How to run the package?
* **Hardware configuration** An RGB-D sensor and an RGB sensor need to be correctly connected. Only Kinect is tested now. In order to get a reliable pose estimation results, both sensor needs to calibrated.
* **Configuration**
> * *Topic name* go to `src/uts_recogniser.cpp` and set the correct topics you want to subscribe include rgb image, depth image and point cloud from RGB-D sensor and rgb image from RGB sensor.
> * *JSON* the `json` file is in `data` folder using the same format given by Amazon Picking Challenge organiser
> * *method* in `data` folder, `method.txt` represents the correct(most optimal) method for different item
* **Run**
Start `pseudo_request` to send pseudo request for the target items and run `uts_recogniser` to start recognition process
