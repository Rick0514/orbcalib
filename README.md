## Non-Overlap Camera pair calibration based on ORBSLAM3

### Intro.

Implementation of [1], different from open source code base [2], this version keeps no modification of orbslam3[3]. Calibration part is organized to folder ```/calib```.

### Dependencies

Same as [3].

### Build

1. put orb vocabulary to ```/Vocabulary/ORBVoc.txt```
2. prepare your camera config file to ```/config```
3. build all
```bash
cmake -B build
cmake --build build -- -j8
```

### Ref.

[1] J. Xu et al., "CamMap: Extrinsic Calibration of Non-Overlapping Cameras Based on SLAM Map Alignment," in IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 11879-11885, Oct. 2022, doi: 10.1109/LRA.2022.3207793.

[2] https://github.com/jiejie567/SlamForCalib

[3] https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/master