## Non-Overlap Camera pair calibration based on ORBSLAM3

### Intro.

Implementation of [1], different from open source code base [2], this version keeps no modification to orbslam3[3]. Calibration part is contained in folder ```/calib```.

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

### Usage

1. change ```calib.yaml``` and ```cam.yaml``` to run orbslam to save atlas first!!

```yaml
# calib.yaml
# slam or calib
Mode: slam

# 0 for mono
# 1 for stero
# 2 for rgbd
Camera1.Type: 2
Camera1.Image: "/usb_front/image"
Camera1.Depth: "/usb_front/depth/image_raw"

Camera2.Type: 2
Camera2.Image: "/usb_back/image"
Camera2.Depth: "/usb_back/depth/image_raw"
```

```yaml
# cam.yaml
# System.LoadAtlasFromFile: "atlas"
```

```bash
./build/calib/calib ./Vocabulary/ORBvoc.txt config/sim/calib.yaml config/sim/cam.yaml config/sim/cam.yaml
```

2. load atlas to conduct calibration!!

```yaml
# calib.yaml
# slam or calib
Mode: calib
```

```yaml
# cam.yaml
System.LoadAtlasFromFile: "atlas"
```

```
./build/calib/calib ./Vocabulary/ORBvoc.txt config/sim/rgbd2.yaml config/sim/rgbd2.yaml
``` 


### Modification to ORBSLAM3

Little modifications made to ORBSLAM3 for better usage, all modifications are leading with comments: ```// modify: xxx```
1. orb vocabulary only need to be loaded once.
2. windows in viewer(pangolin or opencv) should be binded to different names.

Keep in mind that we try to keep ORBSLAM3 untouched just like the original code.

### Advantages Compared with [2]

0. Easiest installation.
1. Nearly non-intrusive to [3] with software enginearing.
2. Ideas of [1] are all implemented at folder ```/calib```, much more organized than [2].
3. Seems to be more robust than [2] based on my two simulation datasets.
4. Play bag once and run calib forever leveraging the Atlas module provided by [3]

### Ref.

[1] J. Xu et al., "CamMap: Extrinsic Calibration of Non-Overlapping Cameras Based on SLAM Map Alignment," in IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 11879-11885, Oct. 2022, doi: 10.1109/LRA.2022.3207793.

[2] https://github.com/jiejie567/SlamForCalib

[3] https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/master


### Issue

1. Mono mode has bug to be fixed!!