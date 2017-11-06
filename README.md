# icub-grasp-demo

This is a demo for performing object modeling and grasping with superquadrics [[1]](#references) on the iCub robot. 


#### Overview
- [Demo description](#demo-description)
- [Use case](#use-case)
- [Documentation](#documentation)

## Demo description
This module implements a wrapper code for performing the superquadric modeling and grasping pipeline described in [[1]](#references).
This wrapper code communicates with existing modules developed in [`robotology repo`](https://github.com/robotology) and its structure can be summarized as follow:

<img src="https://github.com/robotology/icub-grasp-demo/blob/master/misc/pipeline.png" width=850 height=500> 



1) The robot checks  if a`box`, `sphere` or `cylinder` is in the field of view by querying the [object property collector](http://wiki.icub.org/brain/group__objectsPropertiesCollector.html) and acquires the 2D bounding box information of the object. If multiple objects are in front the robot, one of them is randomly chosen for performing the demo.
Note: The objects are classified in `box`, `sphere` or `cylinder` since they are primary shapes used for improving the object modeling. 
2) Given that, [`lbpExtract module`](https://github.com/robotology/segmentation) provides multiple 2D blobs of the selected object. The demo code sends the 2D blobs of the object to the [`Structure From Motion module`](https://github.com/robotology/stereo-vision) for getting the relative 3D point clouds.
3) The 3D point clouds are then sent to the [`superquadric-model`](https://github.com/robotology/superquadric-model) for computing several superquadrics modeling the object.
4) A median filter computes a model by avering all the reconstructed superquadrics. This approach leads to more stable and robust models.
5) The demo code sends the estimated superquadric to the [`superquadric-grasp module`](https://github.com/robotology/superquadric-grasp), which computes suitable poses for the right and the left hand.
6) The best hand for grasping the object is selected according to proper criteria.

7 - 8) Finally, the `superquadric-grasp` is asked to perform the grasping task.

[`Go to the top`](#icub-grasp-demo)
## Use case

#### How to compile
In Linux systems, the code can be compiled as follows:

```sh
git clone https://github.com/robotology/icub-grasp-demo.git
mkdir build; cd build
ccmake ..
make install
```


[`Go to the top`](#icub-grasp-demo)

#### How to run the code
This demo has been designed in order to be executed on the iCub robot automatically. If you are interested in a interactive mode for launching the grasping algorithm, please visit [this page](http://github.com/robotology/superquadric-grasp-example).

In order to run automatically the `icub-grasp-demo`, please:

1. Launch the `yarprobotinterface`.
2. Launch the `cameras` .
3. Launch the basic modules:`iKinGazeCtrl`, `iKinCartsianSolver`- for both right and left arm. 
4. Launch the [`skinManager`](https://github.com/robotology/icub-main/tree/master/src/modules/skinManager) and `skinManagerGui` and connect. Set the `binarization filter` `off` and the `compensation gain` and the `contact compensation gain` at the minimum values. 
5. Launch and connect all the modules required from the demo, which are collected in [this xml](https://github.com/robotology/icub-grasp-demo/blob/master/app/script/grasp-demo.xml.template).
6. The [`rfsmGui`](https://github.com/robotology/rfsmTools#testing-the-rfsmgui) will open. Play `run` on the Gui to start  the state machine executing the demo. More information about the state machine are provided [here](https://github.com/robotology/icub-grasp-demo/tree/master/app/lua).

[`Go to the top`](#icub-grasp-demo)

#### Setting up the demo
Before running the demo, it is recommended to correctly set up the modules. In particular:
- for the `superquadric-model`, we suggest to [calibrate the stereo vision](https://github.com/robotology/superquadric-model/tree/master/tutorial#calibrate-the-stereo-vision-through-the-sfm-module).
- and for  the `superquadric-grasp`, we recommend to calibrate the robot following [these instructions](https://github.com/robotology/superquadric-grasp/tree/master/tutorial#setting-up-before-running-on-the-robot).

[`Go to the top`](#icub-grasp-demo)

#### How to costum the demo
The `icub-grasp-demo` can be customized by the user by changing the configuration parameters of the [`superquadric-model`](http://robotology.github.io/superquadric-model/doxygen/doc/html/group__superquadric-model.html) and [`superquadric-grasp`](https://robotology.github.io/superquadric-grasp/doxygen/doc/html/group__superquadric-grasp.html) modules, in the proper configuration files (respectively: [config.ini](https://github.com/robotology/superquadric-grasp/blob/master/app/conf/config.ini) and [config.ini](https://github.com/robotology/superquadric-model/blob/master/app/conf/config-classes.ini)).

Some useful options for the `superquadric-grasp` module are the following:
- `lift_object`: available values: `on` (default) / `off`. If you want the robot to test if the pose is good enough for lifting the object.
- `grasp`: available values: `on`  (default) / `off`. If you want the robot to perform the grasp by using tactile feedback. If `off` is selected, the robot just reaches the desired pose, without closing the hand.
- `visual_servoing`: available values: `on` / `off` (default). If you want to reach for the pose by using a markerless visual servoing algorithm and an accurate hand pose estimation (more information are available [here](https://github.com/robotology/visual-tracking-control)). (Currently, visual servoing is available only for the right hand).
- `which_hand`: available values: `right`, `left`, or `both` (default). This variable represents the hand for which the grasping pose is computed. In case `both` is selected, the pose is computed for each hand and the best hand for grasping the object is automatically selected. If only one hand is chosen for the grasping pose computation, it will be consequently select also for grasping the object.


## Documentation
The online documentation of this module is available [here](http://robotology.github.com/icub-grasp-demo).









[`Go to the top`](#icub-grasp-demo)

# References
[1] G. Vezzani, U. Pattacini and L. Natale, "A grasping approach based on superquadric models", IEEE-RAS International Conference on Robotics and Automation 2017, pp 1579-1586.
