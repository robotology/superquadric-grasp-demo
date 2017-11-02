# icub-grasp-demo

This is a demo on how to perform object modeling and grasping with superquadrics [1]. 


#### Overview
- [Demo description](#demo-description)
- [Use case](#use-case)
 -[Documentation](#documentation)

## Demo description
This module implements a wrapper code for performing the superquadric modeling and grasping pipeline described in [[1]](#references).
This wrapper code communicates with existing modules developed in [`robotology repo`](https://github.com/robotology) and its structure can be summarized as follow:


<img src="https://github.com/robotology-playground/experiment-new-grasp/blob/master/experiment-1/misc/298761_Vezzani_Figure3.JPEG" width=400 height=300> <img src="https://github.com/robotology/icub-grasp-demo/blob/master/misc/pipeline.png" width=850 height=500> 



1) The robot checks  if a`box`, `sphere` or `cylinder` is in the field of view by querying the [object property collector](http://wiki.icub.org/brain/group__objectsPropertiesCollector.html) and acquires the 2D bounding box information of the object.
Note: `box`, `sphere` or `cylinder` are the primary shapes used for improving the object modeling. 
2) Given that, [`lbpExtract module`](https://github.com/robotology/segmentation) provides multiple 2D blobs of the object. The demo code sends the 2D blobs of the object to the [`Structure From Motion module`](https://github.com/robotology/stereo-vision) for getting the relative 3D point clouds.
3) The 3D point clouds are then sent to the [`superquadric-model`](https://github.com/robotology/superquadric-model) for computing several superquadrics modeling the object.
4) A median filter computes a model by avering all the reconstructed superquadrics. This approach leads to more stable and robust models.
5) The demo code sends the estimated superquadric to the [`superquadric-grasp module`](https://github.com/robotology/superquadric-grasp), which computes suitable poses for the right and the left hand.
6) The best hand for grasping the object is selected according to proper criteria.

7 - 8) Finally, the `superquadric-grasp` is asked to perform the grasping task.

[`Go to the top`](#superquadric-graps-example)
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
This demo has been designed in order to be executed on the iCub robot automatically. If you are interested in a interactive mode for launching the grasping algorithm, please visit [this page](http://github.com/robotology/superquadric-grasp-examp

In order to run automatically the `icub-grasp-demo`, please:

1. Launch the `yarprobotinterface`
2. Launch the basic modules:`iKinGazeCtrl`, `iKinCartsianSolver`- for both right and left arm. 
3. Launch the [`skinManager`](https://github.com/robotology/icub-main/tree/master/src/modules/skinManager) and `skinManagerGui` and connect. Set the `binarization filter` off and the `compensation gain` and the `contact compensation gain` at the minimum values. 
4. Launch and connect all the modules required from the demo, which are collected in [this xml](https://github.com/robotology/icub-grasp-demo/blob/master/app/script/grasp-demo.xml.template)
5(a). Launch the lua script:
```
cd app/lua
lua grasping_main.lua
```
5(b). Alternatively, you can launch the lua state machine with the [`rfsmGui`](https://github.com/robotology/rfsmTools#testing-the-rfsmgui). More information about the state machine are provided [here](https://github.com/robotology/icub-grasp-demo/tree/master/app/lua).

## Documentation
The online documentation of this module is available [here](http://robotology.github.com/icub-grasp-demo).









[`Go to the top`](#icub-grasp-demo)

# References
[1] G. Vezzani, U. Pattacini and L. Natale, "A grasping approach based on superquadric models", IEEE-RAS International Conference on Robotics and Automation 2017, pp 1579-1586.
