# icub-grasp-demo

This is a demo on how to perform object modeling and grasping with superquadrics [1]. 


#### Overview
- [Demo description](#experiment-1)
- [Use case](#use-case)
 -[Documentation](#documentation)

## Demo description
This module implements a wrapper code for performing the superquadric modeling and grasping pipeline described in [[1]](#references).
This wrapper code communicates with existing modules developed in [`robotology repo`](https://github.com/robotology) and its structure can be summarized as follow:


<img src="https://github.com/robotology-playground/experiment-new-grasp/blob/master/experiment-1/misc/298761_Vezzani_Figure3.JPEG" width=400 height=300> <img src="https://github.com/robotology-playground/experiment-new-grasp/blob/master/experiment-1/misc/298761_Vezzani_Figure2.JPEG" width=450 height=300> 



1) The wrapper code asks the [object property collector](http://wiki.icub.org/brain/group__objectsPropertiesCollector.html) for the 2D bounding box information of the object.
2) Given that, [lbpExtract module](https://github.com/robotology/segmentation) provides the 2D blob of the object.
3) The wrapper code sends the 2D blob of the object to the [Structure From Motion module](https://github.com/robotology/stereo-vision) for getting the relative 3D point cloud.
4) The 3D point cloud is then sent to the [superquadric-model](https://github.com/robotology/superquadric-model) for computing the superquadric modeling the object.
5) The wrapper code sends the estimated superquadric to the [superquadric-grasp module](https://github.com/robotology/superquadric-grasp), which computes suitable poses.
6) Finally, the superquadric-grasp is asked to perform the grasping task.

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

## Documentation
The online documentation of this module is available [here](http://robotology.github.com/icub-grasp-demo).













[`Go to the top`](#icub-grasp-demo)

# References
[1] G. Vezzani, U. Pattacini and L. Natale, "A grasping approach based on superquadric models", IEEE-RAS International Conference on Robotics and Automation 2017, pp 1579-1586.
