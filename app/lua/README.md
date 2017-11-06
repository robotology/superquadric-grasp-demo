# State Machine

This is the render made by the `rfsmGui` of the state machine implemented for this demo:

<img src="https://github.com/robotology/icub-grasp-demo/blob/master/misc/state-machine.png" width = "700">

- The robot looks for a box, cylinder or sphere in the field of view (<b>`ST_LOOK_FOR_OBJECT`</b>).
- When the object is found, the `superquadric-module` acquires multiple superquadrics and filters them with the adaptive median filter (<b>`ST_ACQUIRE_SUPERQ`</b>).
- Once the superquadric is computed, the demo code checks if it is acceptable for computing the grasping pose (<b>`ST_CHECK_SUPERQ`</b>).
In particular, the code checks that the dimensions are bounded ( _i.e. each one <0.3, since the objects graspable by the robot are quite small)_ and the superquadric position is reasonable (_i.e. in the robot workspace_). If these checks are not passed, the superquadric is likely to be wrong, maybe to wrong object segmentation or noisy point clouds. If the superquadric is not acceptable, the robot looks again for an object and repeats the previous steps (<b>`ST_LOOK_FOR_OBJECT`</b>).
- The superquadric is used by the `superquadric-grasp` module to compute the grasping candidates for the right and the left hand (<b>`ST_COMPUTE_POSE`</b>). 
- The code checks if the pose are far away enough from the table on which the object is located (<b>`ST_CHECK_POSE`</b>). If the grasping pose are not safe for executing the grasp, a new superquadric is acquired and the previous steps are repeated (<b>`ST_ACQUIRE_SUPERQ`</b>).
- Then, the best hand for grasping the object is chosen and the movements are executed (<b>`ST_GRASP_OBJECT`</b>).
- When the grasping ends, the robot hand goes back to the home position (<b>`ST_GO_HOME`</b>).
- Once the home position is reached (<b>`ST_CHECK_HOME`</b>), the robot is ready to start again (<b>`ST_LOOK_FOR_OBJECT`</b>).

