# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift
/**
* Bottle
*
* IDL structure to set/show advanced parameters.
*/
struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )

/**
* graspDemo_IDL
*
* IDL Interface to \ref grasp-demo services.
*/

service graspDemo_IDL
{

    /**
    * Send the current 2D blob
    *@return a bottle containing all the 2D points of the blob
    */
    Bottle  get_blob();
    
    /**
    * Set the name of the object
    * (stored by the object property collector).
    * @param entry is the name of the object
    * @return true/false on success/failure.
    */
    bool  set_object_name(1: string entry);

    /**
    * Get the name of the object
    * @return a string with the name of the object.
    */
    string  get_object_name();

    /**
    * Ask to go to the next step, following the pipeline:
    * 1- compute superquadric
    * 2- compute pose
    * 3- ask the robot to move
    * @return true.
    */
    bool go_next();

    /**
    * The pipeline is restarted and is 
    * waiting the command "go_next" to start
    * from step 1.
    * @return true.
    */
    bool start_from_scratch();

    /**
    *If you want just to perform step 1.
    * @return true.
    */
    bool acquire_superq();

    /**
    * Ask if the superquadric has been computed.
    * @return true/false on success/failure.
    */
    bool check_superq();
    
    /**
    *If you want just to perform step 2
    * (if step 1 has been performed).
    * @return true/false on success/failure.
    */
    bool compute_pose();

    /**
    * To ask if the pose has been computed
    * @return true/false on success/failure.
    */
    bool check_pose();
 
    /**
    *If you want just to perform step 3
    * (if step 2 has been performed).
    * @return true/false on success/failure.
    */
    bool grasp_object();

    /**
    *Ask the robot to stop and go back
    * to home position with the arm 
    * that is moving.
    * @return true/false on success/failure.
    */
    bool go_back_home();

    /**
    *Clear all the computed poses
    * @return true.
    */
    bool clear_poses();

    /**
    * Set the hand for pose computation.
    * @param entry can be "right", "left" or "both".
    * @return true/false on success/failure.
    */
    bool set_hand_for_computation(1: string entry);
    
    /**
    * Get the hand for pose computation.
    * @return "right", "left" or "both".
    */
    string get_hand_for_computation();
    
    /**
    * Set the hand for moving.
    * @param entry can be "right" or "left".
    * @return true/false on success/failure.
    */
    bool set_hand_for_moving(1: string entry);

    /**
    * Get the hand for pose computation.
    * @return "right", "left" or "both".
    */
    string get_hand_for_moving();

    /**
    * Get if automatic selection of the hand is on or off
    * @return "on" or "off".
    */
    string get_automatic_hand();

    /**
    * Set if automatic selection of the hand is on or off
    *@param entry can be "on" or "off"
    *@return true/false on success/failure.
    */
    bool set_automatic_hand(1: string entry);

    /**
    * Set if to ask the filtered superquadric or not.
    * @param entry can be "on" or "off".
    * @return "on" or "off".
    */
    bool set_filtered_superq(1: string entry);

    /**
    * Get if to ask the filtered superquadric or not.
    * @return true/false on success/failure.
    */
    string get_filtered_superq();
    
    /**
    * Set if to reset the filtered superquadric or not.
    * @param entry can be "on" or "off".
    * @return "on" or "off".
    */
    bool set_reset_filter(1: string entry);

    /**
    * Get if to reset the filtered superquadric or not.
    * @return true/false on success/failure.
    */
    string get_reset_filter();

    /**
    * Set the current object class
    * @param entry the name of the object class
    * @return true/false on success/failure.
    */
    bool set_object_class(1: string entry);

    /**
    * Get the current object class
    * @return the current object class.
    */
    string get_object_class();

}


