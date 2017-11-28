/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
#include <cmath>
#include <string>
#include <sstream>
#include <deque>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <opencv2/opencv.hpp>

#include "src/graspDemo_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/**
 * @brief The  GraspDemo class provides a wrapper for the superquadric
 * modeling and grasping frameworks.
 * It implements a state machine, that communicates with external modules,
 * including the superquadric-model and -grasp modules,
 * for making the iCub grasping an unknown object.
 */
class GraspDemo : public RFModule,
                    graspDemo_IDL
{
    string hand_for_computation;
    string hand_for_moving;
    string method;
    string objname;

    vector<cv::Point> contour;
    deque<cv::Point> blob_points;
    deque<Vector> points;

    RpcClient portBlobRpc;
    RpcClient portOPCrpc;
    RpcClient portSFMRpc;
    RpcClient superqRpc;
    RpcClient graspRpc;
    RpcServer portRpc;

    Mutex mutex;

    Property superq;
    Vector object;
    Vector superq_aux;
    Vector plane;

    Bottle superq_b;
    string object_class;

    int n_pc;
    bool go_on;
    bool reset;
    bool online;   
    bool go_home;
    bool filtered;
    bool go_basket;
    bool choose_hand;
    bool ok_acq;
    bool ok_acq_pose;
    bool superq_ok;
    bool pose_ok;
    bool superq_received;
    bool pose_received;
    bool robot_moving;

    ResourceFinder *rf;

    ImageOf<PixelRgb> *ImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgIn;

public:

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /**
    * Send the current 2D blob
    *@return a bottle containing all the 2D points of the blob
    */
    /************************************************************************/
    Bottle  get_Blob()
    {
        Bottle blob;
        for (size_t i=0; i<blob_points.size(); i++)
        {
            Bottle &b=blob.addList();
            b.addDouble(blob_points[i].x); b.addDouble(blob_points[i].y);
        }

        return blob;
    }

    /**
    * Set the name of the object
    * (stored by the object property collector).
    * @param entry is the name of the object
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool set_object_name(const string &object_name)
    {
        LockGuard lg(mutex);
        objname=object_name;
        method="name";

        return true;
    }

    /**
    * Get the name of the object
    * @return a string with the name of the object.
    */
    /************************************************************************/
    string get_object_name()
    {
        return objname;
    }

    /**
    * Set the hand for pose computation.
    * @param entry can be "right", "left" or "both".
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool set_hand_for_computation(const string &h)
    {
        LockGuard lg(mutex);
        if ((h=="right") || (h=="left") || (h=="both"))
        {
            hand_for_computation=h;

            return true;
        }
        else
            return false;
    }

    /**
    * Get the hand for pose computation.
    * @return "right", "left" or "both".
    */
    /************************************************************************/
    string get_hand_for_computation()
    {
        return hand_for_computation;
    }

    /**
    * Set the hand for moving.
    * @param entry can be "right" or "left".
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool set_hand_for_moving(const string &h)
    {
        LockGuard lg(mutex);
        if ((h=="right") || (h=="left"))
        {
            hand_for_moving=h;

            return true;
        }
        else
            return false;
    }


    /**
    * Get the hand for pose computation.
    * @return "right", "left" or "both".
    */
    /************************************************************************/
    string get_hand_for_moving()
    {
        return hand_for_moving;
    }

    /**
    * Get if automatic selection of the hand is on or off
    * @return "on" or "off".
    */
    /************************************************************************/
    string get_automatic_hand()
    {
        if (choose_hand)
            return "on";
        else
            return "off";
    }

    /**
    * Set if automatic selection of the hand is on or off
    *@param entry can be "on" or "off"
    *@return true/false on success/failure.
    */
    /************************************************************************/
    bool set_automatic_hand(const string &entry)
    {
        if (entry=="on" || entry=="off")
        {
            choose_hand=(entry=="on");
            return true;
        }
        else
            return false;
    }

    /**
    * Ask to go to the next step, following the pipeline:
    * 1- compute superquadric
    * 2- compute pose
    * 3- ask the robot to move
    * @return true.
    */
    /************************************************************************/
    bool go_next()
    {
        go_on=true;

        return true;
    }

    /**
    * The pipeline is restarted and is
    * waiting the command "go_next" to start
    * from step 1.
    * @return true.
    */
    /************************************************************************/
    bool start_from_scratch()
    {
        go_on=false;
        superq_received=false;
        pose_received=false;
        robot_moving=false;
        superq_ok=false;
        pose_ok=false;
        ok_acq=false;
        ok_acq_pose=false;

        return true;
    }

    /**
    *If you want just to perform step 1.
    * @return true.
    */
    /************************************************************************/
    bool acquire_superq()
    {
        go_on=true;
        superq_received=false;
        pose_received=false;

        return true;
    }

    /**
    * To ask if the superquadric has been computed
    * @return true/false on success/failure.
    */
    /************************************************************************/
    string check_superq()
    {
        yDebug()<<"check_superq";
        yDebug()<<"superq rec"<<superq_ok;
        yDebug()<<"superq rec"<<ok_acq;
        if (ok_acq==false)
        {
            return "wait";
        }
        else
        {
            if (superq_ok)
                return "ok";
            else
                return "again";
        }
    }

    /**
    *If you want just to perform step 2
    * (if step 1 has been performed).
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool compute_pose()
    {
        if (superq_received==true)
        {
            go_on=true;
            pose_received=false;

            return true;
        }
        else
            return false;
    }

    /**
    * To ask if the pose has been computed
    * @return true/false on success/failure.
    */
    /************************************************************************/
    string check_pose()
    {
        
        yDebug()<<"check_pose";
        yDebug()<<"pose rec"<<pose_ok;
        if (ok_acq_pose==false)
        {
            return "wait";
        }
        else
        {
            if (pose_ok)
                return "ok";
            else
            {
                yDebug()<<"returning again";
                return "again";
            }
        }
    }


    /**
    *If you want just to perform step 3
    * (if step 2 has been performed).
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool grasp_object()
    {
        if (pose_received==true)
        {
            go_on=true;
            robot_moving=false;
            go_home=false;

            return true;
        }
        else
        {
            return false;
        }
    }

    /**
    *Ask the robot to stop and go back
    * to home position with the arm
    * that is moving.
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool go_back_home()
    {
        go_home=true;
        go_basket=false;

        robot_moving=true;

        go_on=false;

        return true;
    }

    /**
    * Ask the robot to put the object in a box on its side.
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool go_to_basket()
    {
        go_basket=true;
        go_home=false;

        robot_moving=true;

        go_on=false;

        return true;
    }

    /**
    *Clear  the computed superq
    * @return true.
    */
    /************************************************************************/
    bool clear_poses()
    {
        Bottle cmd, reply;
        cmd.addString("clear_poses");

        graspRpc.write(cmd, reply);

        superq_received=false;
    
        pose_received=false;

        return true;
    }

    /**
    *Clear all the computed poses
    * @return true.
    */
    /************************************************************************/
    bool clear_superq()
    {
        Bottle cmd, reply;
        superq_received=false;

        return true;
    }

    /**
    * Set if to ask the filtered superquadric or not.
    * @param entry can be "on" or "off".
    * @return "on" or "off".
    */
    /************************************************************************/
    bool set_filtered_superq(const string &entry)
    {
        if ((entry=="on") || (entry=="off"))
        {
            filtered=(entry=="on");
            return true;
        }
        else
            return false;          
    }

    /**
    * Get if to ask the filtered superquadric or not.
    * @return true/false on success/failure.
    */
    /************************************************************************/
    string get_filtered_superq()
    {
        if (filtered)
            return "on";
        else
            return "off";
    }

    /**
    * Set if to reset the filtered superquadric or not.
    * @param entry can be "on" or "off".
    * @return "on" or "off".
    */
    /************************************************************************/
    bool set_reset_filter(const string &entry)
    {
        if ((entry=="on") || (entry=="off"))
        {
            reset=(entry=="on");
            return true;
        }
        else
            return false;
    }

    /**
    * Get if to reset the filtered superquadric or not.
    * @return true/false on success/failure.
    */
    /************************************************************************/
    string get_reset_filter()
    {
        if (reset)
            return "on";
        else
            return "off";
    }

    /**
    * Set the current object class
    * @param entry the name of the object class
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool set_object_class(const string &entry)
    {
        object_class=entry;

        if (object_class == "box" || object_class == "sphere" || object_class == "cylinder")
            objname=entry;

        Bottle cmd, reply;
        cmd.addString("set_object_class");
        cmd.addString(entry);
        superqRpc.write(cmd, reply);

        return (reply.get(0).asString()=="ok");
    }

    /**
    * Get the current object class
    * @return the current object class.
    */
    /************************************************************************/
    string get_object_class()
    {
        return object_class;
    }

    /**
    * Set the RFModule period
    * @return the value of the period.
    */
    /************************************************************************/
    bool getperiod()
    {
        return 0.0;
    }

    /**
    * Calibrate plane height via superquadric computation
    * @return true/false on success/failure.
    */
    /************************************************************************/
    bool calibrate()
    {
        bool ok_calibration=false;

        ImgIn=portImgIn.read();

        if (blob_points.size()>1)
        {
            points=get3Dpoints(ImgIn);
        }

        Bottle cmd;
        cmd.addString("get_superq");

        Bottle &in1=cmd.addList();

        for (size_t i=0; i<points.size(); i++)
        {
            Bottle &in=in1.addList();
            in.addDouble(points[i][0]);
            in.addDouble(points[i][1]);
            in.addDouble(points[i][2]);
            in.addDouble(points[i][3]);
            in.addDouble(points[i][4]);
            in.addDouble(points[i][5]);
        }

        go_on=false;

        ok_acq=false;
        //ok_acq=superqRpc.write(cmd, superq_b);
        superqRpc.write(cmd, superq_b);

        yInfo()<<"Received superquadric: "<<superq_b.toString();

        Vector sup(11,0.0);
        sup=getBottle(superq_b, cmd);

        if (!superq_b.isNull())
        {
            ok_acq=true;

            if (superq_b.size()>0 && norm(sup.subVector(0,7))>0.0)
           {
                if (superqOk(sup))
                    superq_ok=superq_received=true;
                else
                    superq_ok=superq_received=false;
            }
            else
            {
                yError()<<"Zero superquadric";
                superq_ok=superq_received=false;
            }
        }

        if (superq_ok)
        {
            yDebug()<<"Plane height: "<<sup[7];
        }

        if (sup[7]<0.0 && sup[7]>-0.18)
        {
            Bottle cmd, reply_pose;
            cmd.addString("set_options");

            Bottle &content=cmd.addList();
            content.addString("plane");
            Bottle &plane_bottle=content.addList();
            plane_bottle.addDouble(0.0); plane_bottle.addDouble(0.0); plane_bottle.addDouble(1.0); plane_bottle.addDouble(-sup[7]);

            cmd.addString("pose");

            yInfo()<<"Command asked "<<cmd.toString();

            ok_calibration=graspRpc.write(cmd, reply_pose);

            yDebug()<<"Ok set plane "<<ok_calibration;

        }

        start_from_scratch();

        return ok_calibration;
    }

    /**
    * Configure method of the RFModule
    * @return true/false on success/failure.
    */
    /**********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        this->rf=&rf;
        method=rf.find("method").asString().c_str();
        if (rf.find("method").isNull())
            method="name";

        objname=rf.find("object_name").asString().c_str();
        if (rf.find("object_name").isNull())
            objname="object";

        filtered=(rf.check("filtered", Value("off")).asString()=="on");
        reset=(rf.check("reset", Value("off")).asString()=="on");
        hand_for_computation=rf.check("hand_for_computation", Value("both")).asString();
        hand_for_moving=rf.check("hand_for_moving", Value("right")).asString();
        choose_hand=rf.check("choose_hand", Value(false)).asBool();

        n_pc=rf.check("num_point_cloud", Value(5)).asInt();

        online=(rf.check("online", Value("off")).asString()=="on");

        if (online==false)
        {
            readSuperq("object", object, 11, this->rf);
            superq=fillProperty(object);
        }

        portBlobRpc.open("/grasp-demo/blob:rpc");
        portOPCrpc.open("/grasp-demo/OPC:rpc");
        portSFMRpc.open("/grasp-demo/SFM:rpc");
        superqRpc.open("/grasp-demo/superq:rpc");
        graspRpc.open("/grasp-demo/grasp:rpc");
        portRpc.open("/grasp-demo/rpc");
        portImgIn.open("/grasp-demo/img:i");

        attach(portRpc);

        go_on=false;
        go_home=false;
        superq_ok=false;
        pose_ok=false;
        go_basket=false;
        superq_received=false;
        pose_received=false;
        robot_moving=false;

        superq_aux.resize(12,0.0);

        return  true;
    }

    /**
    * Close all the ports
    * @return true.
    */
    /**********************************************************************/
    bool close()
    {
        if (portBlobRpc.asPort().isOpen())
            portBlobRpc.close();
        if (portOPCrpc.asPort().isOpen())
            portOPCrpc.close();
        if (portSFMRpc.asPort().isOpen())
            portSFMRpc.close();
        if (superqRpc.asPort().isOpen())
            superqRpc.close();
        if (graspRpc.asPort().isOpen())
            graspRpc.close();
        if (!portImgIn.isClosed())
            portImgIn.close();

        return true;
    }

    /**
    * Execute the state machine and the communications with external modules
    * @return true/false on success/failure.
    */
    /**********************************************************************/
    bool updateModule()
    {
        if (online==true)
        {
            if (method=="point")
            {
                blob_points.clear();

                if (contour.size()>0)
                {
                    getBlob();
                }
                else
                {
                    blob_points.push_back(cv::Point(0,0));
                }
            }
            else if (method=="name")
            {
                pointFromName();

                if ((contour.size()>0) )
                {
                    getBlob();
                }
                else
                {
                    blob_points.push_back(cv::Point(0,0));
                }
            }
        }

        
        if ((go_on==true) && (superq_received==false) && (online==true))
        {
            ImgIn=portImgIn.read(false);

            if (blob_points.size()>1)
            {
                points=get3Dpoints(ImgIn);           
            }

            if (!filtered)
            {
                Bottle cmd;
                cmd.addString("get_superq");

                Bottle &in1=cmd.addList();

                for (size_t i=0; i<points.size(); i++)
                {
                    Bottle &in=in1.addList();
                    in.addDouble(points[i][0]);
                    in.addDouble(points[i][1]);
                    in.addDouble(points[i][2]);
                    in.addDouble(points[i][3]);
                    in.addDouble(points[i][4]);
                    in.addDouble(points[i][5]);
                }

                go_on=false;

                ok_acq=false;
                //ok_acq=superqRpc.write(cmd, superq_b);
                superqRpc.write(cmd, superq_b);

                yInfo()<<"Received superquadric: "<<superq_b.toString();

                Vector sup(11,0.0);
                sup=getBottle(superq_b, cmd);
                
                if (!superq_b.isNull())
                {
                    ok_acq=true;
                    
                    if (superq_b.size()>0 && norm(sup.subVector(0,7))>0.0)
                   {
                        if (superqOk(sup))
                            superq_ok=superq_received=true;
                        else
                            superq_ok=superq_received=false;
                    }
                    else
                    {
                        yError()<<"Zero superquadric";
                        superq_ok=superq_received=false;                       
                    }
                }
                else
                {
                    yError()<<"Null superquadric";  
                    ok_acq=false;
                }
            }
            else
            {
                Bottle cmd, reply;

                if (reset)
                {
                    cmd.addString("reset_filter");
                    superqRpc.write(cmd, reply);
                }

                cmd.clear();
                reply.clear();

                for (size_t k=0; k<n_pc; k++)
                {
                    cmd.addString("send_point_clouds");

                    deque<Vector> pc;

                    Bottle &in0=cmd.addList();

                    pc.clear();


                    if (blob_points.size()>1)
                    {
                        pc=get3Dpoints(ImgIn);
                    }

                    for (size_t i=0; i<pc.size(); i++)
                    {
                        Bottle &in=in0.addList();
                        in.addDouble(pc[i][0]);
                        in.addDouble(pc[i][1]);
                        in.addDouble(pc[i][2]);                        
                    }

                    superqRpc.write(cmd, reply);

                    cmd.clear();
                    reply.clear();
                }

                cmd.addString("get_superq_filtered");

                go_on=false;

                ok_acq=false;
                ok_acq=superqRpc.write(cmd, superq_b);

                yInfo()<<"Received superquadric: "<<superq_b.toString();

                Vector sup(11,0.0);
                sup=getBottle(superq_b, cmd);
               
                if (!superq_b.isNull())
                {
                    if (superq_b.size()>0 && norm(sup.subVector(0,7))>0.0)
                   {
                        if (superqOk(sup))
                            superq_ok=superq_received=true;
                        else
                            superq_ok=superq_received=false;
                    }
                    else
                    {
                        yError()<<"Zero superquadric";
                        superq_ok=superq_received=false;                       
                    }
                }
                else
                    yError()<<"Null superquadric";
            }
        }
        else if (online==false)
            superq_received=true;

        if ((go_on==true) && (superq_received==true) && (pose_received==false))
        {
            Bottle cmd, reply_pose;
            cmd.addString("get_grasping_pose");

            getBottle(superq_b, cmd);

            cmd.addString(hand_for_computation);

            yInfo()<<"Command asked "<<cmd.toString();

            ok_acq_pose=false;
            ok_acq_pose=graspRpc.write(cmd, reply_pose);

            yInfo()<<"Received solution: "<<reply_pose.toString();

            getPlane();

            if (reply_pose.size()>0)
            {
                if (poseOk(reply_pose))
                    pose_received=pose_ok=true;
                else
                    pose_received=pose_ok=false;
                
                yDebug()<<"pose rec "<<pose_received;
            }
            //else
            //    pose_received=false;

            if (choose_hand)
            {
                cmd.clear();
                Bottle reply;
                cmd.addString("get_best_hand");
                graspRpc.write(cmd,reply);

                if (reply.get(0).asString()=="right" || reply.get(0).asString()=="left")
                {
                    hand_for_moving=reply.get(0).asString();
                    yInfo()<<"Best hand for grasping the object: "<<reply.get(0).asString();
                }
                else
                    yError()<<"No best pose received!";

            }
yDebug()<<"pose rec "<<pose_received;

            go_on=false;
        }

        if ((go_on==true) && (pose_received==true) && (robot_moving==false))
        {
            Bottle cmd, reply;
            cmd.clear();
            cmd.addString("move");
            cmd.addString(hand_for_moving);

            yInfo()<<"Asked to move: "<<cmd.toString();

            graspRpc.write(cmd, reply);

            if (reply.get(0).asString()=="ok")
            {
                yInfo()<<"The robot is moving";
                robot_moving=true;
            }

            go_on=false;
            superq_ok=false;
            pose_ok=false;
        }

        if (go_home==true)
        {
            Bottle cmd, reply;
            cmd.clear();
            cmd.addString("go_home");
            cmd.addString(hand_for_moving);

            yInfo()<<"Asked to stop: "<<cmd.toString();

            graspRpc.write(cmd, reply);

            go_home=false;
        }
        else if (go_basket==true)
        {
            Bottle cmd, reply;
            cmd.clear();
            cmd.addString("go_to_basket");
            cmd.addString(hand_for_moving);

            yInfo()<<"Asked to stop: "<<cmd.toString();

            graspRpc.write(cmd, reply);

            go_basket=false;
        }

        return true;
    }

    /**
    * Acquire 2D blob of the object.
    */
    /***********************************************************************/
    void getBlob()
    {
        Bottle cmd,reply;
        blob_points.clear();
        
        cmd.addString("get_component_around");
        cmd.addInt(contour[0].x); cmd.addInt(contour[0].y);

        if (portBlobRpc.write(cmd,reply))
        {           
            if (Bottle *blob_list=reply.get(0).asList())
            {
                for (int i=0; i<blob_list->size();i++)
                {
                    if (Bottle *blob_pair=blob_list->get(i).asList())
                    {
                        blob_points.push_back(cv::Point(blob_pair->get(0).asInt(),blob_pair->get(1).asInt()));
                    }
                    else
                    {
                        yError()<<"Some problems in blob pixels!";
                    }
                }
            }
            else
            {
                yError()<<"Some problem  in object blob!";
            }
        }
        else
        {
            yError("lbpExtract query is fail!");
        }
    }

    /**
    * Get object 2D center from its name.
    */
    /***********************************************************************/
    void pointFromName()
    {
        Bottle cmd,reply;
        blob_points.clear();
        contour.clear();
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content=cmd.addList();
        Bottle &cond_1=content.addList();
        cond_1.addString("entity");
        cond_1.addString("==");
        cond_1.addString("object");
        content.addString("&&");
        Bottle &cond_2=content.addList();
        cond_2.addString("name");
        cond_2.addString("==");
        cond_2.addString(objname);

        portOPCrpc.write(cmd,reply);
        if(reply.size()>1)
        {
            if(reply.get(0).asVocab()==Vocab::encode("ack"))
            {
                if (Bottle *b=reply.get(1).asList())
                {
                    if (Bottle *b1=b->get(1).asList())
                    {
                        cmd.clear();
                        int id=b1->get(0).asInt();
                        cmd.addVocab(Vocab::encode("get"));
                        Bottle &info=cmd.addList();
                        Bottle &info2=info.addList();
                        info2.addString("id");
                        info2.addInt(id);
                        Bottle &info3=info.addList();
                        info3.addString("propSet");
                        Bottle &info4=info3.addList();
                        info4.addString("position_2d_left");
                    }
                    else
                    {
                        yError("no object id provided by OPC!");
                        contour.clear();
                    }
                }
                else
                {
                    yError("uncorrect reply from OPC!");
                    contour.clear();
                }

                Bottle reply;
                if (portOPCrpc.write(cmd,reply))
                {

                    if (reply.size()>1)
                    {
                        if (reply.get(0).asVocab()==Vocab::encode("ack"))
                        {
                            if (Bottle *b=reply.get(1).asList())
                            {
                                if (Bottle *b1=b->find("position_2d_left").asList())
                                {
                                    cv::Point p1,p2,p;
                                    p1.x=b1->get(0).asInt();
                                    p1.y=b1->get(1).asInt();
                                    p2.x=b1->get(2).asInt();
                                    p2.y=b1->get(3).asInt();
                                    p.x=p1.x+(p2.x-p1.x)/2;
                                    p.y=p1.y+(p2.y-p1.y)/2;
                                    contour.push_back(p);
                                }
                                else
                                {
                                    yError("position_2d_left field not found in the OPC reply!");
                                    contour.clear();
                                }
                            }
                            else
                            {
                                yError("uncorrect reply structure received!");
                                contour.clear();
                            }
                        }
                        else
                        {
                            yError("Failure in reply for object 2D point!");
                            contour.clear();
                        }
                    }
                    else
                    {
                        yError("reply size for 2D point less than 1!");
                        contour.clear();
                    }
                }
                else
                    yError("no reply from second OPC query!");
            }
            else
            {
                yError("Failure in reply for object id!");
                contour.clear();
            }
        }
        else
        {
            yError("reply size for object id less than 1!");
            contour.clear();
        }
    }

    /**
    * Get 3D points by querying SFM.
    * @return a deque of Vectors with the point cloud
    */
    /***********************************************************************/
    deque<Vector> get3Dpoints(ImageOf<PixelRgb>  *ImgIn)
    {
        Bottle cmd,reply;
        cmd.addString("Points");
        int count_blob=0;

        deque<Vector> p;

        p.clear();

        for (size_t i=0; i<blob_points.size(); i++)
        {
            cv::Point single_point=blob_points[i];
            cmd.addInt(single_point.x);
            cmd.addInt(single_point.y);
        }

        if (portSFMRpc.write(cmd,reply))
        {
            count_blob=0;

            for (int idx=0;idx<reply.size();idx+=3)
            {
                Vector point(6,0.0);

                point[0]=reply.get(idx+0).asDouble();
                point[1]=reply.get(idx+1).asDouble();
                point[2]=reply.get(idx+2).asDouble();

                if (ImgIn!=NULL && (filtered==false))
                {
                    PixelRgb px=ImgIn->pixel(blob_points[count_blob].x,blob_points[count_blob].y);
                    point[3]=px.r;
                    point[4]=px.g;
                    point[5]=px.b;
                }
                //else
                 //   yInfo()<<"No img received yet!";

                count_blob+=1;

                if ((norm(point)>0))
                {
                    p.push_back(point);
                }
            }

            if (points.size()<=0)
            {
                yError("[SuperqComputation]: Some problems in point acquisition!");
            }
        }
        else
        {
            yError("[SuperqComputation]: SFM reply is fail!");
            p.clear();
        }

        return p;
    }

    /**
    * Read the superquadric from the configuration file, in offline mode
    * @return true.
    */
    /****************************************************************/
    bool readSuperq(const string &name_obj, Vector &x, const int &dimension, ResourceFinder *rf)
    {
        if (Bottle *b=rf->find(name_obj.c_str()).asList())
        {
            if (b->size()>=dimension)
            {
                for(size_t i=0; i<b->size();i++)
                    x.push_back(b->get(i).asDouble());
            }
            return true;
        }
    }

    /**
    * Fill a property with the solutions inside
    * @return a property with the computed solutions.
    */
    /**********************************************************************/
    Property fillProperty(const Vector &sol)
    {
        Property superq;

        Bottle bottle;
        Bottle &b1=bottle.addList();
        b1.addDouble(sol[0]); b1.addDouble(sol[1]); b1.addDouble(sol[2]);
        superq.put("dimensions", bottle.get(0));

        Bottle &b2=bottle.addList();
        b2.addDouble(sol[3]); b2.addDouble(sol[4]);
        superq.put("exponents", bottle.get(1));

        Bottle &b3=bottle.addList();
        b3.addDouble(sol[5]); b3.addDouble(sol[6]); b3.addDouble(sol[7]);
        superq.put("center", bottle.get(2));

        Bottle &b4=bottle.addList();
        Vector orient=dcm2axis(euler2dcm(sol.subVector(8,10)));
        b4.addDouble(orient[0]); b4.addDouble(orient[1]); b4.addDouble(orient[2]); b4.addDouble(orient[3]);
        superq.put("orientation", bottle.get(3));
        return superq;
    }

    /**
    * Process bottle with the superquadric.
    */
    /**********************************************************************/
    Vector  getBottle(Bottle &estimated_superq, Bottle &cmd)
    {
        Bottle *all=estimated_superq.get(0).asList();

        for (size_t i=0; i<all->size(); i++)
        {
            Bottle *group=all->get(i).asList();
            if (group->get(0).asString() == "dimensions")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[0]=dim->get(0).asDouble(); superq_aux[1]=dim->get(1).asDouble(); superq_aux[2]=dim->get(2).asDouble();
            }
            else if (group->get(0).asString() == "exponents")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[3]=dim->get(0).asDouble(); superq_aux[4]=dim->get(1).asDouble();
            }
            else if (group->get(0).asString() == "center")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[5]=dim->get(0).asDouble(); superq_aux[6]=dim->get(1).asDouble(); superq_aux[7]=dim->get(2).asDouble();
            }
            else if (group->get(0).asString() == "orientation")
            {


                 Bottle *dim=group->get(1).asList();

                 superq_aux[8]=dim->get(0).asDouble(); superq_aux[9]=dim->get(1).asDouble(); superq_aux[10]=dim->get(2).asDouble(); superq_aux[11]=dim->get(3).asDouble();
            }

        }
                
        /*Bottle *dim=estimated_superq.find("dimensions").asList();

        if (!estimated_superq.find("dimensions").isNull())
        {
            superq_aux[0]=dim->get(0).asDouble(); superq_aux[1]=dim->get(1).asDouble(); superq_aux[2]=dim->get(2).asDouble();
        }

        Bottle *shape=estimated_superq.find("exponents").asList();

        if (!estimated_superq.find("exponents").isNull())
        {
            superq_aux[3]=shape->get(0).asDouble(); superq_aux[4]=shape->get(1).asDouble();
        }

        Bottle *exp=estimated_superq.find("exponents").asList();

        if (!estimated_superq.find("exponents").isNull())
        {
            superq_aux[3]=exp->get(0).asDouble(); superq_aux[4]=exp->get(1).asDouble();
        }

        Bottle *center=estimated_superq.find("center").asList();

        if (!estimated_superq.find("center").isNull())
        {
            superq_aux[5]=center->get(0).asDouble(); superq_aux[6]=center->get(1).asDouble(); superq_aux[7]=center->get(2).asDouble();
        }

        Bottle *orientation=estimated_superq.find("orientation").asList();

        if (!estimated_superq.find("orientation").isNull())
        {
            Vector axis(4,0.0);
            axis[0]=orientation->get(0).asDouble(); axis[1]=orientation->get(1).asDouble(); axis[2]=orientation->get(2).asDouble(); axis[3]=orientation->get(3).asDouble();
            superq_aux.setSubvector(8,axis);
        }*/

        Bottle &b1=cmd.addList();
        Bottle &b2=b1.addList();
        b2.addString("dimensions");
        Bottle &b2l=b2.addList();
        b2l.addDouble(superq_aux[0]); b2l.addDouble(superq_aux[1]); b2l.addDouble(superq_aux[2]);

        Bottle &b3=b1.addList();
        b3.addString("exponents");
        Bottle &b3l=b3.addList();
        b3l.addDouble(superq_aux[3]); b3l.addDouble(superq_aux[4]);

        Bottle &b4=b1.addList();
        b4.addString("center");
        Bottle &b4l=b4.addList();
        b4l.addDouble(superq_aux[5]); b4l.addDouble(superq_aux[6]); b4l.addDouble(superq_aux[7]);

        Bottle &b5=b1.addList();
        b5.addString("orientation");
        Bottle &b5l=b5.addList();
        b5l.addDouble(superq_aux[8]); b5l.addDouble(superq_aux[9]); b5l.addDouble(superq_aux[10]); b5l.addDouble(superq_aux[11]);

        return superq_aux;
    }

    /**
    * Check if the reconstructed superquadric is safe for grasping.
    */
    /***********************************************************************/
    bool superqOk(Vector &sup)
    {
        bool ok=false;
        yDebug()<<"Checking superq..";
        cout<<endl;
        yDebug()<<"superq "<<sup.toString();
        if ((sup[0] <= 0.3) && (sup[1] <= 0.3) && (sup[2]<=0.3))
            ok=true;
        if ((sup[5] <= -0.15) && (sup[5] >= -0.4) && (sup[6] <= -0.25) && (sup[6] >= 0.25) && (sup[7] >= -plane[3]) && (sup[7] <= 0.35) )
            ok = ok && true;

        cout<<"ok "<<ok<<endl;
        return ok;
    }

    /**
    * Get plane information from superquadric-grasp module.
    */
    /***********************************************************************/
    bool getPlane()
    {
        Bottle cmd, reply;
        cmd.clear();
        cmd.addString("get_options");
        cmd.addString("pose");
        plane.resize(4,0.0);


        if (graspRpc.write(cmd, reply))
        {
            Bottle *all=reply.get(0).asList();

            for (size_t i=0; i<all->size(); i++)
            {

                Bottle *group=all->get(i).asList();
                if (group->get(0).asString() == "plane")
                {
                    Bottle *dim=group->get(1).asList();

                    plane[0]=dim->get(0).asDouble(); plane[1]=dim->get(1).asDouble();
                    plane[2]=dim->get(2).asDouble(); plane[3]=dim->get(3).asDouble();
                    yDebug()<<"plane"<<plane.toString(3,3);
                }
            }
        }

        if (norm(plane)!=0.0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
    * Check if the reconstructed superquadric is safe for grasping.
    */
    /***********************************************************************/
    bool poseOk(Bottle &reply)
    {
        bool ok=false;
        double z_left, z_right;
        Vector left(3,0.0);
        Vector right(3,0.0);

        Bottle *all=reply.get(0).asList();

        for (size_t i=0; i<all->size(); i++)
        {
            Bottle *group=all->get(i).asList();
            if (group->get(0).asString() == "pose_left")
            {
                Bottle *dim=group->get(1).asList();

                z_left=dim->get(2).asDouble();

                left[0]=dim->get(0).asDouble(); left[1]=dim->get(1).asDouble(); left[2]=dim->get(2).asDouble();
                yDebug()<<"z_left"<<z_left;
                yDebug()<<"left"<<left.toString();
            }

            if (group->get(0).asString() == "pose_right")
            {
                Bottle *dim=group->get(1).asList();

                z_right=dim->get(2).asDouble();
                right[0]=dim->get(0).asDouble(); right[1]=dim->get(1).asDouble(); right[2]=dim->get(2).asDouble();
                yDebug()<<"z_right"<<z_right;
                yDebug()<<"right"<<right.toString();
            }
        }

        cout<<endl<<-plane[3] + 0.04<<endl<<endl;

        if ((norm(left)>0.0) && (norm(right)>0.0))
        {
            if ( z_left >= -plane[3] + 0.04)
                ok=true;
            if ( z_right >= -plane[3] + 0.04)
                ok=ok && true;

            if ((left[0]>-0.4) && (left[0]<-0.2))
                ok=ok && true;

            if ((right[0]>-0.4) && (right[0]<-0.2))
                ok=ok && true;

        }
        else if (norm(left)>0.0) 
        {
            if ( z_left >= -plane[3] + 0.04)
             ok=true;

            if ((left[0]>-0.4) && (left[0]<-0.2))
                ok=ok && true;
        }
        else if (norm(right)>0.0) 
        {
            if ( z_right>= -plane[3] + 0.04)
             ok=true;

            if ((right[0]>-0.4) && (right[0]<-0.2))
                ok=ok && true;
        }
        else
            ok=false;

        yDebug()<<"ok "<<ok;

        return ok;
    }
};

/**********************************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    GraspDemo mod;
    ResourceFinder rf;
    rf.setDefaultContext("grasp-demo");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
