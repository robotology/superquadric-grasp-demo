#!/usr/bin/lua

dofile(rf:findFile("grasping_funcs.lua"))

return rfsm.state {

   ----------------------------------
   -- state INITPORTS                  --
   ----------------------------------
   ST_INITPORTS = rfsm.state{
           entry=function()
                   ret = graps_demo_port:open("/grasping-lua/demo")

                   if ret == false then
                           rfsm.send_events(fsm, 'e_error')
                   else
                           rfsm.send_events(fsm, 'e_connect')
                   end
           end
},

----------------------------------
   -- state CONNECTPORTS           --
   ----------------------------------
   ST_CONNECTPORTS = rfsm.state{
           entry=function()
                   ret = yarp.NetworkBase_connect(grasp_demo_port:getName(), "/grasp-demo/rpc")
                   ret = ret and yarp.NetworkBase_connect(grasp_demo_port:getName(), "/memory/rpc")
                   if ret == false then
                           print("\n\nERROR WITH CONNECTIONS, PLEASE CHECK\n\n")
                           rfsm.send_events(fsm, 'e_error')
                   end
           end
},


----------------------------------
  -- state FATAL                  --
  ----------------------------------
  ST_FATAL = rfsm.state{
          entry=function()
                  print("Fatal!")
                  shouldExit = true;
          end
},



----------------------------------
   -- state FINI                   --
   ----------------------------------
   ST_FINI = rfsm.state{
           entry=function()
                   print("Closing...")
                   yarp.NetworkBase_disconnect(grasp_demo_port:getName(), "/grasp-demo/rpc")

                   graps_demo_port:close()

                   shouldExit = true;
           end
},

----------------------------------
  -- state LOOK_FOR_OBJECT        --
  ----------------------------------
  ST_LOOK_FOR_OBJECT  = rfsm.state{
          entry=function()
                  print(" looking for object ..")
                  ret = GRASPING_look_for_object("/memory/rpc", grasp_demo_port)
                  if ret == "fail" then
                      print("\n\nNO OBJECT FOUND...\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  end

          end
},


----------------------------------
  -- state ACQUIRE_SUPERQ                --
  ----------------------------------
  ST_ACQUIRE_SUPERQ = rfsm.state{
          entry=function()
                  print(" acquiring superquadric ..")
                  ret = GRASPING_get_superq(grasp_demo_port)
                  if ret == "fail" then
                      print("\n\nERROR WITH ACQUIRING SUPERQUADRIC, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  end

          end
},

----------------------------------
  -- state COMPUTE_POSE                --
  ----------------------------------
  ST_COMPUTE_POSE = rfsm.state{
          entry=function()
                  print(" computing poses ..")
                  ret =  GRASPING_compute_pose(grasp_demo_port)
                  if ret == "fail" then
                      print("\n\nERROR WITH COMPUTING POSES, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state GRASP_OBJECT                --
  ----------------------------------
  ST_GRASP_OBJECT = rfsm.state{
          entry=function()
                  print(" grasping object ..")
                  ret = GRASPING_grasp_object(grasp_demo_port)
                  if ret == "fail" then
                      print("\n\nERROR WITH GRASPING OBJECT, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  end
          end

},


ST_INTERACT = interact_fsm,


 rfsm.transition { src='initial', tgt='ST_INITPORTS' },
 rfsm.transition { src='ST_INITPORTS', tgt='ST_CONNECTPORTS', events={ 'e_connect' } },
 rfsm.transition { src='ST_INITPORTS', tgt='ST_FATAL', events={ 'e_error' } },
 rfsm.transition { src='ST_CONNECTPORTS', tgt='ST_FINI', events={ 'e_error' } },

 rfsm.transition { src='ST_CONNECTPORTS', tgt='ST_LOOK_FOR_OBJECT', events={ 'e_done' } },
 rfsm.transition { src='ST_LOOK_FOR_OBJECT', tgt='ST_LOOK_FOR_OBJECT', events={ 'e_error' } },

 rfsm.transition { src='ST_LOOK_FOR_OBJECT', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_done' } },
 rfsm.transition { src='ST_LOOK_FOR_OBJECT', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_done' } },

 rfsm.transition { src='ST_ACQUIRE_SUPERQ', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_error' } },
 rfsm.transition { src='ST_ACQUIRE_SUPERQ', tgt='ST_COMPUTE_POSE', events={ 'e_done' } },

 rfsm.transition { src='ST_COMPUTE_POSE', tgt='ST_COMPUTE_POSE', events={ 'e_error' } },
 rfsm.transition { src='ST_COMPUTE_POSE', tgt='ST_GRASP_OBJECT', events={ 'e_done' } },

 rfsm.transition { src='ST_GRASP_OBJECT', tgt='ST_COMPUTE_POSE', events={ 'e_error' } },
 rfsm.transition { src='ST_GRASP_OBJECT', tgt='ST_LOOK_FOR_OBJECT', events={ 'e_done' } },
































 }
