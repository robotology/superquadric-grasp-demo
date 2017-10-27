#!/usr/bin/lua

dofile(rf:findFile("grasping_funcs.lua"))

return rfsm.state {

   ----------------------------------
   -- state INITPORTS                  --
   ----------------------------------
   ST_INITPORTS = rfsm.state{
           entry=function()
                   ret = grasp_demo_port:open("/grasping-lua/demo")
                   ret = ret and memory_port:open("/grasping-lua/memory")

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
                   ret = ret and yarp.NetworkBase_connect(memory_port:getName(), "/memory/rpc")
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

                   grasp_demo_port:close()

                   shouldExit = true;
           end
},

----------------------------------
  -- state LOOK_FOR_OBJECT        --
  ----------------------------------
  ST_LOOK_FOR_OBJECT  = rfsm.state{
          entry=function()
                  print(" looking for object ..")
                  ret = GRASPING_look_for_object(memory_port, grasp_demo_port)
                  if ret == "false" then
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
  -- state CHECK_SUPERQ                --
  ----------------------------------
  ST_CHECK_SUPERQ = rfsm.state{
          entry=function()
                  print(" checking superquadric ..")
                  ret = GRASPING_check_superq(grasp_demo_port)
                  if ret == "fail" then
                      print("\n\nERROR IN CHECKING SUPERQUADRIC, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  elseif ret == "ok" then
                      rfsm.send_events(fsm, 'e_done')
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
                  elseif ret == "ok" then
                      rfsm.send_events(fsm, 'e_done')
                  end
          end
},

----------------------------------
  -- state CHECK_COMPUTE_POSE        --
  ----------------------------------
  ST_CHECK_POSE = rfsm.state{
          entry=function()
                  print(" checking pose ..")
                  ret = GRASPING_check_pose(grasp_demo_port)
                  if ret == "fail" then
                      print("\n\nERROR IN CHECKING POSE, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  elseif ret == "ok" then
                      rfsm.send_events(fsm, 'e_done')
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

----------------------------------
  -- state GO_HOME                --
  ----------------------------------
  ST_GO_HOME = rfsm.state{
          entry=function()
                  print(" grasping object ..")
                  ret = GRASPING_go_home(grasp_demo_port)
                  if ret == "fail" then
                      print("\n\nERROR WITH GOING HOME, PLEASE CHECK\n\n")
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

 rfsm.transition { src='ST_ACQUIRE_SUPERQ', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_error' } },
 rfsm.transition { src='ST_ACQUIRE_SUPERQ', tgt='ST_CHECK_SUPERQ', events={ 'e_done' } },

rfsm.transition { src='ST_CHECK_SUPERQ', tgt='ST_CHECK_SUPERQ', events={ 'e_error' } },
rfsm.transition { src='ST_CHECK_SUPERQ', tgt='ST_COMPUTE_POSE', events={ 'e_done' } },

rfsm.transition { src='ST_COMPUTE_POSE', tgt='ST_COMPUTE_POSE', events={ 'e_error' } },
rfsm.transition { src='ST_COMPUTE_POSE', tgt='ST_CHECK_POSE', events={ 'e_done' } },

rfsm.transition { src='ST_CHECK_POSE', tgt='ST_CHECK_POSE', events={ 'e_error' } },
--rfsm.transition { src='ST_CHECK_POSE', tgt='ST_GRASP_OBJECT', events={ 'e_done' } },

--rfsm.transition { src='ST_GRASP_OBJECT', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_error' } },
--rfsm.transition { src='ST_GRASP_OBJECT', tgt='ST_CHECK_MOVEMENT', events={ 'e_done' } },

--rfsm.transition { src='ST_CHECK_MOVEMENT', tgt='ST_CHECK_MOVEMENT', events={ 'e_error' } },
--rfsm.transition { src='ST_CHECK_MOVEMENT', tgt='ST_GO_HOME', events={ 'e_done' } },

 --rfsm.transition { src='ST_GO_HOME', tgt='ST_LOOK_FOR_OBJECT', events={ 'e_done' } },
rfsm.transition { src='ST_CHECK_POSE', tgt='ST_LOOK_FOR_OBJECT', events={ 'e_done' } },
































 }
