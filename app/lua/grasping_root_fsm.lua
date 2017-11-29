#!/usr/bin/lua

-- initialize yarp
if yarp == nil then
    require("yarp")
    yarp.Network()
end

-- find all required files
if rf ~= nil then
    funcs = rf:findFile("grasping_funcs.lua")
else
    funcs = "grasping_funcs.lua"
end

dofile(funcs)

-- initialization

grasp_demo_port = yarp.Port()
memory_port = yarp.Port()
grasp_motion_port = yarp.Port()
grasp_attention_port = yarp.Port()

return rfsm.state {

   ----------------------------------
   -- state INITPORTS                  --
   ----------------------------------
   ST_INITPORTS = rfsm.state{
           doo=function()
                   ret = grasp_demo_port:open("/grasping-lua/demo")
                   ret = grasp_motion_port:open("/grasping-lua/motion")
                   ret = ret and memory_port:open("/grasping-lua/memory")
                   ret = ret and grasp_attention_port:open("/grasping-lua/attention")

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
           doo=function()
                   ret = yarp.NetworkBase_connect(grasp_demo_port:getName(), "/grasp-demo/rpc")
                   ret = yarp.NetworkBase_connect(grasp_motion_port:getName(), "/superquadric-grasp/rpc")
                   ret = ret and yarp.NetworkBase_connect(memory_port:getName(), "/memory/rpc")
                   ret = ret and yarp.NetworkBase_connect(grasp_attention_port:getName(), "/iolStateMachineHandler/human:rpc")
                   if ret == false then
                           ----print("\n\nERROR WITH CONNECTIONS, PLEASE CHECK\n\n")
                           rfsm.send_events(fsm, 'e_error')
                   end
           end
},

----------------------------------
  -- state PREPARATION        --
  ----------------------------------
  ST_PREPARATION  = rfsm.state{
          doo=function()
                  ret = GRASPING_preparation( grasp_attention_port)
                  if ret == false then
                      rfsm.send_events(fsm, 'e_error')
                  elseif ret == true then
                      rfsm.send_events(fsm, 'e_done')
                  end
          end
},


----------------------------------
  -- state FATAL                  --
  ----------------------------------
  ST_FATAL = rfsm.state{
          doo=function()
                  ----print("Fatal!")
                  shouldExit = true;
          end
},



----------------------------------
   -- state FINI                   --
   ----------------------------------
   ST_FINI = rfsm.state{
           doo=function()
                   ----print("Closing...")
                   yarp.NetworkBase_disconnect(grasp_demo_port:getName(), "/grasp-demo/rpc")

                   grasp_demo_port:close()

                   shouldExit = true;
           end
},

----------------------------------
  -- state LOOK_FOR_OBJECT        --
  ----------------------------------
  ST_LOOK_FOR_OBJECT  = rfsm.state{
          doo=function()
                  ----print(" looking for object ..")
                  while true do
                     ret = GRASPING_look_for_object(memory_port, grasp_demo_port)
                     if ret == "ok" then
                         rfsm.send_events(fsm, 'e_ok')
                     end
                     rfsm.yield(true)
                  end

          end
},


----------------------------------
  -- state ACQUIRE_SUPERQ                --
  ----------------------------------
  ST_ACQUIRE_SUPERQ = rfsm.state{
          doo=function()
                  ----print(" acquiring superquadric ..")
                  ret = GRASPING_look_center(grasp_motion_port)
                  ret = ret and GRASPING_start_from_scratch(grasp_demo_port)
                  ret = ret and GRASPING_get_superq(grasp_demo_port)
                  if ret == "fail" then
                      ----print("\n\nERROR WITH ACQUIRING SUPERQUADRIC, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  end

          end
},

----------------------------------
  -- state CHECK_SUPERQ                --
  ----------------------------------
  ST_CHECK_SUPERQ = rfsm.state{
          doo=function()
                  ----print(" checking superquadric ..")

                  while true do
                     ret = GRASPING_check_superq(grasp_demo_port)
                     --if ret == "wait" then
                      ------print("\n\nERROR IN CHECKING SUPERQUADRIC, PLEASE CHECK\n\n")
                      --  rfsm.send_events(fsm, 'e_error')
                     if ret == "ok" then
                      ----print("\n\nSUPERQ COMPUTED!!!\n\n")
                        rfsm.send_events(fsm, 'e_ok')
                     elseif ret == "again" then
                      ----print("\n\nSUPERQ NOT COMPUTED!!!\n\n")
                        rfsm.send_events(fsm, 'e_again')
                     end
                     rfsm.yield(true)
                  end




          end
},

----------------------------------
  -- state COMPUTE_POSE                --
  ----------------------------------
  ST_COMPUTE_POSE = rfsm.state{
          doo=function()
                  ----print(" computing poses ..")
                  while true do
                      ret =  GRASPING_compute_pose(grasp_demo_port)
                      if ret == "ok" then
                          rfsm.send_events(fsm, 'e_ok')
                      elseif ret == "again" then
                      ----print("\n\nSUPERQ NOT COMPUTED!!!\n\n")
                        rfsm.send_events(fsm, 'e_again')
                      end
                      rfsm.yield(true)
                  end
          end
},

----------------------------------
  -- state CHECK_COMPUTE_POSE        --
  ----------------------------------
  ST_CHECK_POSE = rfsm.state{
          doo=function()
                  ----print(" checking pose ..")
                  while true do
                     ret = GRASPING_check_pose(grasp_demo_port)
                     --if ret == "wait" then
                      ------print("\n\nERROR IN CHECKING SUPERQUADRIC, PLEASE CHECK\n\n")
                      --  rfsm.send_events(fsm, 'e_error')
                     if ret == "ok" then
                      ----print("\n\nSUPERQ COMPUTED!!!\n\n")
                        rfsm.send_events(fsm, 'e_ok')
                     elseif ret == "again" then
                      ----print("\n\nSUPERQ NOT COMPUTED!!!\n\n")
                        rfsm.send_events(fsm, 'e_again')
                     end
                     rfsm.yield(true)
                  end


          end
},

----------------------------------
  -- state GRASP_OBJECT                --
  ----------------------------------
  ST_GRASP_OBJECT = rfsm.state{
          doo=function()
                  ----print(" grasping object ..")
                  ret = GRASPING_grasp_object(grasp_demo_port)
                  if ret == "fail" then
                      ----print("\n\nERROR WITH GRASPING OBJECT, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  elseif ret == "ok" then
                      rfsm.send_events(fsm, 'e_ok')
                  end
          end

},

----------------------------------
  -- state CHECK_MOVEMENT        --
  ----------------------------------
  ST_CHECK_MOVEMENT = rfsm.state{
          doo=function()
                  ----print(" checking movement ..")
                  while true do
                      ret =  GRASPING_check_motion(grasp_motion_port)
                      if ret == "ok" then
                          rfsm.send_events(fsm, 'e_ok')
                      end
                      rfsm.yield(true)
                  end

          end
},

----------------------------------
  -- state GO_HOME                --
  ----------------------------------
  ST_GO_HOME = rfsm.state{
          doo=function()
                  ----print(" going home ..")
                  ret = GRASPING_go_home(grasp_demo_port)
                  ret = ret and GRASPING_look_center(grasp_motion_port)
                  if ret == "fail" then
                      ------print("\n\nERROR WITH GOING HOME, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  end
          end

},

----------------------------------
  -- state GO_TO_BASKET                --
  ----------------------------------
  ST_GO_TO_BASKET = rfsm.state{
          doo=function()
                  ----print(" going home ..")
                  ret = GRASPING_go_to_basket(grasp_demo_port)
                  if ret == "fail" then
                      ------print("\n\nERROR WITH GOING HOME, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  end
          end

},

----------------------------------
  -- state CLEAR_POSES            --
  ----------------------------------
  ST_CLEAR_POSES = rfsm.state{
          doo=function()
                  ----print(" clear poses ..")
                  ret = GRASPING_clear_poses(grasp_demo_port)
                  if ret == "fail" then
                      ------print("\n\nERROR WITH CLEARING POSE, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                 elseif ret == "ok" then
                      rfsm.send_events(fsm, 'e_ok')
                  end
          end

},

----------------------------------
  -- state CLEAR_SUPERQ            --
  ----------------------------------
  ST_START_FROM_SCRATCH = rfsm.state{
          doo=function()
                  ----print(" starting from scratch ..")
                  ret = GRASPING_start_from_scratch(grasp_demo_port)
                  if ret == "fail" then
                      ----print("\n\nERROR WITH STARTING FROM SCRATCH, PLEASE CHECK\n\n")
                      rfsm.send_events(fsm, 'e_error')
                  elseif ret == "ok" then
                      rfsm.send_events(fsm, 'e_ok')
                  end
          end

},

----------------------------------
  -- state CHECK_HOME        --
  ----------------------------------
  ST_CHECK_HOME = rfsm.state{
          doo=function()
                  ----print(" checking home ..")
                  while true do
                      ret =  GRASPING_check_home(grasp_motion_port)
                      if ret == "ok" then
                          rfsm.send_events(fsm, 'e_ok')
                      end
                      rfsm.yield(true)
                  end

          end
},


ST_INTERACT = interact_fsm,


 rfsm.transition { src='initial', tgt='ST_INITPORTS' },
 rfsm.transition { src='ST_INITPORTS', tgt='ST_CONNECTPORTS', events={ 'e_connect' } },
 rfsm.transition { src='ST_INITPORTS', tgt='ST_FATAL', events={ 'e_error' } },
 rfsm.transition { src='ST_CONNECTPORTS', tgt='ST_FINI', events={ 'e_error' } },
 rfsm.transition { src='ST_CONNECTPORTS', tgt='ST_PREPARATION', events={ 'e_done' } },

 rfsm.transition { src='ST_PREPARATION', tgt='ST_LOOK_FOR_OBJECT', events={ 'e_done' } },
 rfsm.transition { src='ST_LOOK_FOR_OBJECT', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_ok' } },

 rfsm.transition { src='ST_ACQUIRE_SUPERQ', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_error' } },
 rfsm.transition { src='ST_ACQUIRE_SUPERQ', tgt='ST_CHECK_SUPERQ', events={ 'e_done' } },

rfsm.transition { src='ST_CHECK_SUPERQ', tgt='ST_LOOK_FOR_OBJECT', events={ 'e_again' } },
rfsm.transition { src='ST_CHECK_SUPERQ', tgt='ST_COMPUTE_POSE', events={ 'e_ok' } },

rfsm.transition { src='ST_COMPUTE_POSE', tgt='ST_CHECK_POSE', events={ 'e_ok' } },

rfsm.transition { src='ST_CHECK_POSE', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_again' } },
rfsm.transition { src='ST_CHECK_POSE', tgt='ST_GRASP_OBJECT', events={ 'e_ok' } },

--rfsm.transition { src='ST_GRASP_OBJECT', tgt='ST_ACQUIRE_SUPERQ', events={ 'e_error' } },
rfsm.transition { src='ST_GRASP_OBJECT', tgt='ST_CHECK_MOVEMENT', events={ 'e_ok' } },

--rfsm.transition { src='ST_CHECK_MOVEMENT', tgt='ST_GO_HOME', events={ 'e_ok' } },
rfsm.transition { src='ST_CHECK_MOVEMENT', tgt='ST_GO_TO_BASKET', events={ 'e_ok' } },
rfsm.transition { src='ST_GO_TO_BASKET', tgt='ST_CLEAR_POSES', events={ 'e_done' } },

rfsm.transition { src='ST_CLEAR_POSES', tgt='ST_GO_HOME', events={ 'e_done' } },
rfsm.transition { src='ST_GO_HOME', tgt='ST_START_FROM_SCRATCH', events={'e_done'} },
rfsm.transition { src='ST_START_FROM_SCRATCH', tgt='ST_CHECK_HOME', events={ 'e_ok' } },


rfsm.transition { src='ST_CHECK_HOME', tgt='ST_LOOK_FOR_OBJECT', events={ 'e_ok' } },

































 }
