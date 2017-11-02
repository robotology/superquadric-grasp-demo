----------------------------------
-- functions GRASPING        --
----------------------------------

classes = {"box", "sphere", "cylinder"}


function GRASPING_look_for_object(port, port2)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    local voc = yarp.Vocab()
    local ask = "ask"
    local voc2 = yarp.Vocab.encode(ask)
    local k=1000
    for i in pairs(classes) do
      wb:clear()
      wb:addVocab(voc2)
      local content = wb:addList()
      local content1 = content:addList()
      content1:addString("entity")
      content1:addString("==")
      content1:addString("object")
      content:addString("&&")
      local content2 = content:addList()
      content2:addString("name")
      content2:addString("==")
      content2:addString(classes[i])
      port:write(wb,reply)
      if reply:get(0):asVocab() == yarp.Vocab.encode("ack") then
        local b=yarp.Bottle()
        b=reply:get(1):asList()
        if  b then

             local b1=yarp.Bottle()
             b1=b:get(1):asList()
             if b1 then

                wb:clear()
                local id=b1:get(0):asInt()
                wb:addVocab(yarp.Vocab.encode("get"))
                local info = yarp.Bottle()
                info=wb:addList()
                local info2 = yarp.Bottle()
                info2=info:addList()
                info2:addString("id");
                info2:addInt(id);
                local info3 = yarp.Bottle()
                info3=info2:addList()
                info3:addString("propSet");
                local info4 = yarp.Bottle()
                info4=info3:addList()
                info4:addString("position_2d_left");
                port:write(wb, reply)

                if reply:get(0):asVocab() == yarp.Vocab.encode("ack") then
                    local pos=yarp.Bottle()               
                    pos = reply:get(1):asList()
                    
                    local pos2d = pos:get(2):asList()               
                     if pos2d then
                        k=i 
                        print("found")
                        print(classes[k])
                    --else
                    --   k=1000
                    end
                end

             end      
        end

      end

      if k < 1000 then       
        ok=true
      elseif k==1000 then
        ok=false
      end
    end
     if ok then
        local wb2 = yarp.Bottle()
        local reply2 = yarp.Bottle()
        wb2:clear()
        wb2:addString("set_object_class")
        wb2:addString(classes[k])
        port2:write(wb2,reply)
        ok = ok and  reply:get(0):asString()
     end

    return ok
end

function GRASPING_preparation(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("attention")
    wb:addString("stop")
    port:write(wb,reply)
    return reply:get(0):asString()
end

function GRASPING_get_superq(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("acquire_superq")
    port:write(wb,reply)
    yarp.Time_delay(1.0)
    return reply:get(0):asString()
end

function GRASPING_check_superq(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("check_superq")
    port:write(wb,reply)
    print(reply:get(0):asString())
    return reply:get(0):asString()
end

function GRASPING_compute_pose(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("compute_pose")
    port:write(wb,reply)
    yarp.Time_delay(1.0)
    return reply:get(0):asString()
end

function GRASPING_check_pose(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("check_pose")
    port:write(wb,reply)
    return reply:get(0):asString()
end

function GRASPING_grasp_object(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("grasp_object")
    yarp.Time_delay(1.5)
    port:write(wb,reply)
    yarp.Time_delay(1.5)    
    return reply:get(0):asString()
end

function GRASPING_go_home(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("go_back_home")
    port:write(wb,reply)
    yarp.Time_delay(2.0)
    return reply:get(0):asString()
end

function GRASPING_check_motion(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("check_motion")
    port:write(wb,reply)
    print(reply:get(0):asString())
    return reply:get(0):asString()
end

function GRASPING_check_home(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("check_home")
    port:write(wb,reply)
    return reply:get(0):asString()
end

function GRASPING_clear_poses(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("clear_poses")
    port:write(wb,reply)
    print(reply:get(0):asString())
    return reply:get(0):asString()
end

function GRASPING_start_from_scratch(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("start_from_scratch")    
    port:write(wb,reply)
    print(reply:get(0):asString())
    return reply:get(0):asString()
end


--function GRASPING_set_object(port, name)
--    local wb = yarp.Bottle()
--   local reply = yarp.Bottle()
--    wb:clear()
--    wb:addString("set_object_name")
--    wb::addString(name)
--    port:write(wb,reply)
--    return reply:get(0):asString()
--end
