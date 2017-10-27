----------------------------------
-- functions GRASPING        --
----------------------------------

classes = {"box", "sphere", "cylinder"}


function GRASPING_look_for_object(port, port2)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    for i=1, classes:size() do
      wb:clear()
      wb:addVocab("ask")
      local content = wb:addList()
      local content1 = content:addList()
      content1.addString("entity")
      content1.addString("==")
      content1.addString("object")
      content.addString("&&")
      local content2 = content:addList()
      content2.addString("name")
      content2.addString("==")
      content2.addString(classes[i])
      port:write(wb,reply)
      local ok = ok && (reply:get(0):asVocab() == "ack")
    end
     if ok==true then
        local wb2 = yarp.Bottle()
        local reply2 = yarp.Bottle()
        wb2:clear()
        wb2:addString("set_object_class")
        port2:write(wb,reply)
        ok = ok &&  reply:get(0):asString()
     end

    return ok
end

function GRASPING_get_superq(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("acquire_superq")
    port:write(wb,reply)
    return reply:get(0):asString()
end

function GRASPING_compute_pose(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("compute_pose")
    port:write(wb,reply)
    return reply:get(0):asString()
    end
end

function GRASPING_grasp(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("grasp_object")
    port:write(wb,reply)
    return reply:get(0):asString()
end

function GRASPING_go_home(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("go_back_home")
    port:write(wb,reply)
    return reply:get(0):asString()
end

function GRASPING_set_object(port, name)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("set_object_name")
    wb::addString(name)
    port:write(wb,reply)
    return reply:get(0):asString()
end
