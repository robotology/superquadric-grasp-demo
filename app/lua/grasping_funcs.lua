----------------------------------
-- functions GRASPING        --
----------------------------------

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

function GRASPING_set_object(port, name)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("set_object_name")
    wb::addString(name)
    port:write(wb,reply)
    return reply:get(0):asString()
end
