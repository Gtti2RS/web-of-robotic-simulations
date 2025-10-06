-- RG2 Hand Pose Publisher - Console Executable Version
-- Usage: Run this script from CoppeliaSim console to start publishing RG2 hand pose
-- Usage: startUR10RG2Helper(ur10_handle) - where ur10_handle is the handle of the UR10 base object
simROS2=require'simROS2'

-- Global variables
ur10_base_handle = nil
rg2_hand_handle = nil
ur10_joint_handles = {}
rg2_pose_publisher = nil
trajectory_subscriber = nil
rg2_publish_frequency = 10.0 -- Hz
rg2_last_publish_time = 0
rg2_publisher_active = false
trajectory_subscriber_active = false

-- Main function to start the UR10 RG2 helper
function startUR10RG2Helper(ur10_handle)
    if not simROS2 then
        sim.addLog(sim.verbosity_scripterrors, "ROS2 interface was not found. Cannot run UR10 RG2 helper.")
        return false
    end
    
    if rg2_publisher_active then
        sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper is already running. Stop it first with stopUR10RG2Helper()")
        return false
    end
    
    -- Validate UR10 handle parameter
    if not ur10_handle then
        sim.addLog(sim.verbosity_scripterrors, "UR10 handle parameter is required. Usage: startUR10RG2Helper(ur10_handle)")
        return false
    end
    
    -- Verify the handle is valid
    local objectType = sim.getObjectType(ur10_handle)
    if objectType == -1 then
        sim.addLog(sim.verbosity_scripterrors, "Invalid UR10 handle: " .. tostring(ur10_handle))
        return false
    end
    
    ur10_base_handle = ur10_handle
    
    -- Find RG2 hand by name (this still needs to be found by name)
    rg2_hand_handle = findUR10ObjectByName("rg2_hand_rg2_tipp")
    
    if rg2_hand_handle == -1 then
        sim.addLog(sim.verbosity_scripterrors, "Could not find RG2 hand object 'rg2_hand_rg2_tipp'.")
        return false
    end
    
    -- Get joint handles for the UR10 robot
    ur10_joint_handles = getUR10JointHandles(ur10_base_handle)
    if #ur10_joint_handles == 0 then
        sim.addLog(sim.verbosity_scripterrors, "Could not find UR10 joint handles.")
        return false
    end
    
    -- Set all UR10 joints to dynamic mode
    for i = 1, #ur10_joint_handles do
        local jointHandle = ur10_joint_handles[i]
        local alias = sim.getObjectAlias(jointHandle, 1)
        
        local setModeSuccess, errorMsg = pcall(sim.setJointMode, jointHandle, sim.jointmode_dynamic)
        if not setModeSuccess then
            sim.addLog(sim.verbosity_scripterrors, "Failed to set joint " .. alias .. " to dynamic mode: " .. tostring(errorMsg))
        end
    end
    
    -- Create ROS2 publisher for pose (using String message with JSON format)
    rg2_pose_publisher = simROS2.createPublisher('/coppeliasim/rg2_tipp/pose', 'std_msgs/msg/String')
    
    -- Create ROS2 subscriber for trajectory commands
    trajectory_subscriber = simROS2.createSubscription('/ur10_rg2/trajectory', 'std_msgs/msg/String', 'trajectory_callback')
    
    rg2_publisher_active = true
    trajectory_subscriber_active = true
    rg2_last_publish_time = sim.getSimulationTime()
    
    sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper started successfully")
    sim.addLog(sim.verbosity_msgs, "UR10 base handle: " .. tostring(ur10_base_handle))
    sim.addLog(sim.verbosity_msgs, "RG2 hand handle: " .. tostring(rg2_hand_handle))
    sim.addLog(sim.verbosity_msgs, "Found " .. #ur10_joint_handles .. " joint handles")
    sim.addLog(sim.verbosity_msgs, "Pose topic: /coppeliasim/rg2_tipp/pose")
    sim.addLog(sim.verbosity_msgs, "Trajectory topic: /ur10_rg2/trajectory")
    sim.addLog(sim.verbosity_msgs, "Frequency: " .. rg2_publish_frequency .. " Hz")
    
    return true
end

-- Function to stop the UR10 RG2 helper
function stopUR10RG2Helper()
    if not rg2_publisher_active then
        sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper is not running.")
        return false
    end
    
    if simROS2 then
        if rg2_pose_publisher then
            simROS2.shutdownPublisher(rg2_pose_publisher)
        end
        if trajectory_subscriber then
            simROS2.shutdownSubscription(trajectory_subscriber)
        end
    end
    
    rg2_publisher_active = false
    trajectory_subscriber_active = false
    rg2_pose_publisher = nil
    trajectory_subscriber = nil
    ur10_base_handle = nil
    rg2_hand_handle = nil
    ur10_joint_handles = {}
    
    sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper stopped successfully")
    return true
end

function findUR10ObjectByName(objectName)
    -- Try direct path search first
    local directHandle = sim.getObject('/' .. objectName)
    if directHandle ~= -1 then
        sim.addLog(sim.verbosity_msgs, "Found object '" .. objectName .. "' via direct path, handle: " .. directHandle)
        return directHandle
    end
    
    -- Search for object by alias
    local objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    sim.addLog(sim.verbosity_msgs, "Searching through " .. (objects and #objects or 0) .. " objects for '" .. objectName .. "'")
    
    if objects then
        for i = 1, #objects do
            local handle = objects[i]
            local alias = sim.getObjectAlias(handle, 1)
            
            if alias == objectName then
                sim.addLog(sim.verbosity_msgs, "Found object '" .. objectName .. "' via alias search, handle: " .. handle)
                return handle
            end
        end
    end
    
    sim.addLog(sim.verbosity_scripterrors, "Object '" .. objectName .. "' not found in scene")
    return -1
end

function getUR10JointHandles(baseHandle)
    local jointHandles = {}
    
    -- Get all joint objects in the UR10 robot tree
    local joints = sim.getObjectsInTree(baseHandle, sim.object_joint_type, 0)
    
    if joints then
        for i = 1, #joints do
            local handle = joints[i]
            local alias = sim.getObjectAlias(handle, 1)
            
            -- Include all joints (UR10 + RG2 gripper joints)
            if alias then
                jointHandles[#jointHandles+1] = handle
            end
        end
    end
    
    -- Sort joints to ensure correct order: UR10 joints first, then RG2 joints
    table.sort(jointHandles, function(a, b)
        local aliasA = sim.getObjectAlias(a, 1)
        local aliasB = sim.getObjectAlias(b, 1)
        
        -- Define joint order priority
        local function getJointPriority(alias)
            if string.find(alias, "shoulder_pan") then return 1
            elseif string.find(alias, "shoulder_lift") then return 2
            elseif string.find(alias, "elbow") then return 3
            elseif string.find(alias, "wrist_1") then return 4
            elseif string.find(alias, "wrist_2") then return 5
            elseif string.find(alias, "wrist_3") then return 6
            elseif string.find(alias, "rg2_finger_joint1") or string.find(alias, "finger_joint") then return 7
            elseif string.find(alias, "rg2_finger_joint2") or string.find(alias, "finger_joint_2") then return 8
            else return 999 -- Unknown joints go last
            end
        end
        
        local priorityA = getJointPriority(aliasA)
        local priorityB = getJointPriority(aliasB)
        
        return priorityA < priorityB
    end)
    
    return jointHandles
end

-- Trajectory callback function
function trajectory_callback(msg)
    if not trajectory_subscriber_active then
        return
    end
    
    local trajectoryData = msg.data
    
    -- Parse JSON string
    local success, result = pcall(function()
        -- Simple JSON parsing for our specific format
        local handleMatch = string.match(trajectoryData, '"handle"%s*:%s*([^,}]+)')
        local positionsMatch = string.match(trajectoryData, '"positions"%s*:%s*%[([^%]]+)%]')
        
        if not handleMatch or not positionsMatch then
            error("Invalid JSON format")
        end
        
        local handle = tonumber(handleMatch:gsub("^%s*(.-)%s*$", "%1"))
        if not handle then
            error("Invalid handle")
        end
        
        -- Parse positions array
        local positions = {}
        for num in string.gmatch(positionsMatch, '[-+%d%.eE]+') do
            positions[#positions+1] = tonumber(num)
        end
        
        if #positions == 0 then
            error("No joint positions found in trajectory command")
        end
        
        return {handle = handle, positions = positions}
    end)
    
    if not success then
        sim.addLog(sim.verbosity_scripterrors, "Failed to parse trajectory command: " .. tostring(result))
        return
    end
    
    local handle = result.handle
    local positions = result.positions
    
    -- Verify the handle matches our UR10 base
    if handle ~= ur10_base_handle then
        sim.addLog(sim.verbosity_scripterrors, "Handle mismatch. Expected: " .. ur10_base_handle .. ", got: " .. handle)
        return
    end
    
    -- Set joint target positions (for dynamic mode)
    local success = true
    for i = 1, math.min(#positions, #ur10_joint_handles) do
        local jointHandle = ur10_joint_handles[i]
        local position = positions[i]
        
        local setSuccess, errorMsg = pcall(sim.setJointTargetPosition, jointHandle, position)
        if not setSuccess then
            sim.addLog(sim.verbosity_scripterrors, "Failed to set joint " .. i .. " target position: " .. tostring(errorMsg))
            success = false
        end
    end
    
    if success then
        sim.addLog(sim.verbosity_msgs, "Successfully set all joint target positions")
    else
        sim.addLog(sim.verbosity_scripterrors, "Some joint target positions failed to set")
    end
end

function publishRG2HandPose()
    if not rg2_pose_publisher or rg2_hand_handle == -1 or not rg2_publisher_active then
        return
    end
    
    -- Get current simulation time
    local sim_time = sim.getSimulationTime()
    local sec = math.floor(sim_time)
    local nsec = math.floor((sim_time - sec) * 1000000000)
    
    -- Get object pose (position and quaternion)
    local pose = sim.getObjectPose(rg2_hand_handle, -1)
    
    if pose and #pose >= 7 then
        -- Create JSON string with pose data
        local pose_json = string.format('{"header":{"stamp":{"sec":%d,"nanosec":%d},"frame_id":"base_link"},"pose":{"position":{"x":%.6f,"y":%.6f,"z":%.6f},"orientation":{"x":%.6f,"y":%.6f,"z":%.6f,"w":%.6f}}}',
            sec, nsec,
            pose[1], pose[2], pose[3],  -- position x, y, z
            pose[4], pose[5], pose[6], pose[7]  -- orientation x, y, z, w
        )
        
        -- Publish the pose as JSON string
        simROS2.publish(rg2_pose_publisher, {data = pose_json})
    end
end

-- Function to manually publish one pose (for testing)
function publishRG2HandPoseOnce()
    if not rg2_publisher_active then
        sim.addLog(sim.verbosity_msgs, "RG2 Hand Pose Publisher is not active. Start it first with startRG2HandPosePublisher()")
        return false
    end
    
    publishRG2HandPose()
    return true
end

-- Function to set publish frequency
function setRG2PublishFrequency(frequency)
    if frequency > 0 then
        rg2_publish_frequency = frequency
        return true
    else
        sim.addLog(sim.verbosity_scripterrors, "Invalid frequency. Must be > 0")
        return false
    end
end

-- Function to get publisher status
function getRG2PublisherStatus()
    local status = {
        active = rg2_publisher_active,
        handle = rg2_hand_handle,
        frequency = rg2_publish_frequency,
        topic = "/coppeliasim/rg2_tipp/pose"
    }
    
    sim.addLog(sim.verbosity_msgs, "RG2 Publisher Status:")
    sim.addLog(sim.verbosity_msgs, "  Active: " .. tostring(status.active))
    sim.addLog(sim.verbosity_msgs, "  Handle: " .. tostring(status.handle))
    sim.addLog(sim.verbosity_msgs, "  Frequency: " .. status.frequency .. " Hz")
    sim.addLog(sim.verbosity_msgs, "  Topic: " .. status.topic)
    
    return status
end

-- Function to debug and list all objects with "rg2" in their name/alias
function listRG2Objects()
    sim.addLog(sim.verbosity_msgs, "Searching for objects with 'rg2' in their alias...")
    
    local objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    local rg2Objects = {}
    
    if objects then
        for i = 1, #objects do
            local handle = objects[i]
            local alias = sim.getObjectAlias(handle, 1)
            
            if alias and string.find(string.lower(alias), "rg2") then
                rg2Objects[#rg2Objects+1] = {handle = handle, alias = alias}
                sim.addLog(sim.verbosity_msgs, "  Handle " .. handle .. ": '" .. alias .. "'")
            end
        end
    end
    
    if #rg2Objects == 0 then
        sim.addLog(sim.verbosity_msgs, "No objects found with 'rg2' in their alias")
    else
        sim.addLog(sim.verbosity_msgs, "Found " .. #rg2Objects .. " objects with 'rg2' in their alias")
    end
    
    return rg2Objects
end

-- Override the main script's actuation function to publish when active
local original_actuation = sysCall_actuation
function sysCall_actuation()
    -- Call original actuation if it exists
    if original_actuation then
        original_actuation()
    end
    
    -- Publish RG2 hand pose if active
    if rg2_publisher_active and simROS2 then
        local current_time = sim.getSimulationTime()
        local time_diff = current_time - rg2_last_publish_time
        
        if time_diff >= (1.0 / rg2_publish_frequency) then
            publishRG2HandPose()
            rg2_last_publish_time = current_time
        end
    end
end

-- Function to debug and list all objects in scene
function listAllObjects()
    sim.addLog(sim.verbosity_msgs, "All objects in scene:")
    local objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    if objects then
        for i = 1, math.min(#objects, 50) do  -- Limit to first 50 objects
            local handle = objects[i]
            local alias = sim.getObjectAlias(handle, 1)
            sim.addLog(sim.verbosity_msgs, "  Handle " .. handle .. ": '" .. (alias or "no alias") .. "'")
        end
        if #objects > 50 then
            sim.addLog(sim.verbosity_msgs, "  ... and " .. (#objects - 50) .. " more objects")
        end
    end
    return objects
end

-- Function to list all UR10 joints
function listUR10Joints()
    sim.addLog(sim.verbosity_msgs, "UR10 Joint Handles:")
    for i = 1, #ur10_joint_handles do
        local handle = ur10_joint_handles[i]
        local alias = sim.getObjectAlias(handle, 1)
        local position = sim.getJointPosition(handle)
        sim.addLog(sim.verbosity_msgs, "  " .. i .. ": " .. alias .. " (handle: " .. handle .. ", pos: " .. position .. ")")
    end
    return ur10_joint_handles
end

-- Function to send test trajectory command
function sendTestTrajectory()
    if not rg2_publisher_active then
        sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper is not active. Start it first with startUR10RG2Helper()")
        return false
    end
    
    -- Create test trajectory with current joint positions
    local positions = {}
    for i = 1, #ur10_joint_handles do
        positions[i] = sim.getJointPosition(ur10_joint_handles[i])
    end
    
    -- Add small offset to test movement
    if positions[1] then
        positions[1] = positions[1] + 0.1  -- Move first joint by 0.1 rad
    end
    
    -- Build positions array string dynamically
    local positionsStr = ""
    for i = 1, #positions do
        if i > 1 then
            positionsStr = positionsStr .. ","
        end
        positionsStr = positionsStr .. string.format("%.3f", positions[i] or 0.0)
    end
    
    local trajectoryJson = string.format('{"handle":%d,"positions":[%s]}',
        ur10_base_handle, positionsStr
    )
    
    
    -- Simulate receiving the message
    trajectory_callback({data = trajectoryJson})
    
    return true
end

-- Help function to show available functions
function showHelp()
    sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper - Available Functions:")
    sim.addLog(sim.verbosity_msgs, "  startUR10RG2Helper(ur10_handle) - Start helper with UR10 handle")
    sim.addLog(sim.verbosity_msgs, "  stopUR10RG2Helper() - Stop helper")
    sim.addLog(sim.verbosity_msgs, "  publishRG2HandPoseOnce() - Publish pose once")
    sim.addLog(sim.verbosity_msgs, "  setRG2PublishFrequency(freq) - Set frequency")
    sim.addLog(sim.verbosity_msgs, "  getRG2PublisherStatus() - Get status")
    sim.addLog(sim.verbosity_msgs, "  listUR10Joints() - List joint handles")
    sim.addLog(sim.verbosity_msgs, "  sendTestTrajectory() - Send test trajectory")
    sim.addLog(sim.verbosity_msgs, "  showHelp() - Show this help message")
    sim.addLog(sim.verbosity_msgs, "")
    sim.addLog(sim.verbosity_msgs, "ROS2 Topics:")
    sim.addLog(sim.verbosity_msgs, "  /coppeliasim/rg2_tipp/pose - RG2 hand pose (std_msgs/msg/String)")
    sim.addLog(sim.verbosity_msgs, "  /ur10_rg2/trajectory - Trajectory commands (std_msgs/msg/String)")
    sim.addLog(sim.verbosity_msgs, "")
    sim.addLog(sim.verbosity_msgs, "Trajectory Format:")
    sim.addLog(sim.verbosity_msgs, '  {"handle":30,"positions":[j1,j2,j3,j4,j5,j6,j7,j8]}')
    sim.addLog(sim.verbosity_msgs, "  Joint order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, rg2_finger1, rg2_finger2")
end

-- Initialization message
sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper script loaded. Type showHelp() for available functions.")
