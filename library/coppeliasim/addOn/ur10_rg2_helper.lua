-- UR10 RG2 Helper - Standalone Addon
-- Automatically detects and manages UR10 RG2 models loaded via ROS2
-- Monitors model load/remove events and auto-starts/stops the helper
sim=require'sim'
simROS2=require'simROS2'

-- Global variables
ur10_base_handle = nil
rg2_hand_handle = nil
ur10_joint_handles = {}
rg2_pose_publisher = nil
joint_state_publisher = nil
trajectory_subscriber = nil
rg2_publish_frequency = 10.0 -- Hz
rg2_last_publish_time = 0
joint_state_last_publish_time = 0
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
    
    -- Verify the handle is valid (with retry for newly loaded objects)
    local objectType = -1
    local retryCount = 0
    local maxRetries = 5
    
    while objectType == -1 and retryCount < maxRetries do
        objectType = sim.getObjectType(ur10_handle)
        if objectType == -1 then
            retryCount = retryCount + 1
            if retryCount < maxRetries then
                sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper: Handle " .. tostring(ur10_handle) .. " not ready, retrying... (" .. retryCount .. "/" .. maxRetries .. ")")
                sim.wait(0.1) -- Wait 100ms before retry
            end
        end
    end
    
    if objectType == -1 then
        sim.addLog(sim.verbosity_scripterrors, "Invalid UR10 handle after " .. maxRetries .. " retries: " .. tostring(ur10_handle))
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
    
    -- Create ROS2 publisher for joint states (using String message with JSON format)
    joint_state_publisher = simROS2.createPublisher('/coppelia/ur10/joint_states_string', 'std_msgs/msg/String')
    
    -- Create ROS2 subscriber for UR10 arm joint commands (6 joints)
    ur10_subscriber = simROS2.createSubscription('/coppelia/ur10/ur10_joints', 'std_msgs/msg/String', 'ur10_callback')
    
    -- Create ROS2 subscriber for RG2 gripper commands (2 joints)
    gripper_subscriber = simROS2.createSubscription('/coppelia/ur10/gripper', 'std_msgs/msg/String', 'gripper_callback')
    
    rg2_publisher_active = true
    trajectory_subscriber_active = true
    rg2_last_publish_time = sim.getSimulationTime()
    
    joint_state_last_publish_time = sim.getSimulationTime()
    
    sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper started successfully")
    sim.addLog(sim.verbosity_msgs, "UR10 base handle: " .. tostring(ur10_base_handle))
    sim.addLog(sim.verbosity_msgs, "RG2 hand handle: " .. tostring(rg2_hand_handle))
    sim.addLog(sim.verbosity_msgs, "Found " .. #ur10_joint_handles .. " joint handles (6 UR10 + 2 RG2)")
    sim.addLog(sim.verbosity_msgs, "Pose topic: /coppeliasim/rg2_tipp/pose")
    sim.addLog(sim.verbosity_msgs, "Joint states topic: /coppelia/ur10/joint_states_string")
    sim.addLog(sim.verbosity_msgs, "UR10 arm topic: /coppelia/ur10/ur10_joints")
    sim.addLog(sim.verbosity_msgs, "RG2 gripper topic: /coppelia/ur10/gripper")
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
        if joint_state_publisher then
            simROS2.shutdownPublisher(joint_state_publisher)
        end
        if ur10_subscriber then
            simROS2.shutdownSubscription(ur10_subscriber)
        end
        if gripper_subscriber then
            simROS2.shutdownSubscription(gripper_subscriber)
        end
    end
    
    rg2_publisher_active = false
    trajectory_subscriber_active = false
    rg2_pose_publisher = nil
    joint_state_publisher = nil
    ur10_subscriber = nil
    gripper_subscriber = nil
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

-- UR10 arm joints callback function (controls first 6 joints only)
function ur10_callback(msg)
    if not trajectory_subscriber_active then
        return
    end
    
    local ur10Data = msg.data
    
    -- Debug: log received data
    sim.addLog(sim.verbosity_msgs, "UR10 callback received: " .. tostring(ur10Data))
    
    -- Parse JSON string
    local success, result = pcall(function()
        -- Simple JSON parsing for UR10 command format
        local handleMatch = string.match(ur10Data, '"handle"%s*:%s*([^,}]+)')
        local positionsMatch = string.match(ur10Data, '"positions"%s*:%s*%[([^%]]+)%]')
        
        if not handleMatch then
            error("Invalid JSON format: missing 'handle' field")
        end
        
        if not positionsMatch then
            error("Invalid JSON format: missing or empty 'positions' array")
        end
        
        local handle = tonumber(handleMatch:gsub("^%s*(.-)%s*$", "%1"))
        if not handle then
            error("Invalid handle")
        end
        
        -- Parse positions array (should be 6 values for UR10)
        local positions = {}
        for num in string.gmatch(positionsMatch, '[-+%d%.eE]+') do
            positions[#positions+1] = tonumber(num)
        end
        
        if #positions < 6 then
            error("Need at least 6 UR10 joint positions")
        end
        
        return {handle = handle, positions = positions}
    end)
    
    if not success then
        sim.addLog(sim.verbosity_scripterrors, "Failed to parse UR10 command: " .. tostring(result))
        return
    end
    
    local handle = result.handle
    local positions = result.positions
    
    -- Verify the handle matches our UR10 base
    if handle ~= ur10_base_handle then
        sim.addLog(sim.verbosity_scripterrors, "Handle mismatch. Expected: " .. ur10_base_handle .. ", got: " .. handle)
        return
    end
    
    -- Set UR10 joint target positions only (first 6 joints)
    local success = true
    for i = 1, math.min(#positions, 6) do
        local jointHandle = ur10_joint_handles[i]
        local position = positions[i]
        
        if jointHandle then
            local setSuccess, errorMsg = pcall(sim.setJointTargetPosition, jointHandle, position)
            if not setSuccess then
                sim.addLog(sim.verbosity_scripterrors, "Failed to set UR10 joint " .. i .. " target position: " .. tostring(errorMsg))
                success = false
            end
        end
    end
    
    if success then
        sim.addLog(sim.verbosity_msgs, "Successfully set UR10 arm target positions")
    else
        sim.addLog(sim.verbosity_scripterrors, "Some UR10 joint positions failed to set")
    end
end

-- Gripper callback function (controls gripper joints only)
function gripper_callback(msg)
    if not trajectory_subscriber_active then
        return
    end
    
    local gripperData = msg.data
    
    -- Parse JSON string
    local success, result = pcall(function()
        -- Simple JSON parsing for gripper command format
        local handleMatch = string.match(gripperData, '"handle"%s*:%s*([^,}]+)')
        local positionsMatch = string.match(gripperData, '"positions"%s*:%s*%[([^%]]+)%]')
        
        if not handleMatch or not positionsMatch then
            error("Invalid JSON format")
        end
        
        local handle = tonumber(handleMatch:gsub("^%s*(.-)%s*$", "%1"))
        if not handle then
            error("Invalid handle")
        end
        
        -- Parse positions array (should be 2 values for gripper)
        local positions = {}
        for num in string.gmatch(positionsMatch, '[-+%d%.eE]+') do
            positions[#positions+1] = tonumber(num)
        end
        
        if #positions < 2 then
            error("Need at least 2 gripper joint positions")
        end
        
        return {handle = handle, positions = positions}
    end)
    
    if not success then
        sim.addLog(sim.verbosity_scripterrors, "Failed to parse gripper command: " .. tostring(result))
        return
    end
    
    local handle = result.handle
    local positions = result.positions
    
    -- Verify the handle matches our UR10 base
    if handle ~= ur10_base_handle then
        sim.addLog(sim.verbosity_scripterrors, "Handle mismatch. Expected: " .. ur10_base_handle .. ", got: " .. handle)
        return
    end
    
    -- Set gripper joint target positions only (last 2 joints)
    local gripperStartIndex = #ur10_joint_handles - 1  -- Index of first gripper joint
    local success = true
    for i = 1, math.min(#positions, 2) do
        local jointIndex = gripperStartIndex + i - 1
        local jointHandle = ur10_joint_handles[jointIndex]
        local position = positions[i]
        
        if jointHandle then
            local setSuccess, errorMsg = pcall(sim.setJointTargetPosition, jointHandle, position)
            if not setSuccess then
                sim.addLog(sim.verbosity_scripterrors, "Failed to set gripper joint " .. i .. " target position: " .. tostring(errorMsg))
                success = false
            end
        end
    end
    
    if success then
        sim.addLog(sim.verbosity_msgs, "Successfully set RG2 gripper target positions")
    else
        sim.addLog(sim.verbosity_scripterrors, "Some gripper joint positions failed to set")
    end
end

-- Legacy trajectory callback function (for backward compatibility - controls all 8 joints)
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

-- Function to publish joint states as JSON string
function publishJointStates()
    if not joint_state_publisher or not rg2_publisher_active then
        return
    end
    
    -- Get joint names
    local joint_names = {
        "shoulder_pan_joint",
        "shoulder_lift_joint", 
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        "rg2_finger_joint1",
        "rg2_finger_joint2"
    }
    
    -- Get joint positions from all joints (UR10 + RG2)
    local positions = {}
    for i = 1, #ur10_joint_handles do
        positions[i] = sim.getJointPosition(ur10_joint_handles[i])
    end
    
    -- If we have less than 8 joints, pad with zeros
    while #positions < 8 do
        positions[#positions + 1] = 0.0
    end
    
    -- Build JSON string
    local names_str = '["' .. table.concat(joint_names, '","') .. '"]'
    local positions_str = "[" .. table.concat(positions, ",") .. "]"
    
    local joint_state_json = string.format('{"names":%s,"positions":%s,"velocities":[],"efforts":[]}',
        names_str, positions_str)
    
    -- Publish
    simROS2.publish(joint_state_publisher, {data = joint_state_json})
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

-- Note: sysCall_actuation integration is now handled by ros2_helper.lua
-- The actuation logic calls publishRG2HandPose() and publishJointStates()
-- when rg2_publisher_active is true

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
    sim.addLog(sim.verbosity_msgs, "  /coppelia/ur10/ur10_joints - UR10 arm control (6 joints, std_msgs/msg/String)")
    sim.addLog(sim.verbosity_msgs, "  /coppelia/ur10/gripper - RG2 gripper control (2 joints, std_msgs/msg/String)")
    sim.addLog(sim.verbosity_msgs, "")
    sim.addLog(sim.verbosity_msgs, "UR10 Joints Format:")
    sim.addLog(sim.verbosity_msgs, '  {"handle":30,"positions":[j1,j2,j3,j4,j5,j6]}')
    sim.addLog(sim.verbosity_msgs, "  Joint order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3")
    sim.addLog(sim.verbosity_msgs, "")
    sim.addLog(sim.verbosity_msgs, "Gripper Format:")
    sim.addLog(sim.verbosity_msgs, '  {"handle":30,"positions":[finger1, finger2]}')
    sim.addLog(sim.verbosity_msgs, "  finger1 = rg2_finger_joint1, finger2 = rg2_finger_joint2")
end

-- Initialization message
sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper script loaded. Type showHelp() for available functions.")

--------------------------------------------------------------------------------
-- Standalone Addon System Callbacks
--------------------------------------------------------------------------------

-- Subscription for monitoring model events
local modelResponseSub = nil


-- Callback for model management response
function ur10_rg2_modelResponse_callback(msg)
    local responseData = msg.data
    
    -- Parse JSON response to detect UR10 RG2 model loads
    local modelPath = string.match(responseData, '"modelPath"%s*:%s*"([^"]+)"')
    local handle = string.match(responseData, '"handle"%s*:%s*([^,}]+)')
    local modelType = string.match(responseData, '"modelType"%s*:%s*"([^"]+)"')
    local success = string.match(responseData, '"success"%s*:%s*(%a+)')
    local message = string.match(responseData, '"message"%s*:%s*"([^"]*)"')
    
    -- Handle model removal (detect if our UR10 base handle was removed)
    if success == "true" and message and string.match(message, "Model removed successfully") and handle then
        handle = tonumber(handle)
        if rg2_publisher_active and ur10_base_handle == handle then
            sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper: Detected removal of UR10 handle " .. handle .. ". Auto-stopping...")
            stopUR10RG2Helper()
        end
    end
    
    -- Handle model load
    if modelPath and handle and modelType == "urdf" then
        handle = tonumber(handle)
        
        -- Check if this is the UR10 RG2 URDF
        if string.match(modelPath, "ur10_rg2_coppelia%.urdf") then
            sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper: Detected UR10 RG2 URDF load. Auto-starting...")
            
            -- Start the UR10 RG2 Helper (ROS2 publishers/subscribers)
            local helperStarted = startUR10RG2Helper(handle)
            if helperStarted then
                sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper: Started successfully for handle " .. handle)
                sim.addLog(sim.verbosity_msgs, "Note: MoveIt stack and WoT server are managed by cs_controller.js")
            else
                sim.addLog(sim.verbosity_scripterrors, "UR10 RG2 Helper: Failed to auto-start")
            end
        end
    end
end

-- Initialize the addon
function sysCall_init_ur10_rg2()
    if simROS2 then
        sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper: Initializing standalone addon...")
        
        -- Subscribe to model management responses to detect UR10 RG2 loads
        modelResponseSub = simROS2.createSubscription(
            '/coppeliasim/manageModelResponse',
            'std_msgs/msg/String',
            'ur10_rg2_modelResponse_callback'
        )
        
        sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper: Monitoring for UR10 RG2 model loads...")
    else
        sim.addLog(sim.verbosity_scripterrors, "UR10 RG2 Helper: ROS2 interface not found!")
    end
end

-- Handle actuation cycle - publish RG2 pose and joint states if active
function sysCall_actuation_ur10_rg2()
    if rg2_publisher_active and simROS2 then
        -- Check if handles are still valid before publishing
        if ur10_base_handle then
            local objectType = sim.getObjectType(ur10_base_handle)
            if objectType == -1 then
                -- Handle is invalid, UR10 was probably removed
                sim.addLog(sim.verbosity_warnings, "UR10 RG2 Helper: Detected invalid handle " .. ur10_base_handle .. ". Stopping helper...")
                stopUR10RG2Helper()
                return
            end
        end
        
        local current_time = sim.getSimulationTime()
        local time_diff = current_time - rg2_last_publish_time
        local joint_time_diff = current_time - joint_state_last_publish_time
        
        if time_diff >= (1.0 / rg2_publish_frequency) then
            local success, errorMsg = pcall(publishRG2HandPose)
            if not success then
                sim.addLog(sim.verbosity_warnings, "UR10 RG2 Helper: Error publishing pose: " .. tostring(errorMsg))
                stopUR10RG2Helper()
                return
            end
            rg2_last_publish_time = current_time
        end
        
        if joint_time_diff >= (1.0 / rg2_publish_frequency) then
            local success, errorMsg = pcall(publishJointStates)
            if not success then
                sim.addLog(sim.verbosity_warnings, "UR10 RG2 Helper: Error publishing joint states: " .. tostring(errorMsg))
                stopUR10RG2Helper()
                return
            end
            joint_state_last_publish_time = current_time
        end
    end
end

-- Cleanup on addon shutdown
function sysCall_cleanup_ur10_rg2()
    if rg2_publisher_active then
        sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper: Cleanup - Stopping helper...")
        stopUR10RG2Helper()
    end
    
    if simROS2 and modelResponseSub then
        simROS2.shutdownSubscription(modelResponseSub)
    end
    
    sim.addLog(sim.verbosity_msgs, "UR10 RG2 Helper: Addon shutdown complete")
    sim.addLog(sim.verbosity_msgs, "Note: MoveIt stack and WoT server cleanup handled by cs_controller.js")
end
