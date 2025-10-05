sim=require'sim'
simROS2=require'simROS2'
simURDF = require'simURDF'

function sysCall_init()
    if simROS2 then
        sim.addLog(sim.verbosity_msgs,"ROS2 interface was found.")
        startSimService=simROS2.createService('/startCoppeliaSim', 'std_srvs/srv/Trigger', 'startSimulation_service_callback')
        pauseSimService=simROS2.createService('/pauseCoppeliaSim', 'std_srvs/srv/Trigger', 'pauseSimulation_service_callback')
        stopSimService=simROS2.createService('/stopCoppeliaSim', 'std_srvs/srv/Trigger', 'stopSimulation_service_callback')
        manageSceneSub=simROS2.createSubscription('/manageScene', 'std_msgs/msg/String', 'manageScene_callback')
        manageSceneResponsePub=simROS2.createPublisher('/manageSceneResponse', 'std_msgs/msg/String')
        manageModelSub=simROS2.createSubscription('/manageModel', 'std_msgs/msg/String', 'manageModel_callback')
        manageModelResponsePub=simROS2.createPublisher('/manageModelResponse', 'std_msgs/msg/String')
        enableSynModeSub=simROS2.createSubscription('/enableSyncMode', 'std_msgs/msg/Bool', 'enableSyncMode_callback')
        triggerNextStepSub=simROS2.createSubscription('/triggerNextStep', 'std_msgs/msg/Bool', 'triggerNextStep_callback')

        simStepDonePub=simROS2.createPublisher('/simulationStepDone', 'std_msgs/msg/Bool')
        simStatePub=simROS2.createPublisher('/simulationState','std_msgs/msg/Int32')
        simTimePub=simROS2.createPublisher('/simulationTime','std_msgs/msg/Float32')
        auxPub=simROS2.createPublisher('/privateMsgAux', 'std_msgs/msg/Bool')
        auxSub=simROS2.createSubscription('/privateMsgAux', 'std_msgs/msg/Bool', 'aux_callback')
        
        rosInterfaceSynModeEnabled=false
        haltMainScript=false
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS2 interface was not found. Cannot run.")
    end
end
function startSimulation_service_callback(request, response)
    local result = {}
    
    -- Just start the simulation
    local success, errorMsg = pcall(sim.startSimulation)
    
    if success then
        result.success = true
        result.message = "Simulation start command executed successfully"
    else
        result.success = false
        result.message = "Failed to start simulation: " .. tostring(errorMsg)
    end
    return result
end

function pauseSimulation_service_callback(request, response)
    local result = {}
    
    -- Just pause the simulation
    local success, errorMsg = pcall(sim.pauseSimulation)
    
    if success then
        result.success = true
        result.message = "Simulation pause command executed successfully"
    else
        result.success = false
        result.message = "Failed to pause simulation: " .. tostring(errorMsg)
    end
    return result
end

function stopSimulation_service_callback(request, response)
    local result = {}
    
    -- Just stop the simulation
    local success, errorMsg = pcall(sim.stopSimulation)
    
    if success then
        result.success = true
        result.message = "Simulation stop command executed successfully"
    else
        result.success = false
        result.message = "Failed to stop simulation: " .. tostring(errorMsg)
    end
    return result
end

function manageScene_callback(msg)
    local rawCommand = msg.data
    local response = {}
    
    if not rawCommand or rawCommand == "" then
        response.data = '{"success":false,"message":"No command provided","expectedFormat":"data: \'{\"operation\":\"load|close|save\",\"scenePath\":\"/path/to/scene.ttt\"}\'","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
        simROS2.publish(manageSceneResponsePub, response)
        return
    end
    
    -- Debug: log what we received
    sim.addLog(sim.verbosity_msgs, "Received command: " .. tostring(rawCommand))
    
    -- Parse JSON string
    local operation = string.match(rawCommand, '"operation"%s*:%s*"([^"]+)"')
    local scenePath = string.match(rawCommand, '"scenePath"%s*:%s*"([^"]+)"')
    
    if not operation then
        response.data = '{"success":false,"message":"Invalid JSON format","expectedFormat":{"operation":"load|close|save","scenePath":"path/to/scene.ttt (for load and save)"},"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
        simROS2.publish(manageSceneResponsePub, response)
        return
    end
    
    -- Dispatch table for operations
    local operations = {
        load = function()
            if not scenePath or scenePath == "" then
                return '{"success":false,"message":"No scene path provided for load operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            local loadSuccess, errorMsg = pcall(sim.loadScene, scenePath)
            if loadSuccess then
                return '{"success":true,"message":"Scene loaded successfully","scenePath":"' .. scenePath .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            else
                return '{"success":false,"message":"Failed to load scene","error":"' .. tostring(errorMsg) .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
        end,
        
        close = function()
            local closeSuccess, errorMsg = pcall(sim.closeScene)
            if closeSuccess then
                return '{"success":true,"message":"Scene closed successfully","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            else
                return '{"success":false,"message":"Failed to close scene","error":"' .. tostring(errorMsg) .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
        end,
        
        save = function()
            if not scenePath or scenePath == "" then
                return '{"success":false,"message":"No scene path provided for save operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            -- Check if scene path has correct extension
            if not string.match(scenePath, "%.ttt$") then
                return '{"success":false,"message":"Invalid file extension","error":"Scene files must have .ttt extension","scenePath":"' .. scenePath .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            local saveSuccess, result = pcall(sim.saveScene, scenePath)
            sim.addLog(sim.verbosity_msgs, "Save result: success=" .. tostring(saveSuccess) .. ", result=" .. tostring(result) .. ", type=" .. type(result))
            if saveSuccess then
                -- sim.saveScene returns 1 on success, -1 on failure
                if result == 1 then
                    return '{"success":true,"message":"Scene saved successfully","scenePath":"' .. scenePath .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                else
                    local errorMsg = "sim.saveScene returned " .. tostring(result)
                    return '{"success":false,"message":"Failed to save scene","error":"' .. errorMsg .. '","scenePath":"' .. scenePath .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                end
            else
                return '{"success":false,"message":"Failed to save scene","error":"' .. tostring(result) .. '","scenePath":"' .. scenePath .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
        end
    }
    
    -- Execute the operation
    local operationFunc = operations[operation]
    if operationFunc then
        response.data = operationFunc()
    else
        response.data = '{"success":false,"message":"Unknown operation","validOperations":["load","close","save"],"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
    end
    
    -- Publish the response
    simROS2.publish(manageSceneResponsePub, response)
end

function manageModel_callback(msg)
    local rawCommand = msg.data
    local response = {}
    
    if not rawCommand or rawCommand == "" then
        response.data = '{"success":false,"message":"No command provided","expectedFormat":"{\"operation\":\"load\",\"modelPath\":\"/path/to/model.ttm\"}","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
        simROS2.publish(manageModelResponsePub, response)
        return
    end
    
    -- Debug: log what we received
    sim.addLog(sim.verbosity_msgs, "Received model command: " .. tostring(rawCommand))
    
    -- Parse JSON string
    local operation = string.match(rawCommand, '"operation"%s*:%s*"([^"]+)"')
    local modelPath = string.match(rawCommand, '"modelPath"%s*:%s*"([^"]+)"')
    local position = string.match(rawCommand, '"position"%s*:%s*%[([^%]]+)%]')
    local orientation = string.match(rawCommand, '"orientation"%s*:%s*%[([^%]]+)%]')
    local pose = string.match(rawCommand, '"pose"%s*:%s*%[([^%]]+)%]')
    local objectName = string.match(rawCommand, '"objectName"%s*:%s*"([^"]+)"')
    
    if not operation then
        response.data = '{"success":false,"message":"Invalid JSON format","expectedFormat":{"operation":"load|remove|setPose","modelPath":"path/to/model.ttm (for load)","handle":"number (for remove/setPose)","position":"[x,y,z] (for load)","orientation":"[qx,qy,qz,qw] (for load)","objectName":"string (for load)","pose":"[x,y,z,qx,qy,qz,qw] (for setPose)"},"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
        simROS2.publish(manageModelResponsePub, response)
        return
    end
    
    -- Dispatch table for operations
    local operations = {
        load = function()
            if not modelPath or modelPath == "" then
                return '{"success":false,"message":"No model path provided for load operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            -- Check if the file is a URDF file by extension
            local isUrdf = string.match(modelPath, "%.urdf$")
            local success, result
            local handle = -1  -- Initialize handle to -1
            
            if isUrdf then
                -- Use URDF importer for .urdf files
                success, urdfObjectName, handleTable = pcall(simURDF.import, modelPath)
                sim.addLog(sim.verbosity_msgs, "URDF import result: success=" .. tostring(success) .. ", urdfObjectName=" .. tostring(urdfObjectName) .. ", handleTable=" .. tostring(handleTable) .. ", type=" .. type(handleTable))
                if success and handleTable and type(handleTable) == "table" and #handleTable > 0 then
                    handle = handleTable[1]
                else
                    success = false
                    result = "URDF import failed or returned no handles"
                end
            else
                -- Use regular model loader for other files
                success, result = pcall(sim.loadModel, modelPath)
                if success then
                    handle = result
                end
            end
            
            if not success or handle == -1 then
                local errorMsg = isUrdf and "simURDF.import returned no valid handles" or "sim.loadModel returned -1"
                if not success then
                    errorMsg = tostring(result)
                end
                return '{"success":false,"message":"Failed to load model","error":"' .. errorMsg .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            -- Handle post-load operations
            local messages = {}
            local overallSuccess = true
            
            -- Set object alias if objectName provided
            if objectName and objectName ~= "" then
                local setAliasOk, setAliasErr = pcall(sim.setObjectAlias, handle, objectName)
                if setAliasOk then
                    messages[#messages+1] = "alias set to '" .. objectName .. "'"
                else
                    messages[#messages+1] = "failed to set alias: " .. tostring(setAliasErr)
                    overallSuccess = false
                end
            end
            
            -- Set pose if position and orientation provided
            if position and orientation then
                local posVals = {}
                for num in string.gmatch(position, '[-+%d%.eE]+') do
                    posVals[#posVals+1] = tonumber(num)
                end
                local oriVals = {}
                for num in string.gmatch(orientation, '[-+%d%.eE]+') do
                    oriVals[#oriVals+1] = tonumber(num)
                end
                
                if #posVals == 3 and #oriVals == 4 then
                    local poseToApply = {posVals[1], posVals[2], posVals[3], oriVals[1], oriVals[2], oriVals[3], oriVals[4]}
                    local setPoseOk, setPoseErr = pcall(sim.setObjectPose, handle, -1, poseToApply)
                    if setPoseOk then
                        messages[#messages+1] = "pose set"
                    else
                        messages[#messages+1] = "failed to set pose: " .. tostring(setPoseErr)
                        overallSuccess = false
                    end
                else
                    messages[#messages+1] = "invalid pose format"
                    overallSuccess = false
                end
            end
            
            -- Build response message
            local message = isUrdf and "URDF model loaded successfully" or "Model loaded successfully"
            if #messages > 0 then
                message = message .. "; " .. table.concat(messages, "; ")
            end
            
            local modelType = isUrdf and "urdf" or "ttm"
            -- Ensure handle is never nil
            if handle == nil then
                handle = -1
            end
            return '{"success":' .. tostring(overallSuccess) .. ',"message":"' .. message .. '","modelPath":"' .. modelPath .. '","modelType":"' .. modelType .. '","handle":' .. handle .. ',"objectName":"' .. (objectName or "") .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
        end,
        
        remove = function()
            local handleMatch = string.match(rawCommand, '"handle"%s*:%s*([^,}]+)')
            if not handleMatch then
                return '{"success":false,"message":"No handle provided for remove operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            local handle = tonumber(handleMatch:gsub("^%s*(.-)%s*$", "%1"))
            if not handle then
                return '{"success":false,"message":"Invalid handle provided","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            local removeSuccess, errorMsg = pcall(sim.removeModel, handle)
            if removeSuccess then
                return '{"success":true,"message":"Model removed successfully","handle":' .. handle .. ',"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            else
                return '{"success":false,"message":"Failed to remove model","error":"' .. tostring(errorMsg) .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
        end,
        
        setPose = function()
            local handleMatch = string.match(rawCommand, '"handle"%s*:%s*([^,}]+)')
            if not handleMatch then
                return '{"success":false,"message":"No handle provided for setPose operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            local handle = tonumber(handleMatch:gsub("^%s*(.-)%s*$", "%1"))
            if not handle then
                return '{"success":false,"message":"Invalid handle provided","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            if not pose or pose == '' then
                return '{"success":false,"message":"Pose array required for setPose","expectedFormat":{"pose":"[x,y,z,qx,qy,qz,qw]"},"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            local poseVals = {}
            for num in string.gmatch(pose, '[-+%d%.eE]+') do
                poseVals[#poseVals+1] = tonumber(num)
            end
            
            if #poseVals ~= 7 then
                return '{"success":false,"message":"Invalid pose format","expectedFormat":{"pose":"[x,y,z,qx,qy,qz,qw]"},"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
            
            local setPoseSuccess, errorMsg = pcall(sim.setObjectPose, handle, -1, poseVals)
            if setPoseSuccess then
                return '{"success":true,"message":"Pose set successfully","handle":' .. handle .. ',"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            else
                return '{"success":false,"message":"Failed to set pose","error":"' .. tostring(errorMsg) .. '","handle":' .. handle .. ',"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
        end
    }
    
    -- Execute the operation
    local operationFunc = operations[operation]
    if operationFunc then
        response.data = operationFunc()
    else
        response.data = '{"success":false,"message":"Unknown operation","validOperations":["load","remove","setPose"],"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
    end
    
    -- Publish the response
    simROS2.publish(manageModelResponsePub, response)
end

function enableSyncMode_callback(msg)
    rosInterfaceSynModeEnabled=msg.data
    haltMainScript=rosInterfaceSynModeEnabled
end

function triggerNextStep_callback(msg)
    haltMainScript=false
end

function aux_callback(msg)
    simROS2.publish(simStepDonePub,{data=true})
end

function publishSimState()
    local state=0 -- simulation not running
    local s=sim.getSimulationState()
    if s==sim.simulation_paused then
        state=2 -- simulation paused
    elseif s==sim.simulation_stopped then
        state=0 -- simulation stopped
    else
        state=1 -- simulation running
    end
    simROS2.publish(simStatePub,{data=state})
end


function sysCall_nonSimulation()
    if simROS2 then
        publishSimState()
    end
end

function sysCall_beforeMainScript()
    return {doNotRunMainScript=haltMainScript}
end

function sysCall_actuation()
    if simROS2 then
        publishSimState()
        simROS2.publish(simTimePub,{data=sim.getSimulationTime()})
    end
end

function sysCall_sensing()
    if simROS2 then
        simROS2.publish(auxPub,{data=true})
        haltMainScript=rosInterfaceSynModeEnabled
    end
end

function sysCall_suspended()
    if simROS2 then
        publishSimState()
    end
end

function sysCall_afterSimulation()
    if simROS2 then
        publishSimState()
    end
end

function sysCall_cleanup()
    if simROS2 then
        simROS2.shutdownService(startSimService)
        simROS2.shutdownService(pauseSimService)
        simROS2.shutdownService(stopSimService)
        simROS2.shutdownSubscription(manageSceneSub)
        simROS2.shutdownPublisher(manageSceneResponsePub)
        simROS2.shutdownSubscription(manageModelSub)
        simROS2.shutdownPublisher(manageModelResponsePub)
        simROS2.shutdownSubscription(enableSynModeSub)
        simROS2.shutdownSubscription(triggerNextStepSub)
        simROS2.shutdownSubscription(auxSub)
        simROS2.shutdownPublisher(auxPub)
        simROS2.shutdownPublisher(simStepDonePub)
        simROS2.shutdownPublisher(simStatePub)
        simROS2.shutdownPublisher(simTimePub)
    end
end

