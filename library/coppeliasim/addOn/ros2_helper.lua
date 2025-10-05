sim=require'sim'
simROS2=require'simROS2'

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
    
    if rawCommand and rawCommand ~= "" then
        -- Debug: log what we received
        sim.addLog(sim.verbosity_msgs, "Received command: " .. tostring(rawCommand))
        
        -- Use the raw command as JSON (standard format)
        local jsonString = rawCommand
        
        if jsonString then
            -- Parse the extracted JSON string
            local operation = nil
            local scenePath = nil
            
            -- Extract operation
            local opMatch = string.match(jsonString, '"operation"%s*:%s*"([^"]+)"')
            if opMatch then
                operation = opMatch
            end
            
            -- Extract scenePath if present
            local pathMatch = string.match(jsonString, '"scenePath"%s*:%s*"([^"]+)"')
            if pathMatch then
                scenePath = pathMatch
            end
            
            if operation then
                if operation == "load" then
                    if scenePath and scenePath ~= "" then
                        -- Load the scene
                        local loadSuccess, errorMsg = pcall(sim.loadScene, scenePath)
                        
                    if loadSuccess then
                        response.data = '{"success":true,"message":"Scene loaded successfully","scenePath":"' .. scenePath .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                    else
                        response.data = '{"success":false,"message":"Failed to load scene","error":"' .. tostring(errorMsg) .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                    end
                    else
                        response.data = '{"success":false,"message":"No scene path provided for load operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                    end
                elseif operation == "close" then
                    -- Close the current scene
                    local closeSuccess, errorMsg = pcall(sim.closeScene)
                    
                    if closeSuccess then
                        response.data = '{"success":true,"message":"Scene closed successfully","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                    else
                        response.data = '{"success":false,"message":"Failed to close scene","error":"' .. tostring(errorMsg) .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                    end
                else
                    response.data = '{"success":false,"message":"Unknown operation","validOperations":["load","close"],"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                end
            else
                response.data = '{"success":false,"message":"Invalid JSON format","expectedFormat":{"operation":"load|close","scenePath":"path/to/scene.ttt (for load only)"},"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
        else
            response.data = '{"success":false,"message":"No JSON string found between single quotes","expectedFormat":"data: \'{\"operation\":\"load\",\"scenePath\":\"/path/to/scene.ttt\"}\'","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
        end
    else
        response.data = '{"success":false,"message":"No command provided","expectedFormat":"data: \'{\"operation\":\"load\",\"scenePath\":\"/path/to/scene.ttt\"}\'","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
    end
    
    -- Publish the response
    simROS2.publish(manageSceneResponsePub, response)
end

function manageModel_callback(msg)
    local rawCommand = msg.data
    local response = {}
    
    if rawCommand and rawCommand ~= "" then
        -- Debug: log what we received
        sim.addLog(sim.verbosity_msgs, "Received model command: " .. tostring(rawCommand))
        
        -- Use the raw command as JSON (standard format)
        local jsonString = rawCommand
        
        if jsonString then
            -- Parse the extracted JSON string
            local operation = nil
            local modelPath = nil
            local position = nil
            local orientation = nil
            local pose = nil
            local objectName = nil
            
            -- Extract operation
            local opMatch = string.match(jsonString, '"operation"%s*:%s*"([^"]+)"')
            if opMatch then
                operation = opMatch
            end
            
            -- Extract modelPath if present
            local pathMatch = string.match(jsonString, '"modelPath"%s*:%s*"([^"]+)"')
            if pathMatch then
                modelPath = pathMatch
            end
            
            -- Extract pose if present (7 numbers array)
            local poseMatch = string.match(jsonString, '"pose"%s*:%s*%[([^%]]+)%]')
            if poseMatch then
                pose = poseMatch
            end
            
            -- Extract position if present (3 numbers array)
            local posMatch = string.match(jsonString, '"position"%s*:%s*%[([^%]]+)%]')
            if posMatch then
                position = posMatch
            end
            
            -- Extract orientation if present (4 numbers array)
            local oriMatch = string.match(jsonString, '"orientation"%s*:%s*%[([^%]]+)%]')
            if oriMatch then
                orientation = oriMatch
            end
            
            -- Extract objectName if present
            local nameMatch = string.match(jsonString, '"objectName"%s*:%s*"([^"]+)"')
            if nameMatch then
                objectName = nameMatch
            end
            
            if operation then
                if operation == "load" then
                    if modelPath and modelPath ~= "" then
                        -- Load the model safely
                        local success, result = pcall(sim.loadModel, modelPath)
                        
                        if success then
                            local handle = result
                            if handle ~= -1 then
                                local messages = {}
                                local success = true
                                
                                -- Set object alias if objectName provided
                                if objectName and objectName ~= "" then
                                    local setAliasOk, setAliasErr = pcall(sim.setObjectAlias, handle, objectName)
                                    if setAliasOk then
                                        messages[#messages+1] = "alias set to '" .. objectName .. "'"
                                    else
                                        messages[#messages+1] = "failed to set alias: " .. tostring(setAliasErr)
                                        success = false
                                    end
                                end
                                
                                -- Optionally set pose if position and orientation provided
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
                                            success = false
                                        end
                                    else
                                        messages[#messages+1] = "invalid pose format"
                                        success = false
                                    end
                                end
                                
                                -- Build response message
                                local message = "Model loaded successfully"
                                if #messages > 0 then
                                    message = message .. "; " .. table.concat(messages, "; ")
                                end
                                
                                response.data = '{"success":' .. tostring(success) .. ',"message":"' .. message .. '","modelPath":"' .. modelPath .. '","handle":' .. handle .. ',"objectName":"' .. (objectName or "") .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                            else
                                response.data = '{"success":false,"message":"Failed to load model","error":"sim.loadModel returned -1","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                            end
                        else
                            response.data = '{"success":false,"message":"Failed to load model","error":"' .. tostring(result) .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                        end
                    else
                        response.data = '{"success":false,"message":"No model path provided for load operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                    end
                elseif operation == "remove" then
                    -- Remove model by handle
                    local handleMatch = string.match(jsonString, '"handle"%s*:%s*([^,}]+)')
                    if handleMatch then
                        local handle = tonumber(handleMatch:gsub("^%s*(.-)%s*$", "%1"))
                        if handle then
                            local removeSuccess, errorMsg = pcall(sim.removeModel, handle)
                            
                            if removeSuccess then
                                response.data = '{"success":true,"message":"Model removed successfully","handle":' .. handle .. ',"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                            else
                                response.data = '{"success":false,"message":"Failed to remove model","error":"' .. tostring(errorMsg) .. '","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                            end
                        else
                            response.data = '{"success":false,"message":"Invalid handle provided","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                        end
                    else
                        response.data = '{"success":false,"message":"No handle provided for remove operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                    end
                elseif operation == "setPose" then
                    -- Set pose of existing object by handle
                    local handleMatch = string.match(jsonString, '"handle"%s*:%s*([^,}]+)')
                    if handleMatch then
                        local handle = tonumber(handleMatch:gsub("^%s*(.-)%s*$", "%1"))
                        if handle then
                            -- Parse pose data from pose array
                            if pose and pose ~= '' then
                                local poseVals = {}
                                for num in string.gmatch(pose, '[-+%d%.eE]+') do
                                    poseVals[#poseVals+1] = tonumber(num)
                                end
                                
                                if #poseVals == 7 then
                                    local setPoseSuccess, errorMsg = pcall(sim.setObjectPose, handle, -1, poseVals)
                                    if setPoseSuccess then
                                        response.data = '{"success":true,"message":"Pose set successfully","handle":' .. handle .. ',"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                                    else
                                        response.data = '{"success":false,"message":"Failed to set pose","error":"' .. tostring(errorMsg) .. '","handle":' .. handle .. ',"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                                    end
                                else
                                    response.data = '{"success":false,"message":"Invalid pose format","expectedFormat":{"pose":"[x,y,z,qx,qy,qz,qw]"},"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                                end
                            else
                                response.data = '{"success":false,"message":"Pose array required for setPose","expectedFormat":{"pose":"[x,y,z,qx,qy,qz,qw]"},"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                            end
                        else
                            response.data = '{"success":false,"message":"Invalid handle provided","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                        end
                    else
                        response.data = '{"success":false,"message":"No handle provided for setPose operation","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                    end
                else
                    response.data = '{"success":false,"message":"Unknown operation","validOperations":["load","remove","setPose"],"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
                end
            else
                response.data = '{"success":false,"message":"Invalid JSON format","expectedFormat":{"operation":"load|remove|setPose","modelPath":"path/to/model.ttm (for load)","handle":"number (for remove/setPose)","position":"[x,y,z] (for load)","orientation":"[qx,qy,qz,qw] (for load)","objectName":"string (for load)","pose":"[x,y,z,qx,qy,qz,qw] (for setPose)"},"timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
            end
        else
            response.data = '{"success":false,"message":"No command provided","expectedFormat":"{\"operation\":\"load\",\"modelPath\":\"/path/to/model.ttm\"}","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
        end
    else
        response.data = '{"success":false,"message":"No command provided","expectedFormat":"{\"operation\":\"load\",\"modelPath\":\"/path/to/model.ttm\"}","timestamp":"' .. os.date("%Y-%m-%d %H:%M:%S") .. '"}'
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

