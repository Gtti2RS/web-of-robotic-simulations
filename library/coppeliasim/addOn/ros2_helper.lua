sim=require'sim'
simROS2=require'simROS2'

function sysCall_init()
    if simROS2 then
        sim.addLog(sim.verbosity_msgs,"ROS2 interface was found.")
        startSimService=simROS2.createService('/startCoppeliaSim', 'std_srvs/srv/Trigger', 'startSimulation_service_callback')
        pauseSimService=simROS2.createService('/pauseCoppeliaSim', 'std_srvs/srv/Trigger', 'pauseSimulation_service_callback')
        stopSimService=simROS2.createService('/stopCoppeliaSim', 'std_srvs/srv/Trigger', 'stopSimulation_service_callback')
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
        simROS2.shutdownSubscription(enableSynModeSub)
        simROS2.shutdownSubscription(triggerNextStepSub)
        simROS2.shutdownSubscription(auxSub)
        simROS2.shutdownPublisher(auxPub)
        simROS2.shutdownPublisher(simStepDonePub)
        simROS2.shutdownPublisher(simStatePub)
        simROS2.shutdownPublisher(simTimePub)
    end
end

