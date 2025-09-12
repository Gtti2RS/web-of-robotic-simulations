// Workspace directory
const WORKSPACE_DIR = '/home/evan/wos';

// Use CoppeliaSim helper to load RemoteAPIClient from installation directory
const { RemoteAPIClient } = require('./coppelia_helper');
const { resolveFilePath, readAvailableResources } = require('../common/fileUtils');
const { deg2quat } = require('../common/deg2quat');
const { loadUrdf } = require('./urdf_loader');
const { getCoppeliaSimObjects, entityExist } = require('./get_object_list');
const { setupPropertyHandlers } = require('./cs_properties');

// CoppeliaSim connection class
class CoppeliaSimConnection {
    constructor(host = 'localhost', port = 23050) {
        this.client = new RemoteAPIClient(host, port, 'json');
        this.sim = null;
        this.connected = false;
    }

    async connect() {
        try {
            console.log(`[${new Date().toLocaleTimeString()}] CONN: Connecting...`);
            await this.client.websocket.open();
            this.sim = await this.client.getObject('sim');
            this.connected = true;
            console.log(`[${new Date().toLocaleTimeString()}] CONN: Connected ✓`);
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] CONN: Failed - ${error.message}`);
            this.connected = false;
        }
    }

    async disconnect() {
        if (this.connected && this.client.websocket) {
            await this.client.websocket.close();
            this.connected = false;
            console.log(`[${new Date().toLocaleTimeString()}] CONN: Disconnected`);
        }
    }

    async getSimulationState() {
        if (!this.connected) {
            return -1; // Return -1 to indicate no connection
        }
        return Number(await this.sim.getSimulationState());
    }

    async startSimulation() {
        if (!this.connected) {
            throw new Error('Not connected to CoppeliaSim');
        }
        console.log(`[${new Date().toLocaleTimeString()}] SIM: Starting...`);
        await this.sim.startSimulation();
        await this.delay(500);
        console.log(`[${new Date().toLocaleTimeString()}] SIM: Started ✓`);
    }

    async pauseSimulation() {
        if (!this.connected) {
            throw new Error('Not connected to CoppeliaSim');
        }
        console.log(`[${new Date().toLocaleTimeString()}] SIM: Pausing...`);
        await this.sim.pauseSimulation();
        await this.delay(500);
        console.log(`[${new Date().toLocaleTimeString()}] SIM: Paused ✓`);
    }

    async stopSimulation() {
        if (!this.connected) {
            throw new Error('Not connected to CoppeliaSim');
        }
        console.log(`[${new Date().toLocaleTimeString()}] SIM: Stopping...`);
        await this.sim.stopSimulation();
        await this.delay(500);
        console.log(`[${new Date().toLocaleTimeString()}] SIM: Stopped ✓`);
    }

    async loadScene(fileName) {
        if (!this.connected) {
            throw new Error('Not connected to CoppeliaSim');
        }
        
        // Resolve the full path from filename using fileUtils
        const scenePath = await resolveFilePath(fileName);
        if (!scenePath) {
            throw new Error(`File not found: ${fileName}`);
        }
        
        const filename = fileName.split('/').pop();
        console.log(`[${new Date().toLocaleTimeString()}] SCENE: Loading ${filename}...`);
        await this.sim.loadScene(scenePath);
        await this.delay(1000);
        console.log(`[${new Date().toLocaleTimeString()}] SCENE: Loaded ✓`);
    }

    async saveScene(filename) {
        if (!this.connected) {
            throw new Error('Not connected to CoppeliaSim');
        }
        // Always save under workspace/saved/world directory - extract just the filename
        const baseFilename = filename.split('/').pop(); // Get just the filename part
        const fullPath = `${WORKSPACE_DIR}/saved/world/${baseFilename}`;
        console.log(`[${new Date().toLocaleTimeString()}] SAVE: Saving ${baseFilename}...`);
        await this.sim.saveScene(fullPath);
        await this.delay(500);
        console.log(`[${new Date().toLocaleTimeString()}] SAVE: Saved ✓`);
    }

    async closeScene() {
        if (!this.connected) {
            throw new Error('Not connected to CoppeliaSim');
        }
        console.log(`[${new Date().toLocaleTimeString()}] SCENE: Closing...`);
        await this.sim.closeScene();
        await this.delay(500);
        console.log(`[${new Date().toLocaleTimeString()}] SCENE: Closed ✓`);
    }

    async spawnEntity(entityName, fileName, position = {x: 0, y: 0, z: 0}, orientation = {roll: 0, pitch: 0, yaw: 0}) {
        if (!this.connected) {
            throw new Error('Not connected to CoppeliaSim');
        }

        // Resolve the full path from filename using fileUtils
        const filePath = await resolveFilePath(fileName);
        if (!filePath) {
            throw new Error(`File not found: ${fileName}`);
        }

        // Determine file type and spawn accordingly
        const fileExtension = fileName.toLowerCase().split('.').pop();
        let objectHandle;

        console.log(`[${new Date().toLocaleTimeString()}] SPAWN: Loading ${fileName} as ${entityName}...`);

        if (fileExtension === 'ttm') {
            // Load CoppeliaSim model file
            const loadResult = await this.sim.loadModel(filePath);
            console.log(`[${new Date().toLocaleTimeString()}] SPAWN: TTM model loaded ✓`);
            
            // Handle different return types from loadModel
            if (Array.isArray(loadResult)) {
                objectHandle = loadResult[0];
                console.log(`[${new Date().toLocaleTimeString()}] SPAWN: Model returned array, using first handle: ${objectHandle}`);
            } else if (typeof loadResult === 'object' && loadResult.handle !== undefined) {
                objectHandle = loadResult.handle;
                console.log(`[${new Date().toLocaleTimeString()}] SPAWN: Model returned object, using handle: ${objectHandle}`);
            } else if (typeof loadResult === 'number') {
                objectHandle = loadResult;
                console.log(`[${new Date().toLocaleTimeString()}] SPAWN: Model returned handle: ${objectHandle}`);
            } else {
                throw new Error(`Unexpected TTM model handle type: ${typeof loadResult}, value: ${loadResult}`);
            }
        } else if (fileExtension === 'urdf') {
            // Use the loadUrdf function to import URDF with existing sim connection
            objectHandle = await loadUrdf(filePath, entityName, this.sim);
            if (objectHandle === -1) {
                throw new Error('Failed to load URDF using loadUrdf function');
            }
            console.log(`[${new Date().toLocaleTimeString()}] SPAWN: URDF model imported with handle: ${objectHandle} ✓`);
        } else {
            throw new Error(`Unsupported file type: ${fileExtension}. Only .ttm and .urdf files are supported.`);
        }

        // Validate the handle is a valid number
        if (typeof objectHandle !== 'number' || !Number.isFinite(objectHandle)) {
            throw new Error(`Invalid object handle: ${objectHandle} (type: ${typeof objectHandle})`);
        }

        // Set the object alias (name) - only for TTM files, URDF files are already renamed
        if (fileExtension === 'ttm') {
            await this.sim.setObjectAlias(objectHandle, entityName);
        }

        // Set position
        const pos = [position.x, position.y, position.z];
        await this.sim.setObjectPosition(objectHandle, pos);

        // Set orientation (convert from Euler angles to quaternion using deg2quat utility)
        const { qx, qy, qz, qw } = deg2quat(orientation);
        const quaternion = [qx, qy, qz, qw];
        await this.sim.setObjectQuaternion(objectHandle, quaternion);

        console.log(`[${new Date().toLocaleTimeString()}] SPAWN: Entity ${entityName} spawned at position [${pos.join(', ')}] with orientation [${orientation.roll}°, ${orientation.pitch}°, ${orientation.yaw}°] ✓`);
        
        return objectHandle;
    }

    isConnected() {
        return this.connected;
    }

    delay(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
}

// Action handlers setup
function setupActionHandlers(thing, coppeliaSim) {
    // Setup property handlers
    setupPropertyHandlers(thing, coppeliaSim);

    // Set action handlers
    thing.setActionHandler("startSimulation", async () => {
        try {
            await coppeliaSim.startSimulation();
            return "Simulation started successfully";
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] SIM: Start failed - ${error.message}`);
            return `Failed to start simulation: ${error instanceof Error ? error.message : String(error)}`;
        }
    });

    thing.setActionHandler("pauseSimulation", async () => {
        try {
            await coppeliaSim.pauseSimulation();
            return "Simulation paused successfully";
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] SIM: Pause failed - ${error.message}`);
            return `Failed to pause simulation: ${error instanceof Error ? error.message : String(error)}`;
        }
    });

    thing.setActionHandler("stopSimulation", async () => {
        try {
            await coppeliaSim.stopSimulation();
            return "Simulation stopped successfully";
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] SIM: Stop failed - ${error.message}`);
            return `Failed to stop simulation: ${error instanceof Error ? error.message : String(error)}`;
        }
    });

    thing.setActionHandler("loadScene", async (data) => {
        try {
            const fileName = await data.value();
            if (typeof fileName !== 'string') {
                throw new Error('Scene filename must be a string');
            }
            await coppeliaSim.loadScene(fileName);
            return `Scene loaded successfully: ${fileName}`;
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] SCENE: Load failed - ${error.message}`);
            return `Failed to load scene: ${error instanceof Error ? error.message : String(error)}`;
        }
    });

    thing.setActionHandler("saveScene", async (data) => {
        try {
            const filename = await data.value();
            if (typeof filename !== 'string') {
                throw new Error('Filename must be a string');
            }
            // Always save under workspace/saved/world - extract just the filename
            const baseFilename = filename.split('/').pop();
            const fullPath = `${WORKSPACE_DIR}/saved/world/${baseFilename}`;
            await coppeliaSim.saveScene(filename);
            return `Scene saved successfully as: ${fullPath}`;
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] SAVE: Save failed - ${error.message}`);
            return `Failed to save scene: ${error instanceof Error ? error.message : String(error)}`;
        }
    });

    thing.setActionHandler("closeScene", async () => {
        try {
            await coppeliaSim.closeScene();
            return "Scene closed successfully";
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] SCENE: Close failed - ${error.message}`);
            return `Failed to close scene: ${error instanceof Error ? error.message : String(error)}`;
        }
    });

    thing.setActionHandler("spawn_entity", async (data) => {
        try {
            const input = await data.value();
            const { entity_name, file_name, position = {}, orientation = {} } = input;

            if (!entity_name || !file_name) {
                throw new Error('Both entity_name and file_name are required');
            }

            // Check if entity already exists by getting objects list
            const objects = await getCoppeliaSimObjects('localhost', 23050, false, 100);
            const existingEntity = objects.find(obj => obj.name === entity_name);
            if (existingEntity) {
                throw new Error(`Entity "${entity_name}" already exists. Please choose a different name or remove the existing entity first.`);
            }

            // Set default values for position and orientation
            const pos = {
                x: Number.isFinite(position.x) ? position.x : 0,
                y: Number.isFinite(position.y) ? position.y : 0,
                z: Number.isFinite(position.z) ? position.z : 0
            };

            const orient = {
                roll: Number.isFinite(orientation.roll) ? orientation.roll : 0,
                pitch: Number.isFinite(orientation.pitch) ? orientation.pitch : 0,
                yaw: Number.isFinite(orientation.yaw) ? orientation.yaw : 0
            };

            await coppeliaSim.spawnEntity(entity_name, file_name, pos, orient);
            return `Entity "${entity_name}" spawned successfully from "${file_name}" at position [${pos.x}, ${pos.y}, ${pos.z}] with orientation [${orient.roll}°, ${orient.pitch}°, ${orient.yaw}°]`;
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] SPAWN: Spawn failed - ${error.message}`);
            return `Failed to spawn entity: ${error instanceof Error ? error.message : String(error)}`;
        }
    });

    thing.setActionHandler("set_entity_pose", async (data) => {
        try {
            const input = await data.value();
            const { id, name, position = {}, orientation = {} } = input;

            // Either id or name must be provided
            if (!id && !name) {
                throw new Error('Either id or name must be provided to identify the entity');
            }

            // Get all objects once and find the target entity
            const objects = await getCoppeliaSimObjects('localhost', 23050, false, 100);
            let targetObject = null;
            
            if (id !== undefined) {
                // Find by ID (handle)
                targetObject = objects.find(obj => obj.handle === id);
                if (!targetObject) {
                    throw new Error(`Entity with ID ${id} not found`);
                }
            } else {
                // Find by name
                targetObject = objects.find(obj => obj.name === name);
                if (!targetObject) {
                    throw new Error(`Entity with name "${name}" not found`);
                }
            }
            
            const entityHandle = targetObject.handle;
            const entityName = targetObject.name;

            // Set position if provided
            if (position.x !== undefined || position.y !== undefined || position.z !== undefined) {
                const pos = [
                    Number.isFinite(position.x) ? position.x : 0,
                    Number.isFinite(position.y) ? position.y : 0,
                    Number.isFinite(position.z) ? position.z : 0
                ];
                await coppeliaSim.sim.setObjectPosition(entityHandle, pos);
                console.log(`[${new Date().toLocaleTimeString()}] POSE: Set position for "${entityName}" to [${pos.join(', ')}]`);
            }

            // Set orientation if provided
            if (orientation.roll !== undefined || orientation.pitch !== undefined || orientation.yaw !== undefined) {
                const orient = {
                    roll: Number.isFinite(orientation.roll) ? orientation.roll : 0,
                    pitch: Number.isFinite(orientation.pitch) ? orientation.pitch : 0,
                    yaw: Number.isFinite(orientation.yaw) ? orientation.yaw : 0
                };
                
                // Convert from Euler angles to quaternion using deg2quat utility
                const { qx, qy, qz, qw } = deg2quat(orient);
                const quaternion = [qx, qy, qz, qw];
                await coppeliaSim.sim.setObjectQuaternion(entityHandle, quaternion);
                console.log(`[${new Date().toLocaleTimeString()}] POSE: Set orientation for "${entityName}" to [${orient.roll}°, ${orient.pitch}°, ${orient.yaw}°]`);
            }

            return `Entity "${entityName}" (ID: ${entityHandle}) pose updated successfully`;
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] POSE: Set pose failed - ${error.message}`);
            return `Failed to set entity pose: ${error instanceof Error ? error.message : String(error)}`;
        }
    });

    thing.setActionHandler("remove_entity", async (data) => {
        try {
            const input = await data.value();
            const { name } = input;

            if (!name) {
                throw new Error('Entity name is required');
            }

            // Get all objects and find the target entity
            const objects = await getCoppeliaSimObjects('localhost', 23050, false, 100);
            const targetObject = objects.find(obj => obj.name === name);
            
            if (!targetObject) {
                throw new Error(`Entity with name "${name}" not found`);
            }

            const entityHandle = targetObject.handle;
            const entityName = targetObject.name;

            // Remove the entity using CoppeliaSim's removeModel API
            // This removes the entire model hierarchy (root + all children), matching GUI behavior
            try {
                await coppeliaSim.sim.removeModel(entityHandle);
                console.log(`[${new Date().toLocaleTimeString()}] REMOVE: Removed model "${entityName}" (ID: ${entityHandle}) and all its children`);
                return `Model "${entityName}" (ID: ${entityHandle}) and all its children removed successfully`;
            } catch (removeModelError) {
                // If removeModel fails (e.g., object is not a model base), fall back to removeObjects
                console.log(`[${new Date().toLocaleTimeString()}] REMOVE: removeModel failed, trying removeObjects: ${removeModelError.message}`);
                await coppeliaSim.sim.removeObjects([entityHandle]);
                console.log(`[${new Date().toLocaleTimeString()}] REMOVE: Removed object "${entityName}" (ID: ${entityHandle})`);
                return `Object "${entityName}" (ID: ${entityHandle}) removed successfully (note: child objects may remain)`;
            }
        } catch (error) {
            console.error(`[${new Date().toLocaleTimeString()}] REMOVE: Remove failed - ${error.message}`);
            return `Failed to remove entity: ${error instanceof Error ? error.message : String(error)}`;
        }
    });
}

module.exports = {
    CoppeliaSimConnection,
    setupActionHandlers,
    WORKSPACE_DIR
};
