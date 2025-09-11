// Workspace directory
const WORKSPACE_DIR = '/home/evan/wos';

// Use CoppeliaSim helper to load RemoteAPIClient from installation directory
const { RemoteAPIClient } = require('./coppelia_helper');
const { resolveFilePath } = require('../common/fileUtils');

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

    isConnected() {
        return this.connected;
    }

    delay(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
}

// Action handlers setup
function setupActionHandlers(thing, coppeliaSim) {
    // Set property handlers
    thing.setPropertyReadHandler("sim_stats", async () => {
        try {
            const state = await coppeliaSim.getSimulationState();
            // Map CoppeliaSim states to string values for user understanding
            switch (state) {
                case 0: // stopped
                    return "stopped";
                case 8: // paused
                    return "paused";
                case 17: // running
                    return "running";
                default:
                    return `unknown(${state})`; // return unknown state with original value
            }
        } catch (error) {
            console.error('Error getting simulation state:', error);
            return "error";
        }
    });

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
}

module.exports = {
    CoppeliaSimConnection,
    setupActionHandlers,
    WORKSPACE_DIR
};
