const { Servient } = require('@node-wot/core');
const { HttpServer } = require('@node-wot/binding-http');
const { CoppeliaSimConnection, setupActionHandlers } = require('../library/coppeliasim/cs_actions');
const fs = require('fs');

// Load Thing Description from JSON file
const simulationThingDescription = JSON.parse(fs.readFileSync('./cs_controller.json', 'utf8'));

// Main server function
async function startServer() {
    console.log(`[${new Date().toLocaleTimeString()}] Starting CoppeliaSim WoT Server...`);

    // Create CoppeliaSim connection
    const coppeliaSim = new CoppeliaSimConnection('localhost', 23050);

    // Create WoT servient
    const server = new Servient();
    server.addServer(
        new HttpServer({
            port: 8085,
        })
    );

    try {
        // Start the servient
        const WoT = await server.start();
        console.log(`[${new Date().toLocaleTimeString()}] WoT Servient started`);

        // Connect to CoppeliaSim
        await coppeliaSim.connect();

        // Produce the thing
        const thing = await WoT.produce(simulationThingDescription);

        // Setup action handlers
        setupActionHandlers(thing, coppeliaSim);

        // Expose the thing
        await thing.expose();
        console.log(`[${new Date().toLocaleTimeString()}] Server running on http://localhost:8085/cs_controller`);

        // Graceful shutdown
        process.on('SIGINT', async () => {
            console.log(`[${new Date().toLocaleTimeString()}] Shutting down server...`);
            await coppeliaSim.disconnect();
            process.exit(0);
        });

    } catch (error) {
        console.error(`[${new Date().toLocaleTimeString()}] Failed to start server:`, error);
        process.exit(1);
    }
}

// Start the server
startServer().catch(console.error);
