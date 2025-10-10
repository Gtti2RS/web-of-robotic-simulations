// CoppeliaSim Properties Handler
// Handles WoT property read operations for CoppeliaSim controller

const { getCoppeliaSimObjects } = require('./get_object_list');
const { readAvailableResources } = require('../common/fileUtils');

/**
 * Setup property handlers for CoppeliaSim WoT server
 * @param {Object} thing - WoT Thing object
 * @param {Object} coppeliaSim - CoppeliaSim connection instance
 */
function setupPropertyHandlers(thing, coppeliaSim) {
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

    thing.setPropertyReadHandler("availableResources", readAvailableResources);

    // Set models property handler
    thing.setPropertyReadHandler("models", async () => {
        try {
            const objects = await getCoppeliaSimObjects('localhost', 23050, false, 100);
            const timestamp = new Date().toISOString();
            
            // Format models array with handle instead of id
            const models = objects.map(obj => ({
                name: obj.name,
                handle: obj.handle
            }));
            
            return {
                timestamp: timestamp,
                models: models,
                total_count: models.length
            };
        } catch (error) {
            console.error('Error getting models:', error);
            return {
                timestamp: new Date().toISOString(),
                models: [],
                total_count: 0,
                error: error.message
            };
        }
    });
}

module.exports = {
    setupPropertyHandlers
};
