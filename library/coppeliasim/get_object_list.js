#!/usr/bin/env node

/**
 * CoppeliaSim Object List Utility
 * Gets object list from CoppeliaSim using WebSocket Remote API
 * Deduplicates objects by name, keeping the one with smallest handle number
 * 
 * @param {number} totalHandles - Maximum handle number to check (default: 50)
 *                               This limits the number of objects that can be found.
 *                               For scenes with many objects, increase this value.
 *                               Note: Higher values will take longer to process.
 * 
 * Usage:
 *   node get_object_list.js [options]
 * 
 * Options:
 *   --debug, -d          Enable debug mode (shows all objects and their types)
 *   --handles N, -h N    Set maximum handle number to check (default: 50)
 * 
 * Examples:
 *   node get_object_list.js                    # Default: 50 handles, no debug
 *   node get_object_list.js --debug            # Debug mode with 50 handles
 *   node get_object_list.js --handles 100      # Check up to 100 handles
 *   node get_object_list.js --debug --handles 200  # Debug mode with 200 handles
 */

const { RemoteAPIClient } = require('./coppelia_helper');

class CoppeliaSimObjectTester {
    constructor(host = 'localhost', port = 23050, debug = false, totalHandles = 50) {
        this.client = new RemoteAPIClient(host, port, 'json');
        this.debug = debug;
        this.totalHandles = totalHandles;
        this.sim = null;
        this.connected = false;
    }

    async connect() {
        try {
            await this.client.websocket.open();
            this.sim = await this.client.getObject('sim');
            this.connected = true;
        } catch (error) {
            this.connected = false;
            throw error;
        }
    }

    async disconnect() {
        if (this.connected && this.client.websocket) {
            try {
                await this.client.websocket.close();
                this.connected = false;
            } catch (error) {
                // Ignore disconnect errors
            }
        }
    }

    async getShapeObjects() {
        if (!this.connected) {
            throw new Error('Not connected to CoppeliaSim');
        }

        const foundObjects = [];
        const allObjects = []; // For debug output
        const batchSize = 10;
        const totalHandles = this.totalHandles;
        
        for (let start = 1; start <= totalHandles; start += batchSize) {
            const end = Math.min(start + batchSize - 1, totalHandles);
            const batch = [];
            
            // Create batch of promises
            for (let handle = start; handle <= end; handle++) {
                batch.push(this.processHandle(handle));
            }
            
            // Wait for batch to complete
            const batchResults = await Promise.all(batch);
            
            // Process results
            batchResults.forEach(result => {
                if (result) {
                    if (this.debug) {
                        allObjects.push(result);
                    }
                    
                    // Filter out common objects: Floor, box (floor link), and generic link objects
                    const excludeObjects = ['Floor', 'box'];
                    if (!result.name.includes('_visible') && !result.name.includes('_visual') && !excludeObjects.includes(result.name)) {
                        if (result.type == 0) { // 0 = shape (use == for type coercion)
                            foundObjects.push(result);
                        }
                    }
                }
            });
        }
        
        // Debug output: show all objects and their types
        if (this.debug) {
            console.log('DEBUG: ALL OBJECTS FOUND');
            allObjects.forEach(obj => {
                console.log(`Handle: ${obj.handle} | Name: ${obj.name} | Type: ${obj.type}`);
            });
        }
        
        // Deduplicate objects by name, keeping the one with smallest handle number
        const uniqueObjects = this.deduplicateObjects(foundObjects);
        
        // Return the deduplicated objects
        return uniqueObjects.map(obj => ({
            handle: obj.handle,
            name: obj.name,
            type: 'shape',
            position: null,
            orientation: null
        }));
    }
    
    deduplicateObjects(objects) {
        const nameMap = new Map();
        
        // Group objects by name
        objects.forEach(obj => {
            const existing = nameMap.get(obj.name);
            if (!existing || obj.handle < existing.handle) {
                // Keep the object with smaller handle number
                nameMap.set(obj.name, obj);
            }
        });
        
        // Convert back to array and sort by handle number
        return Array.from(nameMap.values()).sort((a, b) => a.handle - b.handle);
    }
    
    async processHandle(handle) {
        try {
            const [name, type] = await Promise.all([
                this.sim.getObjectAlias(handle),
                this.sim.getObjectType(handle)
            ]);
            
            const nameStr = Array.isArray(name) ? name[0] : name;
            
            if (nameStr && nameStr !== '') {
                return {
                    handle: handle,
                    name: nameStr,
                    type: type
                };
            }
            return null;
        } catch (e) {
            return null;
        }
    }
}

// Main test function
async function runTest(debug = false, totalHandles = 50) {
    const tester = new CoppeliaSimObjectTester('localhost', 23050, debug, totalHandles);
    
    try {
        await tester.connect();
        const objects = await tester.getShapeObjects();
        
        console.log(`Found ${objects.length} shape objects`);
        console.log(JSON.stringify(objects.map(obj => ({ handle: obj.handle, name: obj.name })), null, 2));
        
    } catch (error) {
        console.error(`Error: ${error.message}`);
        process.exit(1);
    } finally {
        await tester.disconnect();
    }
}

// Run the test if this file is executed directly
if (require.main === module) {
    const debug = process.argv.includes('--debug') || process.argv.includes('-d');
    
    let totalHandles = 50; // default
    const handlesIndex = process.argv.findIndex(arg => arg === '--handles' || arg === '-h');
    if (handlesIndex !== -1 && process.argv[handlesIndex + 1]) {
        const handlesValue = parseInt(process.argv[handlesIndex + 1]);
        if (!isNaN(handlesValue) && handlesValue > 0) {
            totalHandles = handlesValue;
        }
    }
    
    runTest(debug, totalHandles).catch((error) => {
        console.error(`Fatal error: ${error.message}`);
        process.exit(1);
    });
}

// Simple function to get shape objects programmatically
async function getShapeObjects(host = 'localhost', port = 23050, debug = false, totalHandles = 50) {
    const tester = new CoppeliaSimObjectTester(host, port, debug, totalHandles);
    
    try {
        await tester.connect();
        const objects = await tester.getShapeObjects();
        return objects;
    } finally {
        await tester.disconnect();
    }
}

module.exports = {
    CoppeliaSimObjectTester,
    runTest,
    getShapeObjects
};