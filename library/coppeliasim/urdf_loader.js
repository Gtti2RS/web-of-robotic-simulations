#!/usr/bin/env node

const fs = require('fs');
const path = require('path');
const { RemoteAPIClient } = require('./coppelia_helper');

// Parse command line arguments
function parseArguments() {
    const args = process.argv.slice(2);
    
    if (args.length < 2) {
        console.log('Usage: node urdf_loader.js <urdf_path> <name>');
        console.log('Example: node urdf_loader.js /path/to/robot.urdf my_robot');
        process.exit(1);
    }
    
    const urdfPath = args[0];
    const name = args[1];
    
    return { urdfPath, name };
}

// Core function to load URDF - can be called by other modules
async function loadUrdf(urdf_path, name, sim = null) {
    if (!fs.existsSync(urdf_path)) {
        throw new Error(`URDF file not found: ${urdf_path}`);
    }
    
    const resolvedUrdfPath = path.resolve(urdf_path);
    console.log(`[INFO] Importing ${path.basename(resolvedUrdfPath)} as "${name}"`);
    
    let client = null;
    let shouldCloseClient = false;
    
    try {
        // Use provided sim object or create new connection
        if (sim === null) {
            client = new RemoteAPIClient('localhost', 23050, 'json');
            await client.websocket.open();
            sim = await client.getObject('sim');
            shouldCloseClient = true;
        }
        
        // Direct URDF import with renaming - based on URDF importer logic
        const importScript = `
-- Direct URDF import with custom naming
local sim = require('sim')
local simURDF = require('simURDF')

-- Get snapshot of objects before import
local function snapshotAll()
    local t = {}
    local list = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    for _,h in ipairs(list) do t[h] = true end
    return t
end

-- Take snapshot before import
local before = snapshotAll()

-- Import URDF with default options (same as URDF importer)
local opts = 0  -- Default options
local packageStr = ''  -- Empty package string

local res, err = pcall(function()
    return simURDF.import("${resolvedUrdfPath.replace(/"/g, '\\"')}", opts, packageStr)
end)

if not res then
    sim.addLog(sim.verbosity_errors, 'Import error: ' .. tostring(err))
    return {false, -1}
end

-- Find newly created top-level objects
local after = snapshotAll()
local newObjects = {}
for h,_ in pairs(after) do
    if not before[h] then
        local parent = sim.getObjectParent(h)
        if parent == -1 then  -- Top-level object
            table.insert(newObjects, h)
        end
    end
end

-- Rename the first new top-level object and return its handle
local importedHandle = -1
if #newObjects > 0 then
    importedHandle = newObjects[1]
    local success = pcall(sim.setObjectAlias, importedHandle, "${name.replace(/"/g, '\\"')}")
    if success then
        sim.addLog(sim.verbosity_scriptinfos, 'Renamed imported object to: ${name}')
    else
        sim.addLog(sim.verbosity_scriptinfos, 'Could not rename object, using default name')
    end
end

sim.addLog(sim.verbosity_scriptinfos, 'URDF imported successfully: ${path.basename(resolvedUrdfPath)}')
return {true, importedHandle}
`;
        
        console.log('[INFO] Executing direct URDF import...');
        const result = await sim.executeLuaCode(importScript);
        
        // Parse the result - it could be an array, object, or comma-separated string
        let success, importedHandle;
        
        if (Array.isArray(result)) {
            success = result[0];
            // The handle might be in the last element, or we need to parse it
            const lastElement = result[result.length - 1];
            const lastElementStr = String(lastElement);
            if (lastElementStr.includes(',')) {
                // If the last element contains commas, get the last part
                const parts = lastElementStr.split(',');
                importedHandle = parseInt(parts[parts.length - 1].trim());
            } else {
                importedHandle = parseInt(lastElementStr);
            }
        } else {
            // Handle other cases (object, string, etc.) by converting to string and parsing
            const str = String(result);
            if (str.includes(',')) {
                const parts = str.split(',');
                success = parts[0].trim() === 'true';
                importedHandle = parseInt(parts[parts.length - 1].trim());
            } else {
                success = false;
                importedHandle = -1;
            }
        }
        
        if (success && importedHandle !== -1) {
            console.log(`[INFO] URDF imported successfully with direct method, handle: ${importedHandle}`);
            return importedHandle;
        }
        
        console.log('[ERROR] URDF import failed');
        return -1;
        
    } catch (error) {
        console.error(`[ERROR] ${error.message}`);
        throw error;
    } finally {
        if (client && shouldCloseClient) {
            try {
                await client.websocket.close();
            } catch (error) {
                // Ignore cleanup errors
            }
        }
    }
}

// Main function for command line usage
async function main() {
    const { urdfPath, name } = parseArguments();
    
    try {
        await loadUrdf(urdfPath, name);
    } catch (error) {
        console.error(`[ERROR] ${error.message}`);
        process.exit(1);
    }
}

// Run the main function
if (require.main === module) {
    main().catch((error) => {
        console.error(`[FATAL] ${error.message}`);
        process.exit(1);
    });
}

module.exports = {loadUrdf};
