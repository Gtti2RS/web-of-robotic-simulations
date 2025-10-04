const fs = require('fs');
const path = require('path');
const { readModels } = require('./gz_observable_topics');

// Store the extracted world name
let storedWorldName = null;

/**
 * Extract world name from SDF file and store it
 * @param {string} sdfFilePath - Path to the SDF file
 * @returns {Promise<string>} - World name
 */
async function extract_world(sdfFilePath) {
  try {
    console.log(`[${new Date().toISOString()}] [extract_world] Starting extraction from: ${sdfFilePath}`);
    
    if (!sdfFilePath) {
      throw new Error('SDF file path is required');
    }
    
    if (!fs.existsSync(sdfFilePath)) {
      throw new Error(`SDF file not found: ${sdfFilePath}`);
    }

    const content = fs.readFileSync(sdfFilePath, 'utf8');
    
    // Simple regex to extract world name from <world name="...">
    const worldMatch = content.match(/<world\s+name\s*=\s*["']([^"']+)["']/);
    
    if (worldMatch && worldMatch[1]) {
      storedWorldName = worldMatch[1];
      console.log(`[${new Date().toISOString()}] [extract_world] Successfully extracted world name: ${storedWorldName}`);
      return storedWorldName;
    }
    
    throw new Error(`No world name found in SDF file: ${sdfFilePath}`);
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [extract_world] Error: ${error.message}`);
    throw new Error(`Failed to extract world name: ${error.message}`);
  }
}

/**
 * Get the stored world name
 * @returns {string} - Stored world name
 */
function get_world() {
  if (!storedWorldName) {
    throw new Error('No active world detected.');
  }
  return storedWorldName;
}

/**
 * Clear the stored world name
 */
function clear_world() {
  console.log(`[${new Date().toISOString()}] [clear_world] Clearing stored world name`);
  storedWorldName = null;
}

/**
 * Get entities using the model list from gz_topics.js instead of child process
 * @returns {Promise<Array>} - Array of entity objects with name, id, and pose
 */
async function get_entity() {
  try {    
    // Get the current models data
    const modelsData = await readModels();
    
    if (!modelsData || !modelsData.models) {
      console.warn('[get_entity] No models data available');
      return [];
    }
    
    // Transform the models data to match the expected format
    const entities = modelsData.models.map(model => ({
      name: model.name,
      id: model.id,
      pose: {
        position: { x: 0, y: 0, z: 0 }, // Default position since pose data isn't available in models
        orientation: { x: 0, y: 0, z: 0, w: 1 } // Default orientation
      }
    }));
    
    return entities;
  } catch (error) {
    console.error('[get_entity] Error getting entities:', error);
    throw new Error(`Failed to get entities: ${error.message}`);
  }
}

/**
 * Check if an entity exists by ID or name
 * @param {string|number|bigint} idOrName - Entity ID or name to check
 * @returns {Promise<boolean>} - True if entity exists, false otherwise
 */
async function entityExists(idOrName) {
  try {
    const entities = await get_entity();
    if (idOrName == null) return false;

    const isId =
      typeof idOrName === 'bigint' ||
      typeof idOrName === 'number' ||
      (typeof idOrName === 'string' && /^\d+$/.test(idOrName));

    if (isId) {
      const targetId = typeof idOrName === 'bigint' ? idOrName : BigInt(idOrName);
      return entities.some(m => m.id !== undefined && BigInt(m.id) === targetId);
    }

    return entities.some(m => m.name === idOrName);
  } catch (error) {
    console.error('[entityExists] Error checking entity existence:', error);
    return false;
  }
}

/**
 * Extract SDF file path from ROS2 launch file
 * @param {string} launchFilePath - Path to the launch file
 * @returns {Promise<string|null>} - SDF file path if found, null otherwise
 */
async function extractSdfFromLaunch(launchFilePath) {
  try {
    console.log(`[${new Date().toISOString()}] [extractSdfFromLaunch] Starting extraction from: ${launchFilePath}`);
    
    if (!launchFilePath) {
      throw new Error('Launch file path is required');
    }
    
    if (!fs.existsSync(launchFilePath)) {
      throw new Error(`Launch file not found: ${launchFilePath}`);
    }

    const content = fs.readFileSync(launchFilePath, 'utf8');
    
    // Look for gz_args with SDF file references
    // Pattern: 'gz_args': '-r -z 1000000 linear_battery_demo.sdf'
    const gzArgsMatch = content.match(/['"]gz_args['"]\s*:\s*['"]([^'"]+)['"]/);
    
    if (gzArgsMatch && gzArgsMatch[1]) {
      const gzArgs = gzArgsMatch[1];
      console.log(`[${new Date().toISOString()}] [extractSdfFromLaunch] Found gz_args: ${gzArgs}`);
      
      // Extract SDF path from gz_args
      // Look for .sdf files in the arguments (both relative and absolute paths)
      const sdfMatch = gzArgs.match(/([^\s]+\.sdf)/);
      
      if (sdfMatch && sdfMatch[1]) {
        const sdfPath = sdfMatch[1];
        console.log(`[${new Date().toISOString()}] [extractSdfFromLaunch] Found SDF path: ${sdfPath}`);
        
        // Check if it's already an absolute path
        if (path.isAbsolute(sdfPath)) {
          // Verify the file exists
          if (fs.existsSync(sdfPath)) {
            console.log(`[${new Date().toISOString()}] [extractSdfFromLaunch] Using absolute SDF path: ${sdfPath}`);
            return sdfPath;
          } else {
            console.warn(`[${new Date().toISOString()}] [extractSdfFromLaunch] Absolute SDF path does not exist: ${sdfPath}`);
            return null;
          }
        } else {
          // Try to resolve the relative path to the SDF file
          const { resolveFilePath } = require('../common/fileUtils');
          const resolvedSdfPath = await resolveFilePath(sdfPath);
          
          if (resolvedSdfPath) {
            console.log(`[${new Date().toISOString()}] [extractSdfFromLaunch] Resolved SDF path: ${resolvedSdfPath}`);
            return resolvedSdfPath;
          } else {
            console.warn(`[${new Date().toISOString()}] [extractSdfFromLaunch] Could not resolve SDF file: ${sdfPath}`);
            return null;
          }
        }
      }
    }
    
    console.log(`[${new Date().toISOString()}] [extractSdfFromLaunch] No SDF file found in launch file`);
    return null;
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [extractSdfFromLaunch] Error: ${error.message}`);
    return null;
  }
}

/**
 * Check if a world file contains UR10 robot references
 * @param {string} sdfFilePath - Path to the SDF file
 * @returns {Promise<boolean>} - True if UR10 is referenced, false otherwise
 */
async function ur10Exists(sdfFilePath) {
  try {
    console.log(`[${new Date().toISOString()}] [ur10Exists] Checking for UR10 in: ${sdfFilePath}`);
    
    if (!sdfFilePath) {
      throw new Error('SDF file path is required');
    }
    
    if (!fs.existsSync(sdfFilePath)) {
      throw new Error(`SDF file not found: ${sdfFilePath}`);
    }

    const content = fs.readFileSync(sdfFilePath, 'utf8');
    
    // Check for UR10 references in the file
    // Look for URI references to ur10_rg2
    const ur10Patterns = [
      /<uri>.*ur10_rg2.*<\/uri>/i,
      /<uri>.*ur10.*<\/uri>/i,
      /ur10_rg2/i,
      /ur10\.urdf/i
    ];
    
    const hasUr10 = ur10Patterns.some(pattern => pattern.test(content));
    
    console.log(`[${new Date().toISOString()}] [ur10Exists] UR10 found: ${hasUr10}`);
    return hasUr10;
  } catch (error) {
    console.error(`[${new Date().toISOString()}] [ur10Exists] Error: ${error.message}`);
    return false;
  }
}

module.exports = {
  extract_world,
  get_world,
  clear_world,
  get_entity,
  entityExists,
  extractSdfFromLaunch,
  ur10Exists
};
