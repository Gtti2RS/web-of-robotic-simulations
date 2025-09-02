const fs = require('fs');
const { readModels } = require('./gz_topics');

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
    throw new Error('No world name stored. Call extract_world() first.');
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

module.exports = {
  extract_world,
  get_world,
  clear_world,
  get_entity,
  entityExists
};
