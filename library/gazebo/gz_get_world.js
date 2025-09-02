const fs = require('fs');

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

module.exports = {
  extract_world,
  get_world,
  clear_world
};
