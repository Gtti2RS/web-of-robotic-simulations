/**
 * @fileoverview File Utilities Library
 * 
 * This module provides utilities for file system operations in the WoS (Web of Simulators) project,
 * including file upload handling, asset management, and path resolution for Gazebo simulations.
 * 
 * Key Features:
 * - Secure file upload handling with validation
 * - Asset discovery and organization (URDF, Gazebo files)
 * - Path resolution for simulation files
 * - File conflict detection and management
 * 
 * @author Yifan & Cursor & ChatGPT
 * @version 1.0.0
 */

const fs = require("fs/promises");
const path = require("path");

const BASE_PATH = "/project-root";
const folderTypes = ["world", "model", "launch"];

// ============================================================================
// PUBLIC API FUNCTIONS
// ============================================================================

/**
 * Read assets from urdf and gazebo categories
 * @returns {Promise<Object>} - Object with urdf and gazebo asset structures
 */
async function readGazeboAssets() {
    try {
        return await readAvailableResources(["urdf", "gazebo"]);
    } catch (error) {
        console.error('[readGazeboAssets] Error reading assets:', error);
        throw new Error(`Failed to read assets: ${error.message}`);
    }
}

/**
 * Read assets from coppeliasim category and merge with vendor assets
 * @returns {Promise<Object>} - Object with coppeliasim asset structure (user + vendor)
 */
async function readCoppeliaSimAssets() {
    try {
        // Read user assets from /Assets/coppeliasim and /Assets/urdf
        const userAssets = await readAvailableResources(["urdf", "coppeliasim"]);
        
        // Try to load vendor assets from vendor_assets.json
        let vendorAssets = {};
        try {
            const vendorJsonPath = path.join(__dirname, '../coppeliasim/vendor_assets.json');
            const vendorData = await fs.readFile(vendorJsonPath, 'utf8');
            const vendorJson = JSON.parse(vendorData);
            
            if (vendorJson.assets) {
                vendorAssets = vendorJson.assets;
            }
        } catch (vendorError) {
            console.warn('[readCoppeliaSimAssets] No vendor assets found or error reading vendor_assets.json:', vendorError.message);
        }
        
        // Merge user and vendor assets, with vendor assets coming after user assets
        const mergedAssets = { ...userAssets };
        
        if (!mergedAssets.coppeliasim) {
            mergedAssets.coppeliasim = {};
        }
        
        // Merge vendor assets for each category (scenes, models, etc.)
        for (const [category, vendorContent] of Object.entries(vendorAssets)) {
            if (!mergedAssets.coppeliasim[category]) {
                // If category doesn't exist in user assets, add it with vendor subdirectory
                mergedAssets.coppeliasim[category] = { vendor: vendorContent };
            } else if (typeof mergedAssets.coppeliasim[category] === 'object' && !Array.isArray(mergedAssets.coppeliasim[category])) {
                // If it's an object (with subdirectories), add vendor as a subdirectory at the end
                mergedAssets.coppeliasim[category].vendor = vendorContent;
            }
        }
        
        return mergedAssets;
    } catch (error) {
        console.error('[readCoppeliaSimAssets] Error reading assets:', error);
        throw new Error(`Failed to read assets: ${error.message}`);
    }
}

/**
 * List available files under specified categories in Assets folder.
 * @param {string[]} categories - Array of category names to search (e.g., ["urdf", "gazebo"])
 * @returns {Object} Object with category structure and files (without extensions)
 */
async function readAvailableResources(categories = []) {
    try {
        const result = {};
        
        for (const category of categories) {
            const categoryPath = path.join(BASE_PATH, "Assets", category);
            result[category] = await buildCategoryStructure(categoryPath);
        }
        
        return result;
    } catch (err) {
        console.error('[availableResources read]', err);
        return `Failed to read available Assets: ${err.message || String(err)}`;
    }
}

// ============================================================================
// ASSET DISCOVERY FUNCTIONS
// ============================================================================

/**
 * Build the structured category with files grouped by immediate parent directory
 * @param {string} dirPath - Current directory path
 * @returns {Object} Structure with files grouped by parent directory
 */
async function buildCategoryStructure(dirPath) {
    try {
        const entries = await fs.readdir(dirPath, { withFileTypes: true });
        const structure = {};
        
        for (const entry of entries) {
            const fullPath = path.join(dirPath, entry.name);
            
            if (entry.isDirectory()) {
                // Recursively build subdirectory structure
                const subStructure = await buildCategoryStructure(fullPath);
                // Include directory even if empty (changed from checking length > 0)
                structure[entry.name] = subStructure;
            } else if (entry.isFile()) {
                // Only include files with specified extensions
                if (hasValidExtension(entry.name)) {
                    // Get the immediate parent directory name
                    const parentDir = path.basename(dirPath);
                    if (!structure[parentDir]) {
                        structure[parentDir] = [];
                    }
                    structure[parentDir].push(entry.name);
                }
            }
        }
        
        // Flatten the structure - if a directory only contains arrays, merge them
        const flattened = {};
        for (const [key, value] of Object.entries(structure)) {
            if (Array.isArray(value)) {
                flattened[key] = value;
            } else if (typeof value === 'object') {
                // Check if all values in this object are arrays
                const allArrays = Object.values(value).every(v => Array.isArray(v));
                if (allArrays && key !== "models" && key !== "worlds" && key !== "launch" && key !== "scenes") {
                    // Merge all arrays into one
                    const mergedArray = [];
                    for (const arr of Object.values(value)) {
                        mergedArray.push(...arr);
                    }
                    flattened[key] = mergedArray;
                } else {
                    // Keep the structure but flatten nested arrays
                    const nestedFlattened = {};
                    for (const [nestedKey, nestedValue] of Object.entries(value)) {
                        if (Array.isArray(nestedValue)) {
                            nestedFlattened[nestedKey] = nestedValue;
                        } else if (typeof nestedValue === 'object') {
                            const nestedAllArrays = Object.values(nestedValue).every(v => Array.isArray(v));
                            if (nestedAllArrays) {
                                const nestedMergedArray = [];
                                for (const arr of Object.values(nestedValue)) {
                                    nestedMergedArray.push(...arr);
                                }
                                nestedFlattened[nestedKey] = nestedMergedArray;
                            } else {
                                nestedFlattened[nestedKey] = nestedValue;
                            }
                        }
                    }
                    flattened[key] = nestedFlattened;
                }
            }
        }
        
        return flattened;
    } catch (err) {
        // Return empty structure if directory can't be read
        return {};
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Check if a file has one of the valid extensions
 * @param {string} fileName - Name of the file
 * @returns {boolean} - True if it has a valid extension
 */
function hasValidExtension(fileName) {
    const validExtensions = ['.urdf', '.sdf', '.py', '.ttt', '.ttm'];
    return validExtensions.some(ext => fileName.endsWith(ext));
}

async function saveFile(filePath, data) {
    try {
        await fs.mkdir(path.dirname(filePath), { recursive: true });
        const buffer = Buffer.from(data, 'utf8');
        await fs.writeFile(filePath, buffer);
        // console.log("✅ File written:", filePath);
        return filePath;
    } catch (err) {
        console.error("❌ Failed to write file:", err.message);
        throw new Error("File save failed");
    }
}

// ============================================================================
// FILE UPLOAD FUNCTIONS
// ============================================================================

/**
 * Check if a file name already exists anywhere in the Assets structure
 * @param {string} fileName - The file name to check (with extension)
 * @returns {Promise<boolean>} - True if conflict exists
 */
async function checkFileConflict(fileName, simulator = 'gazebo') {
    try {
        // Get all existing files based on simulator type
        let assets;
        if (simulator === 'coppeliasim') {
            assets = await readCoppeliaSimAssets();
        } else {
            assets = await readGazeboAssets();
        }
        
        // Convert assets to JSON string and check if fileName exists
        const assetsString = JSON.stringify(assets);
        return assetsString.includes(`"${fileName}"`);
    } catch (error) {
        console.error('Error checking for conflicts:', error);
        return false; // If we can't check, assume no conflict to avoid blocking uploads
    }
}

/**
 * Handle Gazebo model upload with automatic model.config generation
 * @param {Object} input - Input data containing name, content, and target
 * @returns {Promise<string>} - Success message
 */
async function handleGazeboUpload(input) {
    const data = await input.value();
    const { name, content, target } = data;

    if (!name || !content || !target || !folderTypes.includes(target)) {
        throw new Error("Invalid upload parameters");
    }

    // Extract filename without extension for model name
    const fileName = path.parse(name).name;
    const fileExt = path.parse(name).ext;
    
    // Map target types to Assets subdirectories
    let assetsSubDir;
    switch (target) {
        case 'model':
            assetsSubDir = path.join('Assets', 'gazebo', 'models', 'uploaded');
            break;
        case 'world':
            assetsSubDir = path.join('Assets', 'gazebo', 'worlds', 'uploaded');
            break;
        case 'launch':
            assetsSubDir = path.join('Assets', 'gazebo', 'launch', 'uploaded');
            break;
        default:
            throw new Error(`Unknown target type: ${target}`);
    }
    
    let filePath;
    let resultMessage;
    
    if (target === 'model') {
        // For models: create folder structure and model.config
        const modelDir = path.join(BASE_PATH, assetsSubDir, fileName);
        filePath = path.join(modelDir, name);
        
        // Check for conflicts before creating directory
        const hasConflict = await checkFileConflict(name);
        if (hasConflict) {
            throw new Error(`Model '${fileName}' already exists. Please choose a different name.`);
        }
        
        await fs.mkdir(modelDir, { recursive: true });
        
        // Save the uploaded file in the model directory
        await saveFile(filePath, content);
        
        // Generate model.config content
        const modelConfigContent = generateModelConfig(fileName, name, fileExt);
        const configPath = path.join(modelDir, 'model.config');
        await saveFile(configPath, modelConfigContent);
        
        resultMessage = `Gazebo model '${fileName}' uploaded to '${modelDir}' with auto-generated model.config`;
    } else {
        // For world and launch files: save directly without creating subfolders
        filePath = path.join(BASE_PATH, assetsSubDir, name);
        
        // Check for conflicts before saving file
        const hasConflict = await checkFileConflict(name);
        if (hasConflict) {
            throw new Error(`File '${name}' already exists. Please choose a different name.`);
        }
        
        await saveFile(filePath, content);
        
        resultMessage = `Gazebo ${target} '${name}' uploaded to '${filePath}'`;
    }
    
    return resultMessage;
}

/**
 * Generate model.config XML content for Gazebo models
 * @param {string} modelName - Name of the model
 * @param {string} fileName - Original filename
 * @param {string} fileExt - File extension
 * @returns {string} - XML content for model.config
 */
function generateModelConfig(modelName, fileName, fileExt) {
    const sdfVersion = fileExt === '.sdf' ? '1.7' : '1.6';
    const description = `Auto-generated Gazebo model: ${modelName}`;
    
    return `<?xml version="1.0"?>
<model>
  <name>${modelName}</name>
  <version>1.0.0</version>
  <sdf version="${sdfVersion}">${fileName}</sdf>
  <author><name>Auto-generated</name></author>
  <description>${description}</description>
</model>`;
}

// ============================================================================
// FILE RESOLUTION FUNCTIONS
// ============================================================================

async function resolveFilePath(fileName) {
    // First, search in user Assets directory
    const foundPath = await searchRecursively(path.join(BASE_PATH, "Assets"), fileName);
    
    if (foundPath) {
        return foundPath;
    }
    
    // If not found in user assets, search in vendor assets
    try {
        const vendorPath = await searchVendorAssets(fileName);
        if (vendorPath) {
            return vendorPath;
        }
    } catch (error) {
        console.warn("Error searching vendor assets:", error.message);
    }
    
    console.warn("File NOT FOUND:", fileName);
    return null;
}

async function searchRecursively(dirPath, fileName) {
    try {
        const entries = await fs.readdir(dirPath, { withFileTypes: true });
        
        for (const entry of entries) {
            const fullPath = path.join(dirPath, entry.name);
            
            if (entry.isFile() && entry.name === fileName) {
                return fullPath;
            } else if (entry.isDirectory()) {
                // Recursively search subdirectories
                const foundPath = await searchRecursively(fullPath, fileName);
                if (foundPath) {
                    return foundPath;
                }
            }
        }
    } catch (err) {
        // Silently skip directories that can't be read
    }
    
    return null;
}

/**
 * Search for a file in vendor assets JSON
 * @param {string} fileName - File name to search for
 * @returns {Promise<string|null>} - Full path to vendor file or null
 */
async function searchVendorAssets(fileName) {
    try {
        const vendorJsonPath = path.join(__dirname, '../coppeliasim/vendor_assets.json');
        const vendorData = await fs.readFile(vendorJsonPath, 'utf8');
        const vendorJson = JSON.parse(vendorData);
        
        if (!vendorJson.assets || !vendorJson.metadata?.vendorRoot) {
            return null;
        }
        
        const vendorRoot = vendorJson.metadata.vendorRoot;
        
        // Recursively search through the vendor assets structure
        function searchInStructure(structure, currentPath) {
            for (const [key, value] of Object.entries(structure)) {
                if (Array.isArray(value)) {
                    // Check if fileName is in this array
                    if (value.includes(fileName)) {
                        // Key is the subdirectory name, add it to path before filename
                        return path.join(currentPath, key, fileName);
                    }
                } else if (typeof value === 'object') {
                    // Recursively search in nested structure
                    const found = searchInStructure(value, path.join(currentPath, key));
                    if (found) {
                        return found;
                    }
                }
            }
            return null;
        }
        
        // Search in each category (scenes, models, etc.)
        for (const [category, content] of Object.entries(vendorJson.assets)) {
            const categoryPath = path.join(vendorRoot, category);
            const found = searchInStructure(content, categoryPath);
            if (found) {
                return found;
            }
        }
        
        return null;
    } catch (error) {
        console.warn('[searchVendorAssets] Error:', error.message);
        return null;
    }
}

module.exports = {
    readGazeboAssets,
    readCoppeliaSimAssets,
    saveFile,
    handleGazeboUpload,
    resolveFilePath,
    checkFileConflict
};
