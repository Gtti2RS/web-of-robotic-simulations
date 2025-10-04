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
                if (Object.keys(subStructure).length > 0) {
                    structure[entry.name] = subStructure;
                }
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
                if (allArrays && key !== "models" && key !== "worlds" && key !== "launch") {
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
async function checkFileConflict(fileName) {
    try {
        // Get all existing files using readGazeboAssets
        const assets = await readGazeboAssets();
        
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
    const foundPath = await searchRecursively(path.join(BASE_PATH, "Assets"), fileName);
    
    if (foundPath) {
        return foundPath;
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

module.exports = {
    readGazeboAssets,
    saveFile,
    handleGazeboUpload,
    resolveFilePath,
    checkFileConflict
};
