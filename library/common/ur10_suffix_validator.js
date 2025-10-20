/**
 * @fileoverview URDF File Validation Utilities
 * 
 * This module provides validation functions for URDF files to ensure
 * they are used with the correct simulator based on their filename suffix.
 * 
 * Key Features:
 * - Validates URDF file suffixes for Gazebo and CoppeliaSim simulators
 * - Provides clear error messages for incorrect file usage
 * - Prevents cross-simulator file usage (e.g., Gazebo URDF in CoppeliaSim)
 * 
 * @author Yifan & Cursor
 * @version 1.0.0
 */

/**
 * Validate URDF file suffix for Gazebo simulator
 * @param {string} fileName - The filename to validate
 * @returns {Object} - { valid: boolean, error?: string }
 */
function validateUrdfSuffixForGazebo(fileName) {
  if (!fileName) {
    return { valid: true }; // Skip validation if no filename
  }
  
  // Check if it's a UR10 URDF file
  if (fileName.includes('ur10_rg2')) {
    if (fileName.includes('_coppelia.urdf')) {
      return {
        valid: false,
        error: `Cannot spawn CoppeliaSim URDF file '${fileName}' in Gazebo simulator. Please use 'ur10_rg2_gazebo.urdf' instead.`
      };
    }
    if (!fileName.includes('_gazebo.urdf')) {
      return {
        valid: false,
        error: `UR10 URDF file '${fileName}' must have '_gazebo' suffix for Gazebo simulator. Please use 'ur10_rg2_gazebo.urdf'.`
      };
    }
  }
  
  return { valid: true };
}

/**
 * Validate URDF file suffix for CoppeliaSim simulator
 * @param {string} fileName - The filename to validate
 * @returns {Object} - { valid: boolean, error?: string }
 */
function validateUrdfSuffixForCoppeliaSim(fileName) {
  if (!fileName) {
    return { valid: true }; // Skip validation if no filename
  }
  
  // Check if it's a UR10 URDF file
  if (fileName.includes('ur10_rg2')) {
    if (fileName.includes('_gazebo.urdf')) {
      return {
        valid: false,
        error: `Cannot spawn Gazebo URDF file '${fileName}' in CoppeliaSim simulator. Please use 'ur10_rg2_coppelia.urdf' instead.`
      };
    }
    if (!fileName.includes('_coppelia.urdf')) {
      return {
        valid: false,
        error: `UR10 URDF file '${fileName}' must have '_coppelia' suffix for CoppeliaSim simulator. Please use 'ur10_rg2_coppelia.urdf'.`
      };
    }
  }
  
  return { valid: true };
}

/**
 * Generic URDF validation function that determines the correct validator based on simulator type
 * @param {string} fileName - The filename to validate
 * @param {string} simulator - The simulator type ('gazebo' or 'coppelia')
 * @returns {Object} - { valid: boolean, error?: string }
 */
function validateUrdfSuffix(fileName, simulator) {
  if (!simulator) {
    return { valid: true }; // Skip validation if no simulator specified
  }
  
  const simulatorLower = simulator.toLowerCase();
  
  if (simulatorLower === 'gazebo') {
    return validateUrdfSuffixForGazebo(fileName);
  } else if (simulatorLower === 'coppelia' || simulatorLower === 'coppeliasim') {
    return validateUrdfSuffixForCoppeliaSim(fileName);
  }
  
  return { valid: true }; // Unknown simulator, skip validation
}

module.exports = {
  validateUrdfSuffixForGazebo,
  validateUrdfSuffixForCoppeliaSim,
  validateUrdfSuffix
};
