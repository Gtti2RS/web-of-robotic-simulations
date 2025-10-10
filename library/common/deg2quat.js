/**
 * @fileoverview Degree to Quaternion Conversion Library
 * 
 * This module provides utilities for converting Euler angles to quaternions,
 * commonly used in robotics and 3D simulations for representing orientations.
 * 
 * Key Features:
 * - Convert Euler angles (roll, pitch, yaw) in degrees to quaternions
 * - ZYX rotation order (yaw → pitch → roll)
 * - Handles missing or undefined angle values with defaults
 * 
 * @author Yifan & Cursor & ChatGPT
 * @version 1.0.0
 */

/**
 * Convert degrees to radians
 * 
 * @param {number} deg - Angle in degrees
 * @returns {number} Angle in radians
 */
function deg2rad(deg) {
  return deg * Math.PI / 180.0;
}

/**
 * Convert Euler angles (ZYX: yaw → pitch → roll) in degrees
 * given as an object {roll, pitch, yaw}
 * @param {{roll:number, pitch:number, yaw:number}} orientation
 * @returns {{w:number, x:number, y:number, z:number}}
 */
function deg2quat(orientation) {
  const roll  = deg2rad(orientation.roll  || 0);
  const pitch = deg2rad(orientation.pitch || 0);
  const yaw   = deg2rad(orientation.yaw   || 0);

  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);

  return {
    qx: sr * cp * cy - cr * sp *sy,
    qy: cr * sp * cy + sr * cp *sy,
    qz: cr * cp * sy - sr * sp *cy,
    qw: cr * cp * cy + sr * sp * sy
  };
}

module.exports = {
  deg2quat
};
