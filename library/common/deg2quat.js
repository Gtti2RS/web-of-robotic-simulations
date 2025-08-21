// deg2quat.js
// Convert orientation {roll, pitch, yaw} in degrees → quaternion {w, x, y, z}

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
