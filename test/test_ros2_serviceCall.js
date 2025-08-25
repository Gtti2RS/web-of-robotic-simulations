// project_root/test_ros2_serviceCall.js
// Export WoT action handlers as factories that capture your already-created rclnodejs node.

const { callService } = require('../library/common/ros2_service_helper');
const { get_world } = require('../library/gazebo/gz_actions'); 

/**
 * Set real-time factor (RTF) for the active world.
 * Uses minimal payload (real_time_factor) and optionally max_step_size if provided.
 */
function makeSetRtf(node, { debug = false, timeoutMs = 1000 } = {}) {
  return async function setRtf(input) {
    const { rtf, maxStep } = await input.value();     // WoT schema validated
    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    const srvType = 'gz_physics_bridge/srv/SetPhysics';
    const serviceName = `/world/${world}/set_physics`;

    const { req, resp } = await callService(
      node,
      {
        srvType,
        serviceName,
        payload: {
          real_time_factor: Number(rtf),
          max_step_size: Number.isFinite(maxStep) ? Number(maxStep) : undefined, // optional
          // If your .srv has more fields, you can add them here; invalid keys are auto-filtered.
        }
      },
      { debug, timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      const parts = [`RTF=${req.real_time_factor}`, `world=${world}`];
      if ('max_step_size' in req && Number.isFinite(req.max_step_size)) {
        parts.splice(1, 0, `max_step_size=${req.max_step_size}`);
      }
      return parts.join(', ');
    }
    throw new Error(resp?.message ? `SetRtf failed: ${resp.message}` : 'SetRtf failed');
  };
}

module.exports = {
  makeSetRtf,
};
