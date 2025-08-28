// Export WoT action handlers as factories that capture your already-created rclnodejs node.

const { callService } = require('../common/ros2_service_helper');
const { get_world, entityExists } = require('./gz_actions'); 

/**
 * Set real-time factor (RTF) for the active world.
 * Uses minimal payload (real_time_factor) and optionally max_step_size if provided.
 */
function makeSetRtf(node, { timeoutMs = 1000 } = {}) {
  return async function setRtf(input) {
    const { rtf, maxStep } = await input.value();
    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    const { req, resp } = await callService(
      node,
      {
        srvType: 'gz_physics_bridge/srv/SetPhysics',
        serviceName: `/world/${world}/set_physics`,
        payload: {
          real_time_factor: Number(rtf),
          max_step_size: Number.isFinite(maxStep) ? Number(maxStep) : undefined
        }
      },
      { timeoutMs }
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



/**
 * Remove an entity from the active world.
 * Uses minimal payload (name) and optionally typce if provided.
 */
function makeDeleteEntity(node, { timeoutMs = 1000 } = {}) {
  return async function deleteEntityAction(input) {
    const { id, name, type } = (await input.value()) ?? {};
    if (id == null && !name) {
      throw new Error('DeleteEntity requires either "id" or "name".');
    }

    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    // Verify the entity exists before attempting deletion
    if (id != null) {
      const exists = await entityExists(id);
      if (!exists) throw new Error(`Entity id ${id} not found.`);
    } else if (name) {
      const exists = await entityExists(name);
      if (!exists) throw new Error(`Entity ${name} not found.`);
    }

    const entity = {};
    if (id != null) {
      entity.id = (typeof id === 'bigint') ? id : BigInt(id);
    }
    if (name) entity.name = name;
    if (typeof type === 'number') entity.type = type;
    else if (name && id == null) entity.type = 2; // Default MODEL=2 when deleting by name

    const { req, resp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/DeleteEntity',
        serviceName: `/world/${world}/remove`,
        payload: { entity }
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      const identifier = name ? `name=${name}` : `id=${id}`;
      const typeInfo = type !== undefined ? `, type=${type}` : '';
      return `Deleted entity (${identifier}${typeInfo}).`;
    }
    throw new Error(resp?.message ? `DeleteEntity failed: ${resp.message}` : 'DeleteEntity failed');
  };
}



module.exports = {
  makeSetRtf,
  makeDeleteEntity,
};
