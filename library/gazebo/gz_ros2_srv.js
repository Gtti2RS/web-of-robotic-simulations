// Export WoT action handlers as factories that capture your already-created rclnodejs node.

const { callService } = require('../common/ros2_service_helper');
const { resolveFilePath } = require('../common/fileUtils');
const { deg2quat } = require('../common/deg2quat');
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



/**
 * Set entity pose via ROS 2 bridged service (ros_gz_interfaces/SetEntityPose)
 */
function makeSetEntityPose(node, { timeoutMs = 1000 } = {}) {
  return async function setEntityPoseAction(input) {
    const data = await input.value();

    const { id, name } = data ?? {};
    if (id == null && !name) throw new Error('SetEntityPose requires either "id" or "name".');

    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    // Existence check (supports id or name)
    const exists = await entityExists(id != null ? id : name);
    if (!exists) {
      throw new Error(id != null ? `Entity id ${id} not found.` : `Entity ${name} not found.`);
    }

    // Position defaults
    const p = data.position || {};
    const position = {
      x: Number.isFinite(p.x) ? p.x : 0,
      y: Number.isFinite(p.y) ? p.y : 0,
      z: Number.isFinite(p.z) ? p.z : 0
    };

    // Orientation: roll, pitch, yaw (deg) -> quaternion
    const rpy = data.orientation || {};
    const { qx, qy, qz, qw } = deg2quat({
      roll: Number.isFinite(rpy.roll) ? rpy.roll : 0,
      pitch: Number.isFinite(rpy.pitch) ? rpy.pitch : 0,
      yaw: Number.isFinite(rpy.yaw) ? rpy.yaw : 0
    });

    // Build request according to ros_gz_interfaces/srv/SetEntityPose
    const entity = {};
    if (id != null) entity.id = (typeof id === 'bigint') ? id : BigInt(id);
    if (name) entity.name = name;
    if (id == null && name) entity.type = 2; // MODEL when targeting by name

    const payload = {
      entity,
      pose: {
        position,
        orientation: { x: qx, y: qy, z: qz, w: qw }
      }
    };

    const { resp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/SetEntityPose',
        serviceName: `/world/${world}/set_pose`,
        payload
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      const identifier = name ? `name=${name}` : `id=${id}`;
      return `Pose set for entity (${identifier}).`;
    }
    throw new Error(resp?.message ? `SetEntityPose failed: ${resp.message}` : 'SetEntityPose failed');
  };
}

/**
 * Spawn entity via ROS 2 bridged service (ros_gz_interfaces/EntityFactory on /world/<world>/create)
 * Input mirrors existing spawn_entity action: { entity_name, file_name, position, orientation }
 */
function makeSpawnEntity(node, { timeoutMs = 1000 } = {}) {
  return async function spawnEntityAction(input) {
    const data = await input.value();

    const entity_name = data?.entity_name;
    const file_name = data?.file_name;
    if (!entity_name) throw new Error('SpawnEntity requires "entity_name".');
    if (!file_name) throw new Error('SpawnEntity requires "file_name".');

    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    // Avoid name collision
    const exists = await entityExists(entity_name);
    if (exists) return `Entity ${entity_name} already exists.`;

    const fullPath = await resolveFilePath(file_name);
    if (!fullPath) throw new Error(`File not found: ${file_name}`);

    // Position defaults
    const p = data.position || {};
    const position = {
      x: Number.isFinite(p.x) ? p.x : 0,
      y: Number.isFinite(p.y) ? p.y : 0,
      z: Number.isFinite(p.z) ? p.z : 0
    };

    // Orientation: roll/pitch/yaw (deg) -> quaternion
    const rpy = data.orientation || {};
    const { qx, qy, qz, qw } = deg2quat({
      roll: Number.isFinite(rpy.roll) ? rpy.roll : 0,
      pitch: Number.isFinite(rpy.pitch) ? rpy.pitch : 0,
      yaw: Number.isFinite(rpy.yaw) ? rpy.yaw : 0
    });

    // ros_gz_interfaces/srv/SpawnEntity expects an EntityFactory inside `entity_factory`
    const payload = {
      entity_factory: {
        name: entity_name,
        allow_renaming: false,
        sdf: undefined,
        sdf_filename: fullPath,
        clone_name: undefined,
        pose: {
          position,
          orientation: { x: qx, y: qy, z: qz, w: qw }
        },
        relative_to: 'world'
      }
    };

    const { resp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/SpawnEntity',
        serviceName: `/world/${world}/create`,
        payload
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      return `${entity_name} is spawned to current world.`;
    }
    throw new Error(resp?.message ? `SpawnEntity failed: ${resp.message}` : 'SpawnEntity failed');
  };
}

/**
 * Control simulation via ROS 2 bridged service (ros_gz_interfaces/ControlWorld)
 * Input: { mode: 'pause' | 'run' | 'resume' | 'reset' }
 */
function makeSimControl(node, { timeoutMs = 1000 } = {}) {
  return async function simControlAction(input) {
    const { mode } = await input.value();

    const world = await get_world();
    if (!world) throw new Error('Active world not found');

    let world_control;
    switch (mode) {
      case 'pause':
        world_control = { pause: true };
        break;
      case 'run':
      case 'resume':
        world_control = { pause: false };
        break;
      case 'reset':
        world_control = { reset: { all: true }, pause: true };
        break;
      default:
        throw new Error(`Invalid mode: ${mode}`);
    }

    const { resp } = await callService(
      node,
      {
        srvType: 'ros_gz_interfaces/srv/ControlWorld',
        serviceName: `/world/${world}/control`,
        payload: { world_control }
      },
      { timeoutMs }
    );

    if (resp?.success ?? resp?.ok ?? resp?.boolean) {
      return `Simulation "${world}" successfully ${mode}d.`;
    }
    throw new Error(resp?.message ? `SimControl failed: ${resp.message}` : 'SimControl failed');
  };
}

module.exports = {
  makeSetRtf,
  makeDeleteEntity,
  makeSetEntityPose,
  makeSpawnEntity,
  makeSimControl,
};
