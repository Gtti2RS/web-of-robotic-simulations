// project_root/library/ros2_call.js
// Ubuntu 24.04.2 LTS • ROS 2 Jazzy • Gazebo Harmonic
// Generic ROS 2 service caller for rclnodejs using the callback-style API.
// - Reuses clients per (srvType, serviceName)
// - Filters payload keys to only those present on the generated Request
// - Returns { req, resp } so callers can log if needed

const rcl = require('rclnodejs');

const _clients = new Map(); // key = `${srvType}|${serviceName}`

function _key(srvType, serviceName) {
  return `${srvType}|${serviceName}`;
}

function getClient(node, srvType, serviceName) {
  const k = _key(srvType, serviceName);
  if (_clients.has(k)) return _clients.get(k);
  const c = node.createClient(srvType, serviceName);
  _clients.set(k, c);
  return c;
}

/**
 * callService(node, { srvType, serviceName, payload }, opts)
 * opts: { timeoutMs=1000, debug=false }
 */
async function callService(
  node,
  { srvType, serviceName, payload = {} },
  opts = {}
) {
  const { timeoutMs = 1000, debug = false } = opts;

  // Load generated type and build a request with only valid fields
  const Srv = rcl.require(srvType);
  const req = new Srv.Request();
  for (const [k, v] of Object.entries(payload)) {
    if (v !== undefined && (k in req)) req[k] = v;
  }

  if (debug) {
    const snap = {};
    for (const k of Object.keys(payload)) if (k in req) snap[k] = req[k];
    console.log('[ros2_call] →', { srvType, serviceName, req: snap });
  }

  const client = getClient(node, srvType, serviceName);
  const ok = await client.waitForService(timeoutMs);
  if (!ok) throw new Error(`Service not available: ${serviceName}`);

  const resp = await new Promise((resolve, reject) => {
    try { client.sendRequest(req, resolve); } catch (e) { reject(e); }
  });

  if (debug) console.log('[ros2_call] ← resp =', resp);
  return { req, resp };
}

module.exports = { callService, getClient };
