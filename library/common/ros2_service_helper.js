// project_root/library/ros2_call.js
// Ubuntu 24.04.2 LTS • ROS 2 Jazzy • Gazebo Harmonic
// Generic ROS 2 service caller for rclnodejs using the callback-style API.
// - Reuses clients per (srvType, serviceName)
// - Filters payload keys to only those present on the generated Request
// - Returns { req, resp } so callers can log if needed

const rcl = require('rclnodejs');

const _clients = new Map(); // key = `${srvType}|${serviceName}`
const _actionClients = new Map(); // key = `${actionType}|${actionName}`

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

function _actionKey(actionType, actionName) {
  return `${actionType}|${actionName}`;
}

function getActionClient(node, actionType, actionName) {
  const k = _actionKey(actionType, actionName);
  if (_actionClients.has(k)) return _actionClients.get(k);
  // Use ActionClient with action type string (supported by rclnodejs)
  const client = new rcl.ActionClient(node, actionType, actionName);
  _actionClients.set(k, client);
  return client;
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

/**
 * callAction(node, { actionType, actionName, goal }, opts)
 * opts: { timeoutMs=60000, debug=false, collectFeedback=false }
 * Returns { goal, result, feedback }
 */
async function callAction(
  node,
  { actionType, actionName, goal = {} },
  opts = {}
) {
  const { timeoutMs = 60000, debug = false, collectFeedback = true, onFeedback } = opts;

  // Build a plain JS goal object; rclnodejs will map fields during send
  const goalMsg = {};
  for (const [k, v] of Object.entries(goal || {})) {
    if (v !== undefined) goalMsg[k] = v;
  }

  if (debug) {
    const snap = {};
    for (const k of Object.keys(goal)) if (k in goalMsg) snap[k] = goalMsg[k];
    console.log('[ros2_action] →', { actionType, actionName, goal: snap });
  }

  const client = getActionClient(node, actionType, actionName);
  const ok = await client.waitForServer(timeoutMs);
  if (!ok) throw new Error(`Action server not available: ${actionName}`);

  const feedback = [];
  const feedbackCb = collectFeedback ? (fb) => { feedback.push(fb); if (typeof onFeedback === 'function') { try { onFeedback(fb); } catch (_) {} } } : undefined;

  const goalHandle = await client.sendGoal(goalMsg, feedbackCb);
  if (!goalHandle) throw new Error('Failed to send goal');

  const result = await goalHandle.getResult();

  if (debug) console.log('[ros2_action] ← result =', result);
  return { goal: goalMsg, result, feedback: collectFeedback ? feedback : undefined };
}

module.exports.callAction = callAction;
module.exports.getActionClient = getActionClient;
