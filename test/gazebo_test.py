#!/usr/bin/env python3
"""
Gazebo Controller WoT Operations Latency Test Script
Tests various WoT operations on gz_controller (port 8080)
Measures response times and exports data to CSV

Target: http://localhost:8080/gz_controller
UR10 Server: http://localhost:8083/ur10_server (when UR10 loaded)

Default All-Enabled Test Sequence (7 tests, ~145 operations):
┌─────────────────────────────────────────────────────────────────────────┐
│ 1. SimStats_Read          - Read simStats 10x with TD baseline (2s)    │
│ 2. Visualize_Toggle       - Toggle visualize 20x, set to true (2s)     │
│ 3. SimControl_RunPause    - Toggle run/pause 20x (2s)                  │
│ 4. SimControl_Speed       - Toggle faster/slower 20x (2s)              │
│ 5. Box_SpawnDelete        - Spawn-Delete box.urdf 10 cycles (2s+2s)    │
│ 6. Box_SetPose            - Load box, set 10 poses, delete (2s)        │
│ 7. UR10_Full_Test         - Load UR10, jointPos 10x (UR10 TD baseline),│
│                             moveJoint 10x, gripper 5x, moveCart 10x    │
│                             (UR10 spawn ~23s, reads 1s, actions 2-5s)  │
└─────────────────────────────────────────────────────────────────────────┘

Baseline Normalization:
- TD baseline read before EACH operation (gz_controller TD for main ops, UR10 TD for UR10 ops)
- Normalized latency = Operation latency - TD baseline latency
- Shows pure operation overhead without network/server baseline cost

Operation Breakdown:
- SimStats Reads: 10 ops + 10 TD baselines (~2ms + ~4ms baseline)
- Visualize Toggles: 20 ops (~300-700ms avg)
- SimControl Run/Pause: 20 ops (~100ms avg)
- SimControl Speed: 20 ops (~100ms avg)
- Box Spawn-Delete: 20 ops (~100ms spawn, ~10ms delete)
- Box SetPose: 10 ops (~100ms avg)
- UR10 Full Test: 45 ops (jointPos:10 + 10 UR10 TD baselines, moveJoint:10, gripper:5, moveCart:10)

Total Estimated Time: ~5-7 minutes
CSV Export: wot_latency_YYYYMMDD_HHMMSS.csv
  Columns: ..., Baseline_Latency_ms, Response_Time_ms, Normalized_Latency_ms, ...

Configuration:
- Edit TEST_SUITE to enable/disable individual tests
- Reorder tests by moving items in the TEST_SUITE list
- Adjust ITERATIONS, TOGGLE_ITERATIONS, INTERVAL_SEC as needed
- Set with_baseline=False in run_test() to disable baseline measurement
"""

import http.client
import json
import time
import csv
import os
from datetime import datetime

# Configuration
HOST = 'localhost'
PORT = 8080
SIMSTATS_PATH = '/gz_controller/properties/simStats'
TD_PATH = '/gz_controller'
VISUALIZE_PATH = '/gz_controller/properties/visualize'
SIMCONTROL_PATH = '/gz_controller/actions/simControl'
MANAGEMODEL_PATH = '/gz_controller/actions/manageModel'
UR10_HOST = 'localhost'
UR10_PORT = 8083
UR10_JOINTPOSITIONS_PATH = '/ur10_server/properties/jointPositions'
ITERATIONS = 10
TOGGLE_ITERATIONS = 20
INTERVAL_SEC = 2.0
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))


def wot_request(path, method='GET', value=None, timeout=10, host=None, port=None):
    """
    Generic WoT request function with response time measurement
    
    Args:
        path: Endpoint path
        method: HTTP method ('GET', 'PUT', or 'POST')
        value: Value to write (for PUT/POST requests)
        timeout: Request timeout in seconds
        host: Override HOST (default: None uses global HOST)
        port: Override PORT (default: None uses global PORT)
    
    Returns:
        dict: Result with success, response_time, status_code, and optional data/error
    """
    start_time = time.time()
    
    target_host = host if host is not None else HOST
    target_port = port if port is not None else PORT
    
    try:
        conn = http.client.HTTPConnection(target_host, target_port, timeout=timeout)
        
        if method == 'GET':
            headers = {'Accept': 'application/json'}
            conn.request('GET', path, headers=headers)
        elif method in ['PUT', 'POST']:
            headers = {'Content-Type': 'application/json'}
            body = json.dumps(value) if value is not None else ''
            conn.request(method, path, body=body, headers=headers)
        else:
            raise ValueError(f'Unsupported HTTP method: {method}')
        
        response = conn.getresponse()
        data = response.read().decode('utf-8')
        conn.close()
        
        response_time = int((time.time() - start_time) * 1000)
        
        # Build result - initially check HTTP status
        result = {
            'success': response.status in [200, 204],
                'response_time': response_time,
                'status_code': response.status
            }
        
        # Parse JSON data for GET requests or responses with content
        if data:
            try:
                json_data = json.loads(data)
                result['data'] = json_data
                
                # For POST requests (actions), check the 'success' field in response
                if method == 'POST' and isinstance(json_data, dict):
                    if 'success' in json_data:
                        result['success'] = json_data['success']
                    if 'message' in json_data:
                        result['response_message'] = json_data['message']
                # If response is a JSON string (not dict), keep HTTP status as success indicator
                elif method == 'POST' and isinstance(json_data, str):
                    result['response_message'] = json_data
                    
            except json.JSONDecodeError:
                if method == 'GET':
                    pass
        
        # Include value for PUT/POST requests
        if method in ['PUT', 'POST']:
            result['value'] = value
        
        return result
            
    except Exception as e:
        result = {
            'success': False,
            'response_time': int((time.time() - start_time) * 1000),
            'error': str(e)
        }
        if method in ['PUT', 'POST']:
            result['value'] = value
        return result


def run_test(path, name, iterations=ITERATIONS, with_baseline=True):
    """Run test iterations for reading an endpoint with TD baseline"""
    print(f'\n{"="*80}')
    print(f'Testing {name}: {iterations} iterations with {int(INTERVAL_SEC*1000)}ms interval')
    print(f'Endpoint: http://{HOST}:{PORT}{path}')
    if with_baseline:
        print(f'Baseline: Reading TD before each operation')
    print(f'{"="*80}\n')
    
    results = []
    
    for i in range(1, iterations + 1):
        timestamp = datetime.now().isoformat()
        
        # Read TD as baseline before each operation
        baseline_latency = 0
        if with_baseline:
            print(f'[{i}/{iterations}] TD baseline ', end='', flush=True)
            baseline_result = wot_request(TD_PATH, method='GET', timeout=5)
            baseline_latency = baseline_result['response_time']
            if baseline_result['success']:
                print(f"({baseline_latency}ms) → ", end='', flush=True)
            else:
                print(f"(✗ {baseline_latency}ms) → ", end='', flush=True)
        else:
            print(f'[{i}/{iterations}] ', end='', flush=True)
        
        # Actual operation
        result = wot_request(path, method='GET', timeout=5)
        result['iteration'] = i
        result['timestamp'] = timestamp
        result['endpoint'] = name
        result['operation'] = 'read'
        result['baseline_latency'] = baseline_latency
        result['normalized_latency'] = result['response_time'] - baseline_latency
        results.append(result)
        
        if result['success']:
            normalized = result['normalized_latency']
            print(f"✓ {result['response_time']}ms (normalized: {normalized:+d}ms)")
        else:
            print(f"✗ {result['response_time']}ms - {result.get('error', 'Unknown error')}")
        
        if i < iterations:
            time.sleep(INTERVAL_SEC)
    
    return results


def run_toggle_test(path, name, iterations, start_value=True):
    """Run test iterations for toggling a property"""
    print(f'\n{"="*80}')
    print(f'Testing {name}: {iterations} toggles with {int(INTERVAL_SEC*1000)}ms interval')
    print(f'Endpoint: http://{HOST}:{PORT}{path}')
    print(f'Starting value: {start_value}')
    print(f'{"="*80}\n')

    results = []
    current_value = start_value

    for i in range(1, iterations + 1):
        timestamp = datetime.now().isoformat()
        print(f'[{i}/{iterations}] {name}={current_value} ', end='', flush=True)

        result = wot_request(path, method='PUT', value=current_value, timeout=10)
        result['iteration'] = i
        result['timestamp'] = timestamp
        result['endpoint'] = name
        result['operation'] = 'write'
        results.append(result)

        if result['success']:
            print(f"✓ {result['response_time']}ms")
        else:
            print(f"✗ {result['response_time']}ms - {result.get('error', 'Unknown error')}")
        
        # Toggle value for next iteration
        current_value = not current_value
        
        if i < iterations:
            time.sleep(INTERVAL_SEC)
    
    return results


def run_visualize_toggle_test():
    """Run visualize toggle test and ensure it ends with visualize=true"""
    results = run_toggle_test(VISUALIZE_PATH, 'visualize', TOGGLE_ITERATIONS, start_value=True)
    
    # After toggle test, set visualize to true
    print(f'\n[Post-test] Setting visualize to true... ', end='', flush=True)
    final_result = wot_request(VISUALIZE_PATH, method='PUT', value=True, timeout=10)
    
    if final_result['success']:
        print(f"✓ {final_result['response_time']}ms")
    else:
        print(f"✗ {final_result['response_time']}ms - {final_result.get('error', 'Unknown error')}")
    
    return results


def run_action_toggle_test(path, name, iterations, mode_a, mode_b):
    """Run test iterations for toggling between two action modes"""
    print(f'\n{"="*80}')
    print(f'Testing {name}: {iterations} toggles with {int(INTERVAL_SEC*1000)}ms interval')
    print(f'Endpoint: http://{HOST}:{PORT}{path}')
    print(f'Modes: {mode_a} <-> {mode_b}')
    print(f'{"="*80}\n')
    
    results = []
    modes = [mode_a, mode_b]
    
    for i in range(1, iterations + 1):
        timestamp = datetime.now().isoformat()
        current_mode = modes[i % 2]
        print(f'[{i}/{iterations}] {name}={current_mode} ', end='', flush=True)
        
        result = wot_request(path, method='POST', value={'mode': current_mode}, timeout=10)
        result['iteration'] = i
        result['timestamp'] = timestamp
        result['endpoint'] = name
        result['operation'] = 'action'
        result['mode'] = current_mode
        results.append(result)
        
        if result['success']:
            print(f"✓ {result['response_time']}ms")
        else:
            print(f"✗ {result['response_time']}ms - {result.get('error', 'Unknown error')}")
        
        if i < iterations:
            time.sleep(INTERVAL_SEC)
    
    return results


def run_setpose_test(path, name, iterations, model_file, model_name='test_setpose'):
    """Run test iterations for setting different poses on a model"""
    print(f'\n{"="*80}')
    print(f'Testing {name}: Load model, set {iterations} poses with {int(INTERVAL_SEC*1000)}ms interval, then delete')
    print(f'Endpoint: http://{HOST}:{PORT}{path}')
    print(f'Model: {model_file}')
    print(f'{"="*80}\n')
    
    results = []
    
    # Step 1: Spawn the model
    print(f'[Setup] Spawning {model_name}... ', end='', flush=True)
    spawn_result = wot_request(
        path,
        method='POST',
        value={'mode': 'load', 'fileName': model_file, 'modelName': model_name},
        timeout=10
    )
    
    if not spawn_result.get('success'):
        error_msg = spawn_result.get('error') or spawn_result.get('response_message') or 'Unknown error'
        print(f'✗ Failed to spawn: {error_msg}')
        return results
    
    print(f'✓ {spawn_result["response_time"]}ms')
    time.sleep(2.0)  # Wait for model to be registered
    
    # Step 2: Set multiple poses
    print(f'\nSetting poses:')
    for i in range(1, iterations + 1):
        # Generate different positions for each iteration
        x = float(i % 3)  # 0, 1, 2, 0, 1, 2, ...
        y = float((i // 3) % 3)  # 0, 0, 0, 1, 1, 1, 2, 2, 2, ...
        z = 0.5
        yaw = (i * 36) % 360  # Rotate 36 degrees each time
        
        timestamp = datetime.now().isoformat()
        print(f'[{i}/{iterations}] SetPose x={x}, y={y}, z={z}, yaw={yaw}° ', end='', flush=True)
        
        setpose_result = wot_request(
            path,
            method='POST',
            value={
                'mode': 'setPose',
                'modelName': model_name,
                'position': {'x': x, 'y': y, 'z': z},
                'orientation': {'roll': 0, 'pitch': 0, 'yaw': yaw}
            },
            timeout=10
        )
        setpose_result['iteration'] = i
        setpose_result['timestamp'] = timestamp
        setpose_result['endpoint'] = name
        setpose_result['operation'] = 'setPose'
        setpose_result['mode'] = 'setPose'
        setpose_result['model_name'] = model_name
        setpose_result['pose'] = f'x={x},y={y},z={z},yaw={yaw}'
        results.append(setpose_result)
        
        if setpose_result['success']:
            print(f"✓ {setpose_result['response_time']}ms")
        else:
            error_msg = setpose_result.get('error') or setpose_result.get('response_message') or 'Unknown error'
            print(f"✗ {setpose_result['response_time']}ms - {error_msg}")
        
        if i < iterations:
            time.sleep(INTERVAL_SEC)

    # Step 3: Delete the model
    print(f'\n[Cleanup] Deleting {model_name}... ', end='', flush=True)
    delete_result = wot_request(
        path,
        method='POST',
        value={'mode': 'remove', 'modelName': model_name},
        timeout=10
    )
    
    if delete_result.get('success'):
        print(f'✓ {delete_result["response_time"]}ms')
    else:
        error_msg = delete_result.get('error') or delete_result.get('response_message') or 'Unknown error'
        print(f'✗ {delete_result["response_time"]}ms - {error_msg}')
    
    return results


def run_spawn_delete_test(path, name, iterations, model_file, model_name_prefix='test_model', wait_after_spawn=2.0, wait_after_delete=None, position=None, orientation=None):
    """Run test iterations for spawning and deleting a model"""
    if wait_after_delete is None:
        wait_after_delete = INTERVAL_SEC
    
    print(f'\n{"="*80}')
    print(f'Testing {name}: {iterations} spawn-delete cycles')
    print(f'Endpoint: http://{HOST}:{PORT}{path}')
    print(f'Model: {model_file}')
    print(f'Wait after spawn: {wait_after_spawn}s | Wait after delete: {wait_after_delete}s')
    print(f'{"="*80}\n')
    
    results = []
    
    for i in range(1, iterations + 1):
        model_name = f'{model_name_prefix}_{i}'
        
        # Spawn model
        timestamp_spawn = datetime.now().isoformat()
        print(f'[{i}/{iterations}] Spawn {model_name} ', end='', flush=True)
        
        # Build spawn value with optional position and orientation
        spawn_value = {'mode': 'load', 'fileName': model_file, 'modelName': model_name}
        if position is not None:
            spawn_value['position'] = position
        if orientation is not None:
            spawn_value['orientation'] = orientation
        
        spawn_result = wot_request(
            path, 
            method='POST', 
            value=spawn_value,
            timeout=60  # Longer timeout for complex models like UR10
        )
        spawn_result['iteration'] = i
        spawn_result['timestamp'] = timestamp_spawn
        spawn_result['endpoint'] = name
        spawn_result['operation'] = 'spawn'
        spawn_result['mode'] = 'load'
        spawn_result['model_name'] = model_name
        results.append(spawn_result)
        
        if spawn_result['success']:
            print(f"✓ {spawn_result['response_time']}ms | Waiting {wait_after_spawn}s... ", end='', flush=True)
        else:
            error_msg = spawn_result.get('error') or spawn_result.get('response_message') or 'Unknown error'
            print(f"✗ {spawn_result['response_time']}ms - {error_msg}")
            if i < iterations:
                time.sleep(wait_after_delete)
            continue
        
        # Wait for entity to be fully registered and configured in Gazebo before deleting
        time.sleep(wait_after_spawn)
        
        # Delete model
        timestamp_delete = datetime.now().isoformat()
        print(f'Delete {model_name} ', end='', flush=True)
        
        delete_result = wot_request(
            path,
            method='POST',
            value={'mode': 'remove', 'modelName': model_name},
            timeout=10
        )
        delete_result['iteration'] = i
        delete_result['timestamp'] = timestamp_delete
        delete_result['endpoint'] = name
        delete_result['operation'] = 'delete'
        delete_result['mode'] = 'remove'
        delete_result['model_name'] = model_name
        results.append(delete_result)
        
        if delete_result['success']:
            print(f"✓ {delete_result['response_time']}ms | Verifying deletion... ", end='', flush=True)
            
            # For UR10, wait extra time and verify entity is actually gone
            if 'ur10' in model_file.lower():
                max_verify_attempts = 15  # 15 attempts * 2s = 30s max
                for attempt in range(max_verify_attempts):
                    time.sleep(2.0)
                    # Check if entity still exists
                    check_result = wot_request(f'/gz_controller/properties/models', method='GET', timeout=5)
                    if check_result['success'] and 'data' in check_result:
                        models = check_result['data'].get('models', [])
                        still_exists = any(m.get('name') == model_name for m in models)
                        if not still_exists:
                            print(f"✓ Verified after {(attempt+1)*2}s")
                            break
                    if attempt == max_verify_attempts - 1:
                        print(f"⚠ Timeout waiting for cleanup")
            else:
                print("✓")
        else:
            error_msg = delete_result.get('error') or delete_result.get('response_message') or 'Unknown error'
            print(f"✗ {delete_result['response_time']}ms - {error_msg}")
        
        if i < iterations:
            time.sleep(wait_after_delete)
    
    return results


def run_ur10_jointpositions_test(iterations=ITERATIONS, model_name='ur10_joint_test', interval=1.0):
    """Load UR10, read jointPositions (with UR10 TD baseline), test moveToJoint, gripper, moveToCartesian (no cleanup)"""
    print(f'\n{"="*80}')
    print(f'Testing UR10: Load, jointPositions {iterations}x (UR10 TD baseline), moveToJoint {iterations}x, gripper 5x, moveToCartesian {iterations}x')
    print(f'Read interval: {int(interval*1000)}ms | moveToJoint: 5s | Gripper: 2s (5 iterations) | moveToCartesian: 5s')
    print(f'Gazebo: http://{HOST}:{PORT}{MANAGEMODEL_PATH}')
    print(f'UR10 Server TD (baseline): http://{UR10_HOST}:{UR10_PORT}/ur10_server')
    print(f'UR10 Server jointPositions: http://{UR10_HOST}:{UR10_PORT}{UR10_JOINTPOSITIONS_PATH}')
    print(f'UR10 Server actions: http://{UR10_HOST}:{UR10_PORT}/ur10_server/actions/[moveToJoint|gripOpen|gripClose|moveToCartesian]')
    print(f'{"="*80}\n')
    
    results = []
    
    # Step 1: Spawn UR10
    print(f'[Setup] Spawning UR10 ({model_name})... ', end='', flush=True)
    spawn_result = wot_request(
        MANAGEMODEL_PATH,
        method='POST',
        value={
            'mode': 'load',
            'fileName': 'ur10_rg2.urdf',
            'modelName': model_name,
            'position': {'x': 0, 'y': 0, 'z': 0},
            'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}
        },
        timeout=60
    )
    
    if not spawn_result.get('success'):
        error_msg = spawn_result.get('error') or spawn_result.get('response_message') or 'Unknown error'
        print(f'✗ Failed to spawn: {error_msg}')
        return results
    
    print(f'✓ {spawn_result["response_time"]}ms (includes UR10 initialization)')
    print(f'[Setup] Waiting 5s for TD server to be ready...')
    time.sleep(5.0)  # Wait for TD server to be ready
    
    # Step 2: Read jointPositions multiple times (with UR10 TD baseline)
    print(f'\nReading jointPositions (with UR10 TD baseline):')
    for i in range(1, iterations + 1):
        timestamp = datetime.now().isoformat()
        
        # Read UR10 TD as baseline
        print(f'[{i}/{iterations}] UR10 TD baseline ', end='', flush=True)
        baseline_result = wot_request('/ur10_server', method='GET', timeout=5, host=UR10_HOST, port=UR10_PORT)
        baseline_latency = baseline_result['response_time']
        if baseline_result['success']:
            print(f"({baseline_latency}ms) → jointPositions ", end='', flush=True)
        else:
            print(f"(✗ {baseline_latency}ms) → jointPositions ", end='', flush=True)
        
        # Read jointPositions
        jointpos_result = wot_request(
            UR10_JOINTPOSITIONS_PATH,
            method='GET',
            timeout=5,
            host=UR10_HOST,
            port=UR10_PORT
        )
        jointpos_result['iteration'] = i
        jointpos_result['timestamp'] = timestamp
        jointpos_result['endpoint'] = 'UR10_jointPositions'
        jointpos_result['operation'] = 'read'
        jointpos_result['model_name'] = model_name
        jointpos_result['baseline_latency'] = baseline_latency
        jointpos_result['normalized_latency'] = jointpos_result['response_time'] - baseline_latency
        results.append(jointpos_result)
        
        if jointpos_result['success']:
            normalized = jointpos_result['normalized_latency']
            # Extract joint values for display
            if 'data' in jointpos_result:
                joints = jointpos_result['data']
                joint_str = ', '.join([f"{k.replace('_joint', '')}={v:.1f}°" 
                                      for k, v in joints.items() if k.endswith('_joint')])
                print(f"✓ {jointpos_result['response_time']}ms (normalized: {normalized:+d}ms)")
            else:
                print(f"✓ {jointpos_result['response_time']}ms (normalized: {normalized:+d}ms)")
        else:
            error_msg = jointpos_result.get('error') or jointpos_result.get('response_message') or 'Unknown error'
            print(f"✗ {jointpos_result['response_time']}ms - {error_msg}")
        
        if i < iterations:
            time.sleep(interval)
    
    # Step 4: Test moveToJoint multiple times
    print(f'\nTesting moveToJoint:')
    
    # Define different joint configurations to test
    joint_configs = [
        {'shoulder_pan_joint': 0, 'shoulder_lift_joint': -90, 'elbow_joint': 90, 'wrist_1_joint': -90, 'wrist_2_joint': 90, 'wrist_3_joint': 0},
        {'shoulder_pan_joint': 45, 'shoulder_lift_joint': -45, 'elbow_joint': 45, 'wrist_1_joint': -45, 'wrist_2_joint': 45, 'wrist_3_joint': 45},
        {'shoulder_pan_joint': -45, 'shoulder_lift_joint': -60, 'elbow_joint': 60, 'wrist_1_joint': -60, 'wrist_2_joint': 60, 'wrist_3_joint': -45},
        {'shoulder_pan_joint': 90, 'shoulder_lift_joint': -30, 'elbow_joint': 30, 'wrist_1_joint': -30, 'wrist_2_joint': 30, 'wrist_3_joint': 90},
        {'shoulder_pan_joint': 0, 'shoulder_lift_joint': -45, 'elbow_joint': 45, 'wrist_1_joint': -45, 'wrist_2_joint': 45, 'wrist_3_joint': 0},
        {'shoulder_pan_joint': 0, 'shoulder_lift_joint': -90, 'elbow_joint': 90, 'wrist_1_joint': -90, 'wrist_2_joint': 90, 'wrist_3_joint': 0},
        {'shoulder_pan_joint': 45, 'shoulder_lift_joint': -45, 'elbow_joint': 45, 'wrist_1_joint': -45, 'wrist_2_joint': 45, 'wrist_3_joint': 45},
        {'shoulder_pan_joint': -45, 'shoulder_lift_joint': -60, 'elbow_joint': 60, 'wrist_1_joint': -60, 'wrist_2_joint': 60, 'wrist_3_joint': -45},
        {'shoulder_pan_joint': 90, 'shoulder_lift_joint': -30, 'elbow_joint': 30, 'wrist_1_joint': -30, 'wrist_2_joint': 30, 'wrist_3_joint': 90},
        {'shoulder_pan_joint': 90, 'shoulder_lift_joint': -90, 'elbow_joint': 90, 'wrist_1_joint': 0, 'wrist_2_joint': 0, 'wrist_3_joint': -90},
    ]
    
    for i in range(1, iterations + 1):
        timestamp = datetime.now().isoformat()
        config = joint_configs[(i - 1) % len(joint_configs)]
        config_str = f"pan={config['shoulder_pan_joint']}°, lift={config['shoulder_lift_joint']}°"
        
        print(f'[{i}/{iterations}] moveToJoint {config_str} ', end='', flush=True)
        
        movejoint_result = wot_request(
            '/ur10_server/actions/moveToJoint',
            method='POST',
            value=config,
            timeout=30,
            host=UR10_HOST,
            port=UR10_PORT
        )
        movejoint_result['iteration'] = i
        movejoint_result['timestamp'] = timestamp
        movejoint_result['endpoint'] = 'UR10_moveToJoint'
        movejoint_result['operation'] = 'action'
        movejoint_result['model_name'] = model_name
        movejoint_result['joint_config'] = config_str
        results.append(movejoint_result)
        
        if movejoint_result['success']:
            print(f"✓ {movejoint_result['response_time']}ms")
        else:
            error_msg = movejoint_result.get('error') or movejoint_result.get('response_message') or 'Unknown error'
            print(f"✗ {movejoint_result['response_time']}ms - {error_msg}")
        
        if i < iterations:
            time.sleep(5.0)  # 5s interval between moveToJoint commands
    
    # Step 5: Test gripper open-close multiple times (5 iterations)
    print(f'\nTesting gripper open-close:')
    
    gripper_actions = ['gripOpen', 'gripClose']
    gripper_iterations = 5
    
    for i in range(1, gripper_iterations + 1):
        timestamp = datetime.now().isoformat()
        action_name = gripper_actions[i % 2]  # Alternate between open and close
        
        print(f'[{i}/{gripper_iterations}] {action_name} ', end='', flush=True)
        
        grip_result = wot_request(
            f'/ur10_server/actions/{action_name}',
            method='POST',
            value=None,  # No input required
            timeout=10,
            host=UR10_HOST,
            port=UR10_PORT
        )
        grip_result['iteration'] = i
        grip_result['timestamp'] = timestamp
        grip_result['endpoint'] = f'UR10_{action_name}'
        grip_result['operation'] = 'action'
        grip_result['model_name'] = model_name
        grip_result['gripper_action'] = action_name
        results.append(grip_result)
        
        if grip_result['success']:
            msg = grip_result.get('response_message', '')
            print(f"✓ {grip_result['response_time']}ms | {msg}")
        else:
            error_msg = grip_result.get('error') or grip_result.get('response_message') or 'Unknown error'
            print(f"✗ {grip_result['response_time']}ms - {error_msg}")
        
        if i < gripper_iterations:
            time.sleep(2.0)  # 2s interval between gripper commands
    
    # Step 6: Test moveToCartesian multiple times
    print(f'\nTesting moveToCartesian:')
    
    # Define different cartesian positions to test
    cartesian_configs = [
        {'position': {'x': 0.3, 'y': 0.2, 'z': 0.3}, 'orientation': {'roll': 0, 'pitch': 90, 'yaw': 0}},
        {'position': {'x': 0.3, 'y': -0.2, 'z': 0.3}, 'orientation': {'roll': 0, 'pitch': 90, 'yaw': 45}},
        {'position': {'x': 0.4, 'y': 0.0, 'z': 0.4}, 'orientation': {'roll': 0, 'pitch': 90, 'yaw': 90}},
        {'position': {'x': 0.3, 'y': 0.2, 'z': 0.2}, 'orientation': {'roll': 0, 'pitch': 90, 'yaw': -45}},
        {'position': {'x': 0.35, 'y': 0.0, 'z': 0.35}, 'orientation': {'roll': 0, 'pitch': 90, 'yaw': 0}},
    ]
    
    for i in range(1, iterations + 1):
        timestamp = datetime.now().isoformat()
        config = cartesian_configs[(i - 1) % len(cartesian_configs)]
        pos = config['position']
        config_str = f"x={pos['x']}, y={pos['y']}, z={pos['z']}"
        
        print(f'[{i}/{iterations}] moveToCartesian {config_str} ', end='', flush=True)
        
        cartesian_result = wot_request(
            '/ur10_server/actions/moveToCartesian',
            method='POST',
            value=config,
            timeout=30,
            host=UR10_HOST,
            port=UR10_PORT
        )
        cartesian_result['iteration'] = i
        cartesian_result['timestamp'] = timestamp
        cartesian_result['endpoint'] = 'UR10_moveToCartesian'
        cartesian_result['operation'] = 'action'
        cartesian_result['model_name'] = model_name
        cartesian_result['cartesian_config'] = config_str
        results.append(cartesian_result)
        
        if cartesian_result['success']:
            msg = cartesian_result.get('response_message', '')
            print(f"✓ {cartesian_result['response_time']}ms | {msg}")
        else:
            error_msg = cartesian_result.get('error') or cartesian_result.get('response_message') or 'Unknown error'
            print(f"✗ {cartesian_result['response_time']}ms - {error_msg}")
        
        if i < iterations:
            time.sleep(5.0)  # 5s interval between moveToCartesian commands
    
    # Step 7: Delete UR10 (SKIPPED - manual cleanup needed)
    print(f'\n[Info] UR10 cleanup skipped - {model_name} remains in scene for manual testing')
    print(f'[Info] To delete: curl -X POST -H "Content-Type: application/json" -d \'{{"mode":"remove","modelName":"{model_name}"}}\' http://{HOST}:{PORT}{MANAGEMODEL_PATH}')
    
    return results


def print_stats(results, name):
    """Print statistics for a test run"""
    successful = [r for r in results if r['success']]
    
    print(f'\n{"-"*80}')
    print(f'{name} Summary:')
    print(f'{"-"*80}')
    print(f'Total: {len(results)} | Success: {len(successful)} | Errors: {len(results) - len(successful)}')
    
    if successful:
        times = [r['response_time'] for r in successful]
        print(f'Response times: Avg={sum(times)/len(times):.2f}ms | Min={min(times)}ms | Max={max(times)}ms')
        
        # Show normalized latency if baseline was measured
        with_baseline = [r for r in successful if 'normalized_latency' in r and r.get('baseline_latency', 0) > 0]
        if with_baseline:
            normalized_times = [r['normalized_latency'] for r in with_baseline]
            baseline_times = [r['baseline_latency'] for r in with_baseline]
            print(f'Baseline TD: Avg={sum(baseline_times)/len(baseline_times):.2f}ms')
            print(f'Normalized: Avg={sum(normalized_times)/len(normalized_times):.2f}ms | Min={min(normalized_times)}ms | Max={max(normalized_times)}ms')


def export_to_csv(all_results):
    """Export results to CSV file"""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_file = os.path.join(OUTPUT_DIR, f'wot_latency_{timestamp}.csv')
    
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Timestamp', 'Endpoint', 'Operation', 'Iteration', 'Value', 'Mode', 'Model_Name', 'Pose', 'Joint_Config', 'Cartesian_Config', 'Gripper_Action', 'Baseline_Latency_ms', 'Response_Time_ms', 'Normalized_Latency_ms', 'Success', 'Status_Code', 'Error'])
        
        for r in all_results:
            writer.writerow([
                r['timestamp'],
                r['endpoint'],
                r.get('operation', 'read'),
                r['iteration'],
                r.get('value', ''),
                r.get('mode', ''),
                r.get('model_name', ''),
                r.get('pose', ''),
                r.get('joint_config', ''),
                r.get('cartesian_config', ''),
                r.get('gripper_action', ''),
                r.get('baseline_latency', ''),
                r['response_time'],
                r.get('normalized_latency', ''),
                r['success'],
                r.get('status_code', ''),
                r.get('error', '')
            ])
    
    print(f'\n✓ Data exported to: {csv_file}')
    return csv_file


# ============================================================================
# TEST CONFIGURATION - Easy enable/disable and reorder tests
# ============================================================================

TEST_SUITE = [
    {
        'name': 'SimStats_Read',
        'enabled': True,
        'description': 'Read simStats property 10 times (with TD baseline)',
        'function': lambda: run_test(SIMSTATS_PATH, 'simStats', ITERATIONS, with_baseline=True),
        'stats_title': 'simStats Property'
    },
    {
        'name': 'Visualize_Toggle',
        'enabled': True,
        'description': 'Toggle visualize property 20 times, then set to true',
        'function': lambda: run_visualize_toggle_test(),
        'stats_title': 'Visualize Toggle'
    },
    {
        'name': 'SimControl_RunPause',
        'enabled': True,
        'description': 'Toggle run/pause 20 times',
        'function': lambda: run_action_toggle_test(SIMCONTROL_PATH, 'simControl', TOGGLE_ITERATIONS, 'run', 'pause'),
        'stats_title': 'SimControl Run/Pause Toggle'
    },
    {
        'name': 'SimControl_Speed',
        'enabled': True,
        'description': 'Toggle faster/slower 20 times',
        'function': lambda: run_action_toggle_test(SIMCONTROL_PATH, 'simControl', TOGGLE_ITERATIONS, 'faster', 'slower'),
        'stats_title': 'SimControl Faster/Slower Toggle'
    },
    {
        'name': 'Box_SpawnDelete',
        'enabled': True,
        'description': 'Spawn-Delete box.urdf 10 times',
        'function': lambda: run_spawn_delete_test(MANAGEMODEL_PATH, 'manageModel', ITERATIONS, 'box.urdf', 'test_box', wait_after_spawn=2.0, wait_after_delete=2.0),
        'stats_title': 'Spawn-Delete Model (box.urdf)'
    },
    {
        'name': 'Box_SetPose',
        'enabled': True,
        'description': 'Load box, set 10 poses, delete',
        'function': lambda: run_setpose_test(MANAGEMODEL_PATH, 'manageModel', ITERATIONS, 'box.urdf', 'box_setpose'),
        'stats_title': 'SetPose Test (box.urdf)'
    },
    {
        'name': 'UR10_Full_Test',
        'enabled': True,
        'description': 'Load UR10, jointPos 10x with UR10 TD baseline (1s), moveJoint 10x (5s), gripper 5x (2s), moveCart 10x (5s)',
        'function': lambda: run_ur10_jointpositions_test(ITERATIONS, 'ur10_joint_test', interval=1.0),
        'stats_title': 'UR10 Full Test (JointPos + MoveJoint + Gripper + MoveCartesian)'
    },
    {
        'name': 'UR10_SpawnDelete',
        'enabled': True,
        'description': 'Spawn-Delete ur10_rg2.urdf 10 times (SLOW: ~70s per cycle)',
        'function': lambda: run_spawn_delete_test(MANAGEMODEL_PATH, 'manageModel', ITERATIONS, 'ur10_rg2.urdf', 'ur10_test', wait_after_spawn=30.0, wait_after_delete=5.0, position={'x': 0, 'y': 0, 'z': 0}, orientation={'roll': 0, 'pitch': 0, 'yaw': 0}),
        'stats_title': 'Spawn-Delete UR10 Robot (ur10_rg2.urdf)'
    },
]


def main():
    """Main execution"""
    print('\n' + '█'*80)
    print('WoT OPERATIONS LATENCY TEST')
    print('█'*80)
    
    # Display test configuration
    enabled_tests = [t for t in TEST_SUITE if t['enabled']]
    print(f'\nEnabled tests: {len(enabled_tests)}/{len(TEST_SUITE)}')
    for test in enabled_tests:
        print(f'  ✓ {test["name"]}: {test["description"]}')
    
    if not enabled_tests:
        print('\n⚠ No tests enabled! Please enable tests in TEST_SUITE configuration.')
        return

    print()
    
    # Run enabled tests
    all_results = []
    test_summaries = []
    
    for test in enabled_tests:
        print(f'\n{"─"*80}')
        print(f'Running: {test["name"]}')
        print(f'{"─"*80}')
        
        results = test['function']()
        all_results.extend(results)
        
        print_stats(results, test['stats_title'])
        
        # Store summary for this test
        successful = [r for r in results if r['success']]
        test_summaries.append({
            'name': test['name'],
            'total': len(results),
            'successful': len(successful),
            'failed': len(results) - len(successful)
        })
    
    # Overall summary
    all_successful = [r for r in all_results if r['success']]
    
    print(f'\n{"="*80}')
    print('OVERALL SUMMARY')
    print(f'{"="*80}')
    print(f'Total operations: {len(all_results)}')
    
    for summary in test_summaries:
        print(f'  - {summary["name"]}: {summary["total"]} ops (Success: {summary["successful"]}, Failed: {summary["failed"]})')
    
    if len(all_results) > 0:
        print(f'\nOverall Success Rate: {len(all_successful)}/{len(all_results)} ({len(all_successful)/len(all_results)*100:.1f}%)')
    
        if all_successful:
            all_times = [r['response_time'] for r in all_successful]
            print(f'Average response time: {sum(all_times)/len(all_times):.2f}ms')
            print(f'Min: {min(all_times)}ms | Max: {max(all_times)}ms')
    else:
        print('\n⚠ No operations completed')
    
    # Export to CSV
    csv_file = export_to_csv(all_results)
    
    print(f'\n{"="*80}')
    print('Done!')
    print(f'{"="*80}\n')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n\nInterrupted by user')
    except Exception as e:
        print(f'\nFatal error: {e}')
        exit(1)
