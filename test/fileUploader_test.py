#!/usr/bin/env python3
"""
File Upload Latency Test Script
Tests file upload to fileUploader service (port 8082) using multipart/form-data
Measures upload latency and exports results to CSV

Target: http://localhost:8082/upload

Default All-Enabled Test Sequence (4 tests, ~40 uploads):
┌─────────────────────────────────────────────────────────────────────────┐
│ 1. URDF_Upload    - Upload ur10_rg2_gazebo.urdf to Gazebo (14 KB, 10x)  │
│                     → Assets/urdf/uploaded/{modelName}/                 │
│ 2. SDF_Upload     - Upload diff_drive.sdf to Gazebo (10x)               │
│                     → Assets/gazebo/worlds/uploaded/                    │
│ 3. Launch_Upload  - Upload diff_drive.launch.py to Gazebo (10x)         │
│                     → Assets/gazebo/launch/uploaded/                    │
│ 4. TTT_Upload     - Upload IoTRemoteLab.ttt to CoppeliaSim (10x)        │
│                     → Assets/coppeliasim/scenes/uploaded/               │
└─────────────────────────────────────────────────────────────────────────┘

Upload Targets:
- URDF files (Gazebo): Create folder + model.config in /urdf/uploaded
- SDF models (Gazebo): Create folder + model.config in /gazebo/models/uploaded
- SDF worlds (Gazebo): Direct save to /gazebo/worlds/uploaded
- Launch files (Gazebo): Direct save to /gazebo/launch/uploaded
- TTM files (CoppeliaSim): Direct save to /coppeliasim/models/uploaded
- TTT files (CoppeliaSim): Direct save to /coppeliasim/scenes/uploaded

Total Estimated Time: ~1-2 minutes
CSV Export: upload_latency_YYYYMMDD_HHMMSS.csv
Automatic Cleanup: Removes all test files after completion

Timeout Configuration:
- Auto-calculated based on file size (60s base + 10s per MB)
- Small files (<1MB): 60s timeout
- Large files (11MB .ttt): 170s timeout (~3 minutes)

Configuration:
- Edit UPLOAD_TEST_SUITE to enable/disable individual tests
- Adjust BASE_ASSETS_PATH if Assets folder location changes
- Adjust ITERATIONS, INTERVAL_SEC as needed
"""

import http.client
import json
import time
import csv
import os
import mimetypes
from datetime import datetime
from io import BytesIO

# Configuration
UPLOAD_HOST = 'localhost'
UPLOAD_PORT = 8082
UPLOAD_PATH = '/upload'

# Please prepare the test files and use the real assets path.
# Base paths
BASE_ASSETS_PATH = '/path/to/test/assets'

# Test files (relative to BASE_ASSETS_PATH)
TEST_FILES = {
    'urdf': os.path.join(BASE_ASSETS_PATH, 'test1.urdf'),
    'sdf': os.path.join(BASE_ASSETS_PATH, 'test2.sdf'),
    'launch': os.path.join(BASE_ASSETS_PATH, 'test3.launch.py'),
    'ttt': os.path.join(BASE_ASSETS_PATH, 'test4.ttt'),
}

ITERATIONS = 10
INTERVAL_SEC = 2.0
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))


def create_multipart_formdata(fields, files):
    """
    Create multipart/form-data body
    
    Args:
        fields: dict of field name -> value
        files: dict of field name -> (filename, content, content_type)
    
    Returns:
        tuple: (content_type, body)
    """
    boundary = '----WebKitFormBoundary' + str(int(time.time() * 1000))
    body = BytesIO()
    
    # Add fields
    for field_name, value in fields.items():
        body.write(f'--{boundary}\r\n'.encode('utf-8'))
        body.write(f'Content-Disposition: form-data; name="{field_name}"\r\n\r\n'.encode('utf-8'))
        body.write(f'{value}\r\n'.encode('utf-8'))
    
    # Add files
    for field_name, (filename, content, content_type) in files.items():
        body.write(f'--{boundary}\r\n'.encode('utf-8'))
        body.write(f'Content-Disposition: form-data; name="{field_name}"; filename="{filename}"\r\n'.encode('utf-8'))
        body.write(f'Content-Type: {content_type}\r\n\r\n'.encode('utf-8'))
        body.write(content)
        body.write(b'\r\n')
    
    # End boundary
    body.write(f'--{boundary}--\r\n'.encode('utf-8'))
    
    content_type = f'multipart/form-data; boundary={boundary}'
    return content_type, body.getvalue()


def upload_file(file_path, simulator='gazebo', target='model', timeout=30, unique_suffix=None):
    """
    Upload file using multipart/form-data and measure response time
    
    Args:
        file_path: Path to file to upload
        simulator: 'gazebo' or 'coppeliasim'
        target: Target type (e.g., 'model', 'world', 'launch')
        timeout: Request timeout in seconds
        unique_suffix: Optional suffix to add to filename for uniqueness
    
    Returns:
        dict: Result with success, response_time, and details
    """
    start_time = time.time()
    
    try:
        # Read file content
        with open(file_path, 'rb') as f:
            file_content = f.read()
        
        filename = os.path.basename(file_path)
        
        # Add unique suffix if provided
        if unique_suffix:
            name, ext = os.path.splitext(filename)
            filename = f'{name}_{unique_suffix}{ext}'
        
        # Guess content type
        content_type = mimetypes.guess_type(file_path)[0] or 'application/octet-stream'
        
        # Create multipart form data
        fields = {
            'simulator': simulator,
            'target': target
        }
        files = {
            'file': (filename, file_content, content_type)
        }
        
        multipart_content_type, body = create_multipart_formdata(fields, files)
        
        # Send request
        conn = http.client.HTTPConnection(UPLOAD_HOST, UPLOAD_PORT, timeout=timeout)
        headers = {
            'Content-Type': multipart_content_type,
            'Content-Length': str(len(body))
        }
        
        conn.request('POST', UPLOAD_PATH, body=body, headers=headers)
        response = conn.getresponse()
        data = response.read().decode('utf-8')
        conn.close()
        
        response_time = int((time.time() - start_time) * 1000)
        
        # Parse response
        result = {
            'success': response.status == 200,
            'response_time': response_time,
            'status_code': response.status,
            'filename': filename,
            'file_size': len(file_content)
        }
        
        try:
            json_data = json.loads(data)
            result['data'] = json_data
            if 'success' in json_data:
                result['success'] = json_data['success']
            if 'message' in json_data:
                result['response_message'] = json_data['message']
        except json.JSONDecodeError:
            pass
        
        return result
        
    except Exception as e:
        result = {
            'success': False,
            'response_time': int((time.time() - start_time) * 1000),
            'error': str(e),
            'filename': os.path.basename(file_path)
        }
        return result


def run_upload_test(file_path, simulator='gazebo', iterations=10, interval=2.0):
    """Run file upload test multiple times"""
    filename = os.path.basename(file_path)
    file_size_kb = os.path.getsize(file_path) / 1024
    
    # Calculate timeout based on file size (minimum 60s, +10s per MB)
    file_size_mb = file_size_kb / 1024
    upload_timeout = max(60, int(60 + (file_size_mb * 10)))
    
    print(f'\n{"="*80}')
    print(f'File Upload Test: {iterations} uploads with {int(interval*1000)}ms interval')
    print(f'Endpoint: http://{UPLOAD_HOST}:{UPLOAD_PORT}{UPLOAD_PATH}')
    print(f'File: {filename} ({file_size_kb:.2f} KB)')
    print(f'Simulator: {simulator}')
    print(f'Timeout: {upload_timeout}s (auto-calculated for file size)')
    print(f'{"="*80}\n')
    
    results = []
    
    for i in range(1, iterations + 1):
        timestamp = datetime.now().isoformat()
        unique_suffix = f'{int(time.time())}_{i}'  # Timestamp + iteration for uniqueness
        print(f'[{i}/{iterations}] Uploading {filename} ', end='', flush=True)
        
        result = upload_file(file_path, simulator=simulator, timeout=upload_timeout, unique_suffix=unique_suffix)
        result['iteration'] = i
        result['timestamp'] = timestamp
        result['endpoint'] = 'fileUploader'
        result['operation'] = 'upload'
        result['simulator'] = simulator
        results.append(result)
        
        if result['success']:
            msg = result.get('response_message', '')
            print(f"✓ {result['response_time']}ms | {msg}")
        else:
            error_msg = result.get('error') or result.get('response_message') or 'Unknown error'
            print(f"✗ {result['response_time']}ms - {error_msg}")
        
        if i < iterations:
            time.sleep(interval)
    
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


def export_to_csv(all_results):
    """Export results to CSV file"""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_file = os.path.join(OUTPUT_DIR, f'upload_latency_{timestamp}.csv')
    
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Timestamp', 'Endpoint', 'Operation', 'Iteration', 'Simulator', 'Filename', 'File_Size_Bytes', 'Response_Time_ms', 'Success', 'Status_Code', 'Error'])
        
        for r in all_results:
            writer.writerow([
                r['timestamp'],
                r['endpoint'],
                r.get('operation', 'upload'),
                r['iteration'],
                r.get('simulator', ''),
                r.get('filename', ''),
                r.get('file_size', ''),
                r['response_time'],
                r['success'],
                r.get('status_code', ''),
                r.get('error', '')
            ])
    
    print(f'\n✓ Data exported to: {csv_file}')
    return csv_file


def cleanup_test_files():
    """Clean up test upload files"""
    import subprocess
    
    print(f'\n{"="*80}')
    print('CLEANUP')
    print(f'{"="*80}')
    
    cleanup_script = os.path.join(OUTPUT_DIR, 'cleanup_test_uploads.sh')
    
    if os.path.exists(cleanup_script):
        try:
            result = subprocess.run([cleanup_script], capture_output=True, text=True, timeout=10)
            print(result.stdout)
            if result.returncode == 0:
                print('✓ All test files removed')
            else:
                print(f'⚠ Cleanup script exited with code {result.returncode}')
                if result.stderr:
                    print(f'Error: {result.stderr}')
        except Exception as e:
            print(f'⚠ Failed to run cleanup script: {e}')
    else:
        print(f'⚠ Cleanup script not found: {cleanup_script}')


# ============================================================================
# TEST CONFIGURATION - Easy enable/disable file upload tests
# ============================================================================

UPLOAD_TEST_SUITE = [
    {
        'name': 'URDF_Upload',
        'enabled': True,
        'file_path': TEST_FILES['urdf'],
        'simulator': 'gazebo',
        'description': 'Upload ur10_rg2_gazebo.urdf to Gazebo (14 KB)',
        'stats_title': 'URDF Upload Test (ur10_rg2_gazebo.urdf)'
    },
    {
        'name': 'SDF_Upload',
        'enabled': True,
        'file_path': TEST_FILES['sdf'],
        'simulator': 'gazebo',
        'description': 'Upload diff_drive.sdf to Gazebo',
        'stats_title': 'SDF Upload Test (diff_drive.sdf)'
    },
    {
        'name': 'Launch_Upload',
        'enabled': True,
        'file_path': TEST_FILES['launch'],
        'simulator': 'gazebo',
        'description': 'Upload diff_drive.launch.py to Gazebo',
        'stats_title': 'Launch File Upload Test (diff_drive.launch.py)'
    },
    {
        'name': 'TTT_Upload',
        'enabled': True,
        'file_path': TEST_FILES['ttt'],
        'simulator': 'coppeliasim',
        'description': 'Upload IoTRemoteLab.ttt to CoppeliaSim',
        'stats_title': 'Scene Upload Test (IoTRemoteLab.ttt)'
    },
]


def main():
    """Main execution"""
    print('\n' + '█'*80)
    print('FILE UPLOAD LATENCY TEST')
    print('█'*80)
    
    # Display test configuration
    enabled_tests = [t for t in UPLOAD_TEST_SUITE if t['enabled']]
    print(f'\nEnabled tests: {len(enabled_tests)}/{len(UPLOAD_TEST_SUITE)}')
    for test in enabled_tests:
        print(f'  ✓ {test["name"]}: {test["description"]}')
    
    if not enabled_tests:
        print('\n⚠ No tests enabled! Please enable tests in UPLOAD_TEST_SUITE configuration.')
        return
    
    print()
    
    # Run enabled tests
    all_results = []
    test_summaries = []
    
    for test in enabled_tests:
        print(f'\n{"─"*80}')
        print(f'Running: {test["name"]}')
        print(f'{"─"*80}')
        
        # Check if file exists
        if not os.path.exists(test['file_path']):
            print(f'⚠ Skipping {test["name"]}: File not found - {test["file_path"]}')
            continue
        
        results = run_upload_test(test['file_path'], simulator=test['simulator'], iterations=ITERATIONS, interval=INTERVAL_SEC)
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
    print(f'Total uploads: {len(all_results)}')
    
    for summary in test_summaries:
        print(f'  - {summary["name"]}: {summary["total"]} ops (Success: {summary["successful"]}, Failed: {summary["failed"]})')
    
    print(f'\nOverall Success Rate: {len(all_successful)}/{len(all_results)} ({len(all_successful)/len(all_results)*100:.1f}%)')
    
    if all_successful:
        all_times = [r['response_time'] for r in all_successful]
        print(f'Average response time: {sum(all_times)/len(all_times):.2f}ms')
        print(f'Min: {min(all_times)}ms | Max: {max(all_times)}ms')
    
    # Export to CSV
    csv_file = export_to_csv(all_results)
    
    # Clean up test files
    cleanup_test_files()
    
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

