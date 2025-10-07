#!/usr/bin/env python3
"""
Scan Vendor Assets

This script scans specified directories and generates a structured JSON file
with all available asset files (.ttt, .ttm, etc.).

Usage:
    python3 scan_vendor_assets.py [vendor_root] [folders] [output_file]
    
Arguments:
    vendor_root: Root path to scan (default: /opt/coppeliasim410)
    folders:     Comma-separated folder names to scan (default: scenes)
                 Format: [folder1,folder2,folder3]
                 Example: [scenes,models]
    output_file: Output JSON file path (default: /project-root/library/coppeliasim/vendor_assets.json)

Examples:
    # Scan only scenes folder (default)
    python3 scan_vendor_assets.py
    
    # Scan scenes and models folders
    python3 scan_vendor_assets.py /opt/coppeliasim410 [scenes,models] ./vendor_assets.json
    
    # Scan custom folders
    python3 scan_vendor_assets.py /opt/coppeliasim410 [scenes,models,lua] /tmp/output.json

Author: Yifan & Cursor
Version: 2.0.0
"""

import os
import sys
import json
from datetime import datetime
from pathlib import Path

# Default values
DEFAULT_VENDOR_ROOT = '/opt/coppeliasim410'
DEFAULT_FOLDERS = 'scenes'
DEFAULT_OUTPUT_FILE = '/project-root/library/coppeliasim/vendor_assets.json'


def scan_directory(dir_path, file_extensions=None):
    """
    Recursively scan directory and build file structure.
    
    Args:
        dir_path: Directory path to scan
        file_extensions: List of file extensions to include (e.g., ['.ttt', '.ttm'])
                        If None, includes .ttt, .ttm, .lua
        
    Returns:
        Dictionary with structured file information
    """
    if file_extensions is None:
        file_extensions = ['.ttt', '.ttm', '.lua']
    
    structure = {}
    files_in_current_dir = []
    
    try:
        if not os.path.exists(dir_path):
            return structure
            
        entries = os.listdir(dir_path)
        entries.sort()  # Sort for consistent output
        
        for entry in entries:
            full_path = os.path.join(dir_path, entry)
            
            if os.path.isdir(full_path):
                # Recursively scan subdirectories
                sub_structure = scan_directory(full_path, file_extensions)
                if sub_structure:  # Only include non-empty directories
                    structure[entry] = sub_structure
                    
            elif os.path.isfile(full_path):
                # Check if file has one of the target extensions
                if any(entry.endswith(ext) for ext in file_extensions):
                    files_in_current_dir.append(entry)
        
        # If there are files in current directory, add them directly
        # without wrapping in parent directory name
        if files_in_current_dir:
            # If this directory also has subdirectories, we need to handle it
            if structure:
                # Has both files and subdirs - this shouldn't happen often
                # but if it does, add files under 'ungrouped' key
                structure['ungrouped'] = files_in_current_dir
            else:
                # Only has files, return them as a simple array at this level
                # The parent will handle putting them under the right key
                return files_in_current_dir
        
    except PermissionError as e:
        print(f"Permission denied: {dir_path}")
    except Exception as e:
        print(f"Error scanning {dir_path}: {e}")
    
    return structure


def count_files(structure):
    """
    Count total number of files in structure.
    
    Args:
        structure: File structure dictionary
        
    Returns:
        Total file count
    """
    count = 0
    
    for value in structure.values():
        if isinstance(value, list):
            count += len(value)
        elif isinstance(value, dict):
            count += count_files(value)
    
    return count


def flatten_structure(structure):
    """
    Flatten structure to remove redundant nesting.
    
    Args:
        structure: File structure dictionary
        
    Returns:
        Flattened structure
    """
    if not isinstance(structure, dict):
        return structure
    
    # Recursively flatten nested structures
    flattened = {}
    for key, value in structure.items():
        if isinstance(value, dict):
            flattened[key] = flatten_structure(value)
        else:
            flattened[key] = value
    
    return flattened


def print_help():
    """Print help message."""
    print(__doc__)


def main():
    """Main function to scan vendor assets and save to JSON."""
    # Parse command line arguments
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help', 'help']:
        print_help()
        return 0
    
    vendor_root = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_VENDOR_ROOT
    folders_str = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_FOLDERS
    output_file = sys.argv[3] if len(sys.argv) > 3 else DEFAULT_OUTPUT_FILE
    
    # Parse folder names from comma-separated string
    # Strip square brackets if present
    folders_str = folders_str.strip('[]')
    folders = [f.strip() for f in folders_str.split(',')]
    
    print(f"Vendor root path: {vendor_root}")
    print(f"Folders to scan: {', '.join(folders)}")
    print(f"Output file: {output_file}")
    print()
    
    # Check if vendor root directory exists
    if not os.path.exists(vendor_root):
        print(f"Error: Vendor root directory not found: {vendor_root}")
        print()
        print("Usage: python3 scan_vendor_assets.py [vendor_root] [folders] [output_file]")
        print(f"  vendor_root: Root path (default: {DEFAULT_VENDOR_ROOT})")
        print(f"  folders:     Comma-separated folder names (default: {DEFAULT_FOLDERS})")
        print(f"  output_file: Output JSON file (default: {DEFAULT_OUTPUT_FILE})")
        return 1
    
    # Scan each folder
    all_assets = {}
    total_files = 0
    
    for folder in folders:
        folder_path = os.path.join(vendor_root, folder)
        
        if not os.path.exists(folder_path):
            print(f"⚠ Warning: Folder not found, skipping: {folder_path}")
            continue
        
        print(f"Scanning {folder}...")
        structure = scan_directory(folder_path)
        structure = flatten_structure(structure)
        
        if structure:
            all_assets[folder] = structure
            file_count = count_files(structure)
            total_files += file_count
            print(f"  ✓ Found {file_count} files in {folder}")
    
    if not all_assets:
        print(f"Error: No assets found in any of the specified folders")
        return 1
    
    print()
    
    # Create output object
    output = {
        "metadata": {
            "scannedAt": datetime.now().isoformat(),
            "vendorRoot": vendor_root,
            "foldersScanned": folders,
            "generatedBy": "scan_vendor_assets.py"
        },
        "assets": all_assets
    }
    
    # Ensure output directory exists
    output_dir = os.path.dirname(output_file)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    
    # Write to JSON file
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(output, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"Error writing output file: {e}")
        return 1
    
    print(f"✓ Vendor assets scanned successfully!")
    print(f"✓ Output saved to: {output_file}")
    print(f"✓ Total files found: {total_files}")
    
    return 0


if __name__ == '__main__':
    exit(main())

