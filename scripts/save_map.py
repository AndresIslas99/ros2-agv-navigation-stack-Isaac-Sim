#!/usr/bin/env python3
# =============================================================================
# Save Map Script
# Author: Andrés Islas Bravo
# Description: Exports RTAB-Map database to 2D occupancy grid for Nav2
# Usage: ros2 run ros2_agv_navigation_stack save_map.py --map-name my_map
# =============================================================================

import argparse
import subprocess
import os
import sys


def main():
    parser = argparse.ArgumentParser(
        description='Export RTAB-Map database to 2D occupancy grid'
    )
    parser.add_argument(
        '--map-name', '-n',
        type=str,
        default='warehouse_map',
        help='Name for the output map files (default: warehouse_map)'
    )
    parser.add_argument(
        '--output-dir', '-o',
        type=str,
        default=None,
        help='Output directory (default: package maps folder)'
    )
    parser.add_argument(
        '--database', '-d',
        type=str,
        default=os.path.expanduser('~/rtabmap.db'),
        help='Path to RTAB-Map database file'
    )
    parser.add_argument(
        '--resolution', '-r',
        type=float,
        default=0.05,
        help='Map resolution in meters (default: 0.05)'
    )
    
    args = parser.parse_args()
    
    # Determine output directory
    if args.output_dir:
        output_dir = args.output_dir
    else:
        # Try to find package maps directory
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('ros2_agv_navigation_stack')
            output_dir = os.path.join(pkg_share, 'maps')
        except Exception:
            output_dir = os.getcwd()
    
    os.makedirs(output_dir, exist_ok=True)
    
    map_path = os.path.join(output_dir, args.map_name)
    
    print(f"Exporting map from: {args.database}")
    print(f"Output files: {map_path}.pgm, {map_path}.yaml")
    print(f"Resolution: {args.resolution}m")
    
    # Check if database exists
    if not os.path.exists(args.database):
        print(f"Error: Database file not found: {args.database}")
        sys.exit(1)
    
    # Export using rtabmap-export
    try:
        # Method 1: Use rtabmap-export CLI tool
        cmd = [
            'rtabmap-export',
            '--poses',
            '--images',
            '--output', map_path,
            args.database
        ]
        
        print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"Warning: rtabmap-export failed, trying alternative method...")
            raise Exception(result.stderr)
            
    except Exception as e:
        print(f"Using Nav2 map_saver as fallback...")
        
        # Method 2: Use Nav2 map_saver (requires map to be published)
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_path,
            '--ros-args', '-p', 'use_sim_time:=true'
        ]
        
        print(f"Running: {' '.join(cmd)}")
        print("Note: This requires RTAB-Map to be publishing /map topic")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"Error: {result.stderr}")
            sys.exit(1)
    
    # Create/update YAML file with correct settings
    yaml_content = f"""image: {args.map_name}.pgm
resolution: {args.resolution}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    
    yaml_path = f"{map_path}.yaml"
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    
    print(f"\n✓ Map exported successfully!")
    print(f"  PGM file: {map_path}.pgm")
    print(f"  YAML file: {yaml_path}")
    print(f"\nTo use with Nav2:")
    print(f"  ros2 launch ros2_agv_navigation_stack bringup.launch.py \\")
    print(f"      slam:=false localization:=true \\")
    print(f"      database_path:={args.database}")


if __name__ == '__main__':
    main()
