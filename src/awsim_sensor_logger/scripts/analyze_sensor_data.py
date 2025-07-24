#!/usr/bin/env python3
"""
AWSIM Sensor Data Analysis Script

This script provides tools for analyzing the synchronized sensor data
collected by the AWSIM sensor logger.
"""

import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import argparse
from datetime import datetime
import json

class AWSimDataAnalyzer:
    def __init__(self, session_directory):
        """
        Initialize the data analyzer with a session directory.
        
        Args:
            session_directory (str): Path to the session data directory
        """
        self.session_dir = Path(session_directory)
        self.sync_data = None
        self.timing_data = None
        self.data_format = "unknown"
        
        # Verify session directory
        if not self.session_dir.exists():
            raise FileNotFoundError(f"Session directory not found: {session_directory}")
        
        # Load data files
        self._load_data()
    
    def _load_data(self):
        """Load CSV data files from the session directory."""
        try:
            sync_file = self.session_dir / "synchronized_data.csv"
            timing_file = self.session_dir / "timing_analysis.csv"
            
            if sync_file.exists():
                self.sync_data = pd.read_csv(sync_file)
                # Check if we have any data rows (more than just header)
                if len(self.sync_data) > 0:
                    print(f"Loaded {len(self.sync_data)} synchronized frames")
                    # Detect data format (old vs new)
                    self.data_format = self._detect_data_format()
                    print(f"Detected data format: {self.data_format}")
                else:
                    print("Warning: synchronized_data.csv is empty (header only)")
            else:
                print("Warning: synchronized_data.csv not found")
            
            if timing_file.exists():
                self.timing_data = pd.read_csv(timing_file)
                if len(self.timing_data) > 0:
                    print(f"Loaded timing analysis for {len(self.timing_data)} frames")
                else:
                    print("Warning: timing_analysis.csv is empty (header only)")
            else:
                print("Warning: timing_analysis.csv not found")
                
        except Exception as e:
            print(f"Error loading data: {e}")
    
    def _detect_data_format(self):
        """Detect data format: 4-sensor, 5-sensor (optimized), 7-sensor (with ground truth), or 9-sensor (with vehicle status)."""
        if self.sync_data is None or len(self.sync_data) == 0:
            return "unknown"
        
        columns = self.sync_data.columns.tolist()
        
        # Check for different format indicators
        camera_calibration = ['camera_fx', 'camera_fy', 'camera_cx', 'camera_cy']
        ground_truth_cols = ['gt_odom_x', 'gt_odom_y', 'gt_odom_z', 'gt_pose_x', 'gt_pose_y', 'gt_pose_z']
        vehicle_status_cols = ['velocity_longitudinal', 'gear_report']
        
        has_camera_calib_in_csv = all(col in columns for col in camera_calibration)
        has_ground_truth = any(col in columns for col in ground_truth_cols)
        has_vehicle_status = any(col in columns for col in vehicle_status_cols)
        
        # Check for optimized 5-sensor format (camera info saved separately)
        if not has_camera_calib_in_csv and has_ground_truth and not has_vehicle_status:
            # Check if we only have ground truth pose (not odom)
            if 'gt_pose_x' in columns and 'gt_odom_x' not in columns:
                return "optimized_5_sensor_with_ground_truth"
            else:
                return "core_7_sensor_with_ground_truth"
        elif has_camera_calib_in_csv and has_ground_truth and has_vehicle_status:
            return "enhanced_9_sensor"
        elif has_camera_calib_in_csv and has_ground_truth and not has_vehicle_status:
            return "core_7_sensor_with_ground_truth"
        elif has_camera_calib_in_csv and not has_ground_truth:
            return "core_5_sensor"
        else:
            return "original_4_sensor"
            
    def analyze_synchronization_quality(self):
        """Analyze the quality of sensor synchronization."""
        if self.timing_data is None or len(self.timing_data) == 0:
            print("\n=== SYNCHRONIZATION QUALITY ANALYSIS ===")
            print("No timing data available for analysis")
            print("This might indicate:")
            print("  - AWSIM was not running during data collection")
            print("  - Topics were not being published")
            print("  - Synchronization failed (no matching timestamps)")
            return None
        
        print("\n=== SYNCHRONIZATION QUALITY ANALYSIS ===")
        
        # Basic statistics
        max_diff = self.timing_data['max_time_diff_ms']
        print(f"Total synchronized frames: {len(self.timing_data)}")
        print(f"Average time difference: {max_diff.mean():.2f} ms")
        print(f"Median time difference: {max_diff.median():.2f} ms")
        print(f"Max time difference: {max_diff.max():.2f} ms")
        print(f"Min time difference: {max_diff.min():.2f} ms")
        print(f"Standard deviation: {max_diff.std():.2f} ms")
        
        # Quality categories
        excellent = (max_diff <= 20).sum()
        good = ((max_diff > 20) & (max_diff <= 50)).sum()
        acceptable = ((max_diff > 50) & (max_diff <= 100)).sum()
        poor = (max_diff > 100).sum()
        
        print(f"\nSynchronization Quality Distribution:")
        print(f"  Excellent (≤20ms): {excellent} frames ({excellent/len(max_diff)*100:.1f}%)")
        print(f"  Good (20-50ms): {good} frames ({good/len(max_diff)*100:.1f}%)")
        print(f"  Acceptable (50-100ms): {acceptable} frames ({acceptable/len(max_diff)*100:.1f}%)")
        print(f"  Poor (>100ms): {poor} frames ({poor/len(max_diff)*100:.1f}%)")
        
        return {
            'mean_diff_ms': max_diff.mean(),
            'median_diff_ms': max_diff.median(),
            'max_diff_ms': max_diff.max(),
            'std_diff_ms': max_diff.std(),
            'quality_distribution': {
                'excellent': excellent,
                'good': good,
                'acceptable': acceptable,
                'poor': poor
            }
        }
    
    def analyze_sensor_data(self):
        """Analyze the synchronized sensor data."""
        if self.sync_data is None or len(self.sync_data) == 0:
            print("\n=== SENSOR DATA ANALYSIS ===")
            print("No synchronized data available for analysis")
            print("This might indicate:")
            print("  - AWSIM was not running during data collection")
            print("  - Topics were not being published")
            print("  - Synchronization failed (no matching timestamps)")
            return None
        
        print("\n=== SENSOR DATA ANALYSIS ===")
        print(f"Data format: {self.data_format}")
        print(f"Total data frames: {len(self.sync_data)}")
        
        # LiDAR analysis
        if 'lidar_points' in self.sync_data.columns:
            lidar_points = self.sync_data['lidar_points']
            print(f"\nLiDAR Statistics:")
            print(f"  Average points per frame: {lidar_points.mean():.0f}")
            print(f"  Min points: {lidar_points.min()}")
            print(f"  Max points: {lidar_points.max()}")
        
        # IMU analysis
        imu_cols = ['imu_linear_x', 'imu_linear_y', 'imu_linear_z',
                   'imu_angular_x', 'imu_angular_y', 'imu_angular_z']
        
        if all(col in self.sync_data.columns for col in imu_cols):
            print(f"\nIMU Statistics:")
            for col in imu_cols:
                data = self.sync_data[col]
                print(f"  {col}: mean={data.mean():.3f}, std={data.std():.3f}")
        
        # GNSS analysis
        gnss_cols = ['gnss_x', 'gnss_y', 'gnss_z']
        if all(col in self.sync_data.columns for col in gnss_cols):
            print(f"\nGNSS Position Statistics:")
            for col in gnss_cols:
                data = self.sync_data[col]
                print(f"  {col}: mean={data.mean():.3f}, range=[{data.min():.3f}, {data.max():.3f}]")
        
        # Enhanced analysis for new data formats
        if self.data_format in ["enhanced_9_sensor", "core_7_sensor_with_ground_truth", "optimized_5_sensor_with_ground_truth"]:
            self._analyze_enhanced_data()
        
        # Calculate trajectory length
        if all(col in self.sync_data.columns for col in ['gnss_x', 'gnss_y']):
            distances = np.sqrt(np.diff(self.sync_data['gnss_x'])**2 + 
                              np.diff(self.sync_data['gnss_y'])**2)
            total_distance = distances.sum()
            print(f"\nTrajectory Analysis:")
            print(f"  Total distance traveled: {total_distance:.2f} meters")
            if len(self.sync_data) > 1:
                print(f"  Average speed: {total_distance / (len(self.sync_data) * 0.1):.2f} m/s")  # Assuming 10Hz
    
    def _analyze_enhanced_data(self):
        """Analyze additional data available in enhanced formats (5-sensor optimized, 7-sensor, or 9-sensor)."""
        print(f"\n=== ENHANCED DATA ANALYSIS ===")
        
        # Camera calibration analysis (check both CSV and separate file)
        camera_cols = ['camera_fx', 'camera_fy', 'camera_cx', 'camera_cy']
        if all(col in self.sync_data.columns for col in camera_cols):
            print(f"\nCamera Calibration (from CSV):")
            for col in camera_cols:
                value = self.sync_data[col].iloc[0]  # Should be constant
                print(f"  {col}: {value:.2f}")
        else:
            # Check for camera info file in optimized format
            camera_info_file = self.session_dir / "metadata" / "camera_info.json"
            if camera_info_file.exists():
                print(f"\nCamera Calibration (from metadata/camera_info.json):")
                try:
                    import json
                    with open(camera_info_file, 'r') as f:
                        camera_info = json.load(f)
                    for key, value in camera_info['camera_matrix'].items():
                        print(f"  {key}: {value:.2f}")
                    print(f"  Image size: {camera_info['image_width']}x{camera_info['image_height']}")
                except Exception as e:
                    print(f"  Error reading camera info: {e}")
        
        # Ground truth analysis (available in 5-sensor optimized, 7-sensor and 9-sensor formats)
        gt_odom_cols = ['gt_odom_x', 'gt_odom_y', 'gt_odom_z']
        if all(col in self.sync_data.columns for col in gt_odom_cols):
            print(f"\nGround Truth Odometry Statistics:")
            for col in gt_odom_cols:
                data = self.sync_data[col]
                print(f"  {col}: mean={data.mean():.3f}, range=[{data.min():.3f}, {data.max():.3f}]")
        
        gt_pose_cols = ['gt_pose_x', 'gt_pose_y', 'gt_pose_z']
        if all(col in self.sync_data.columns for col in gt_pose_cols):
            print(f"\nGround Truth Pose Statistics:")
            for col in gt_pose_cols:
                data = self.sync_data[col]
                print(f"  {col}: mean={data.mean():.3f}, range=[{data.min():.3f}, {data.max():.3f}]")
        
        # Compare ground truth vs GNSS
        if all(col in self.sync_data.columns for col in ['gt_pose_x', 'gt_pose_y', 'gnss_x', 'gnss_y']):
            gt_x_diff = self.sync_data['gt_pose_x'] - self.sync_data['gnss_x']
            gt_y_diff = self.sync_data['gt_pose_y'] - self.sync_data['gnss_y']
            gt_distance_diff = np.sqrt(gt_x_diff**2 + gt_y_diff**2)
            print(f"\nGround Truth vs GNSS Comparison:")
            print(f"  Average position difference: {gt_distance_diff.mean():.3f} m")
            print(f"  Max position difference: {gt_distance_diff.max():.3f} m")
        
        # Vehicle dynamics analysis (only available in 9-sensor format)
        velocity_cols = ['velocity_longitudinal', 'velocity_lateral', 'velocity_heading_rate']
        if all(col in self.sync_data.columns for col in velocity_cols):
            print(f"\nVehicle Dynamics:")
            for col in velocity_cols:
                data = self.sync_data[col]
                print(f"  {col}: mean={data.mean():.3f}, std={data.std():.3f}, max={data.max():.3f}")
        
        # Gear analysis (only available in 9-sensor format)
        if 'gear_report' in self.sync_data.columns:
            gear_data = self.sync_data['gear_report']
            unique_gears = gear_data.unique()
            print(f"\nGear Status:")
            print(f"  Gears used: {unique_gears}")
            for gear in unique_gears:
                count = (gear_data == gear).sum()
                print(f"  Gear {gear}: {count} frames ({count/len(gear_data)*100:.1f}%)")
    
    def plot_synchronization_quality(self, output_dir=None):
        """Create plots for synchronization quality analysis."""
        if self.timing_data is None or len(self.timing_data) == 0:
            print("No timing data available for plotting")
            return None
        
        if output_dir is None:
            output_dir = self.session_dir / "analysis_plots"
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True)
        
        # Set up the plotting style
        plt.style.use('seaborn-v0_8')
        
        # Plot 1: Time difference histogram
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 2, 1)
        plt.hist(self.timing_data['max_time_diff_ms'], bins=50, alpha=0.7, edgecolor='black')
        plt.xlabel('Maximum Time Difference (ms)')
        plt.ylabel('Frequency')
        plt.title('Synchronization Quality Distribution')
        plt.axvline(50, color='red', linestyle='--', alpha=0.7, label='Acceptable threshold')
        plt.legend()
        
        # Plot 2: Time difference over time
        plt.subplot(2, 2, 2)
        plt.plot(self.timing_data['sync_id'], self.timing_data['max_time_diff_ms'], alpha=0.7)
        plt.xlabel('Sync Frame ID')
        plt.ylabel('Max Time Difference (ms)')
        plt.title('Synchronization Quality Over Time')
        plt.axhline(50, color='red', linestyle='--', alpha=0.7)
        
        # Plot 3: Individual sensor timing
        plt.subplot(2, 2, 3)
        # Determine sensor list based on available columns
        all_sensors = ['lidar_timestamp_ns', 'camera_timestamp_ns', 'camera_info_timestamp_ns', 
                      'imu_timestamp_ns', 'gnss_timestamp_ns', 'gt_odom_timestamp_ns', 'gt_pose_timestamp_ns']
        available_sensors = [s for s in all_sensors if s in self.timing_data.columns]
        
        for sensor in available_sensors:
            # Convert to relative time (seconds from start)
            start_time = self.timing_data[sensor].iloc[0]
            rel_time = (self.timing_data[sensor] - start_time) / 1e9
            sensor_label = sensor.replace('_timestamp_ns', '').replace('gt_', 'GT_').upper()
            plt.plot(rel_time, label=sensor_label)
        
        plt.xlabel('Relative Time (seconds)')
        plt.ylabel('Message Number')
        plt.title('Individual Sensor Timing')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.grid(True, alpha=0.3)
        
        # Plot 4: Cumulative distribution
        plt.subplot(2, 2, 4)
        sorted_diffs = np.sort(self.timing_data['max_time_diff_ms'])
        cumulative = np.arange(1, len(sorted_diffs) + 1) / len(sorted_diffs)
        plt.plot(sorted_diffs, cumulative)
        plt.xlabel('Maximum Time Difference (ms)')
        plt.ylabel('Cumulative Probability')
        plt.title('Cumulative Distribution of Sync Quality')
        plt.axvline(50, color='red', linestyle='--', alpha=0.7)
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_dir / "synchronization_analysis.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Synchronization plots saved to: {output_dir / 'synchronization_analysis.png'}")
    
    def plot_sensor_data(self, output_dir=None):
        """Create plots for sensor data analysis."""
        if self.sync_data is None or len(self.sync_data) == 0:
            print("No synchronized data available for plotting")
            return None
        
        if output_dir is None:
            output_dir = self.session_dir / "analysis_plots"
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True)
        
        # Create sensor data plots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: GNSS trajectory
        if all(col in self.sync_data.columns for col in ['gnss_x', 'gnss_y']):
            axes[0, 0].plot(self.sync_data['gnss_x'], self.sync_data['gnss_y'], 'b-', alpha=0.7)
            axes[0, 0].scatter(self.sync_data['gnss_x'].iloc[0], self.sync_data['gnss_y'].iloc[0], 
                              color='green', s=100, label='Start', marker='o')
            axes[0, 0].scatter(self.sync_data['gnss_x'].iloc[-1], self.sync_data['gnss_y'].iloc[-1], 
                              color='red', s=100, label='End', marker='s')
            axes[0, 0].set_xlabel('X Position (m)')
            axes[0, 0].set_ylabel('Y Position (m)')
            axes[0, 0].set_title('GNSS Trajectory')
            axes[0, 0].legend()
            axes[0, 0].grid(True, alpha=0.3)
            axes[0, 0].axis('equal')
        
        # Plot 2: LiDAR point count over time
        if 'lidar_points' in self.sync_data.columns:
            axes[0, 1].plot(self.sync_data['frame_id'], self.sync_data['lidar_points'])
            axes[0, 1].set_xlabel('Frame ID')
            axes[0, 1].set_ylabel('Point Count')
            axes[0, 1].set_title('LiDAR Point Count Over Time')
            axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: IMU acceleration
        imu_linear_cols = ['imu_linear_x', 'imu_linear_y', 'imu_linear_z']
        if all(col in self.sync_data.columns for col in imu_linear_cols):
            for i, col in enumerate(imu_linear_cols):
                axes[1, 0].plot(self.sync_data['frame_id'], self.sync_data[col], 
                               label=col.replace('imu_linear_', '').upper())
            axes[1, 0].set_xlabel('Frame ID')
            axes[1, 0].set_ylabel('Linear Acceleration (m/s²)')
            axes[1, 0].set_title('IMU Linear Acceleration')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: IMU angular velocity
        imu_angular_cols = ['imu_angular_x', 'imu_angular_y', 'imu_angular_z']
        if all(col in self.sync_data.columns for col in imu_angular_cols):
            for i, col in enumerate(imu_angular_cols):
                axes[1, 1].plot(self.sync_data['frame_id'], self.sync_data[col], 
                               label=col.replace('imu_angular_', '').upper())
            axes[1, 1].set_xlabel('Frame ID')
            axes[1, 1].set_ylabel('Angular Velocity (rad/s)')
            axes[1, 1].set_title('IMU Angular Velocity')
            axes[1, 1].legend()
            axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_dir / "sensor_data_analysis.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Sensor data plots saved to: {output_dir / 'sensor_data_analysis.png'}")
        
        # Create ground truth comparison plot if available
        if self.data_format in ["enhanced_9_sensor", "core_7_sensor_with_ground_truth", "optimized_5_sensor_with_ground_truth"]:
            self.plot_ground_truth_analysis(output_dir)
    
    def plot_ground_truth_analysis(self, output_dir=None):
        """Create plots comparing ground truth vs GNSS data."""
        if self.sync_data is None or len(self.sync_data) == 0:
            return None
        
        if output_dir is None:
            output_dir = self.session_dir / "analysis_plots"
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True)
        
        # Check if we have ground truth data
        gt_cols = ['gt_pose_x', 'gt_pose_y', 'gnss_x', 'gnss_y']
        if not all(col in self.sync_data.columns for col in gt_cols):
            return None
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: Ground truth vs GNSS trajectory comparison
        axes[0, 0].plot(self.sync_data['gnss_x'], self.sync_data['gnss_y'], 
                       'b-', alpha=0.7, label='GNSS', linewidth=2)
        axes[0, 0].plot(self.sync_data['gt_pose_x'], self.sync_data['gt_pose_y'], 
                       'r--', alpha=0.7, label='Ground Truth', linewidth=2)
        axes[0, 0].scatter(self.sync_data['gnss_x'].iloc[0], self.sync_data['gnss_y'].iloc[0], 
                          color='blue', s=100, marker='o', label='GNSS Start')
        axes[0, 0].scatter(self.sync_data['gt_pose_x'].iloc[0], self.sync_data['gt_pose_y'].iloc[0], 
                          color='red', s=100, marker='s', label='GT Start')
        axes[0, 0].set_xlabel('X Position (m)')
        axes[0, 0].set_ylabel('Y Position (m)')
        axes[0, 0].set_title('Ground Truth vs GNSS Trajectory Comparison')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].axis('equal')
        
        # Plot 2: Position difference over time
        gt_x_diff = self.sync_data['gt_pose_x'] - self.sync_data['gnss_x']
        gt_y_diff = self.sync_data['gt_pose_y'] - self.sync_data['gnss_y']
        gt_distance_diff = np.sqrt(gt_x_diff**2 + gt_y_diff**2)
        
        axes[0, 1].plot(self.sync_data['frame_id'], gt_distance_diff, 'g-', linewidth=2)
        axes[0, 1].set_xlabel('Frame ID')
        axes[0, 1].set_ylabel('Position Difference (m)')
        axes[0, 1].set_title('Ground Truth vs GNSS Distance Difference')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: X and Y differences
        axes[1, 0].plot(self.sync_data['frame_id'], gt_x_diff, 'r-', label='X Difference', linewidth=2)
        axes[1, 0].plot(self.sync_data['frame_id'], gt_y_diff, 'b-', label='Y Difference', linewidth=2)
        axes[1, 0].set_xlabel('Frame ID')
        axes[1, 0].set_ylabel('Position Difference (m)')
        axes[1, 0].set_title('Ground Truth vs GNSS X/Y Differences')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: Difference distribution histogram
        axes[1, 1].hist(gt_distance_diff, bins=30, alpha=0.7, edgecolor='black')
        axes[1, 1].axvline(gt_distance_diff.mean(), color='red', linestyle='--', 
                          label=f'Mean: {gt_distance_diff.mean():.3f}m')
        axes[1, 1].set_xlabel('Position Difference (m)')
        axes[1, 1].set_ylabel('Frequency')
        axes[1, 1].set_title('Ground Truth vs GNSS Difference Distribution')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_dir / "ground_truth_analysis.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Ground truth analysis plots saved to: {output_dir / 'ground_truth_analysis.png'}")
    
    def generate_report(self, output_file=None):
        """Generate a comprehensive analysis report."""
        if output_file is None:
            output_file = self.session_dir / "analysis_report.json"
        
        report = {
            'session_directory': str(self.session_dir),
            'analysis_timestamp': datetime.now().isoformat(),
            'data_files': {
                'synchronized_data': self.sync_data is not None,
                'timing_data': self.timing_data is not None
            }
        }
        
        # Add synchronization analysis
        sync_analysis = self.analyze_synchronization_quality()
        if sync_analysis:
            report['synchronization_analysis'] = sync_analysis
        
        # Add basic data statistics
        if self.sync_data is not None:
            report['data_statistics'] = {
                'total_frames': len(self.sync_data),
                'data_columns': list(self.sync_data.columns)
            }
        
        # Save report
        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"Analysis report saved to: {output_file}")
        return report


def main():
    parser = argparse.ArgumentParser(description='Analyze AWSIM sensor data')
    parser.add_argument('session_directory', help='Path to the session data directory')
    parser.add_argument('--plots', action='store_true', help='Generate analysis plots')
    parser.add_argument('--report', action='store_true', help='Generate analysis report')
    parser.add_argument('--output-dir', help='Output directory for plots and reports')
    
    args = parser.parse_args()
    
    try:
        # Initialize analyzer
        analyzer = AWSimDataAnalyzer(args.session_directory)
        
        # Perform basic analysis
        analyzer.analyze_synchronization_quality()
        analyzer.analyze_sensor_data()
        
        # Generate plots if requested
        if args.plots:
            analyzer.plot_synchronization_quality(args.output_dir)
            analyzer.plot_sensor_data(args.output_dir)
        
        # Generate report if requested
        if args.report:
            output_file = None
            if args.output_dir:
                output_file = Path(args.output_dir) / "analysis_report.json"
            analyzer.generate_report(output_file)
        
    except Exception as e:
        print(f"Error during analysis: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
