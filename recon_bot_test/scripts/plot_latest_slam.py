#!/usr/bin/env python3
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_latest_slam():
    log_dir = os.path.expanduser('~/recon_bot_logs')
    # Look for slam_test_data_*.csv
    list_of_files = glob.glob(os.path.join(log_dir, 'slam_test_data_*.csv'))
    
    if not list_of_files:
        print("No SLAM log files found in ~/recon_bot_logs")
        return

    # Get latest file
    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Plotting data from: {latest_file}")
    
    try:
        df = pd.read_csv(latest_file)
        
        plt.figure(figsize=(10, 6))
        
        # Normalize Odom to start at (0,0)
        odom_x = df['odom_x'].to_numpy()
        odom_y = df['odom_y'].to_numpy()
        odom_x -= odom_x[0]
        odom_y -= odom_y[0]
        
        plt.plot(odom_x, odom_y, label='Odometry (Relative)', color='blue', linestyle='--')
        plt.scatter(0, 0, color='blue', marker='o', label='Start')
        
        # Plot SLAM
        if df['slam_valid'].dtype == object:
            valid_slam = df[df['slam_valid'] == 'True']
        else:
            valid_slam = df[df['slam_valid'] == True]
            
        if not valid_slam.empty:
            slam_x = valid_slam['slam_x'].to_numpy()
            slam_y = valid_slam['slam_y'].to_numpy()
            
            # Normalize to start at (0,0)
            slam_x -= slam_x[0]
            slam_y -= slam_y[0]
            
            plt.plot(slam_x, slam_y, label='SLAM (Ground Truth)', color='green', marker='.', linestyle='-')
            plt.scatter(slam_x[0], slam_y[0], color='green', marker='x', label='SLAM Start')
        else:
            print("Warning: No valid SLAM data found.")
            
        plt.title(f'Path: {os.path.basename(latest_file)}')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        plot_filename = latest_file.replace('.csv', '.png')
        plt.savefig(plot_filename)
        print(f"Plot saved to {plot_filename}")
        plt.show()
        
    except Exception as e:
        print(f"Error plotting: {e}")

if __name__ == "__main__":
    plot_latest_slam()
