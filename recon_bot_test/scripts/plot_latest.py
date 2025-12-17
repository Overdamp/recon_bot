#!/usr/bin/env python3
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_latest():
    log_dir = os.path.expanduser('~/recon_bot_logs')
    list_of_files = glob.glob(os.path.join(log_dir, '*.csv'))
    
    if not list_of_files:
        print("No log files found in ~/recon_bot_logs")
        return

    # Get latest file
    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Plotting data from: {latest_file}")
    
    try:
        df = pd.read_csv(latest_file)
        
        plt.figure(figsize=(10, 6))
        
        # Plot Odom
        plt.plot(df['odom_x'].to_numpy(), df['odom_y'].to_numpy(), label='Odometry', color='blue', linestyle='--')
        plt.scatter(df['odom_x'].iloc[0], df['odom_y'].iloc[0], color='blue', marker='o', label='Odom Start')
        
        # Plot AprilTag
        # Handle boolean column that might be string 'True'/'False' or bool
        if df['tag_visible'].dtype == object:
            visible_tags = df[df['tag_visible'] == 'True']
        else:
            visible_tags = df[df['tag_visible'] == True]
            
        if not visible_tags.empty:
            plt.plot(visible_tags['apriltag_x'].to_numpy(), visible_tags['apriltag_y'].to_numpy(), label='AprilTag', color='green', marker='.', linestyle='-')
            plt.scatter(visible_tags['apriltag_x'].iloc[0], visible_tags['apriltag_y'].iloc[0], color='green', marker='x', label='Tag Start')
        else:
            print("Warning: No visible AprilTag data points found.")
            
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
    plot_latest()
