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
        
        # Normalize Odom to start at (0,0)
        odom_x = df['odom_x'].to_numpy()
        odom_y = df['odom_y'].to_numpy()
        odom_x -= odom_x[0]
        odom_y -= odom_y[0]
        
        # Rotate Odom by 5 degrees (User Request)
        import numpy as np
        theta = np.radians(5) # 5 degrees
        c, s = np.cos(theta), np.sin(theta)
        # Rotation Matrix: [x'] = [cos -sin] [x]
        #                  [y']   [sin  cos] [y]
        odom_x_rot = odom_x * c - odom_y * s
        odom_y_rot = odom_x * s + odom_y * c
        
        plt.plot(odom_x_rot, odom_y_rot, label='Odometry (Rotated 5°)', color='blue', linestyle='--')
        plt.scatter(0, 0, color='blue', marker='o', label='Start')
        
        # Plot AprilTag
        # Handle boolean column that might be string 'True'/'False' or bool
        if df['tag_visible'].dtype == object:
            visible_tags = df[df['tag_visible'] == 'True']
        else:
            visible_tags = df[df['tag_visible'] == True]
            
        if not visible_tags.empty:
            # Check variance to decide axes
            x_std = visible_tags['apriltag_x'].std()
            y_std = visible_tags['apriltag_y'].std()
            z_std = visible_tags['apriltag_z'].std()
            
            tag_x_raw = visible_tags['apriltag_x'].to_numpy()
            
            if z_std > y_std:
                print(f"Detected vertical tag. Using X-Z.")
                tag_y_raw = visible_tags['apriltag_z'].to_numpy()
            else:
                print(f"Detected horizontal tag. Using X-Y.")
                tag_y_raw = visible_tags['apriltag_y'].to_numpy()

            # Normalize start to 0,0
            tag_x_raw -= tag_x_raw[0]
            tag_y_raw -= tag_y_raw[0]

            # Apply Transformation: Rotate 90 deg then Flip Y (Reflection)
            # Based on user feedback: "90 deg is almost right, just need to fold up"
            # 90 deg was: x = -tag_y_raw, y = tag_x_raw
            # Flip Y means: y = -tag_x_raw
            # Final: x = -tag_y_raw, y = -tag_x_raw
            
            tag_x_final = -tag_y_raw
            tag_y_final = -tag_x_raw

            # Plot Raw Data (Faint)
            plt.plot(tag_x_final, tag_y_final, label='AprilTag (Raw)', color='green', marker='.', linestyle='-', alpha=0.3)
            
            # Apply Moving Average Smoothing
            window_size = 5
            if len(tag_x_final) > window_size:
                tag_x_smooth = pd.Series(tag_x_final).rolling(window=window_size, center=True).mean().to_numpy()
                tag_y_smooth = pd.Series(tag_y_final).rolling(window=window_size, center=True).mean().to_numpy()
                plt.plot(tag_x_smooth, tag_y_smooth, label=f'AprilTag (Smoothed {window_size})', color='darkgreen', linewidth=2)
            
            plt.scatter(tag_x_final[0], tag_y_final[0], color='green', marker='x', label='Tag Start')
            
            plt.title('Robot Path: Odometry vs AprilTag (Aligned)')
            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            
            # Save plot
            # Save plot
            plot_filename = latest_file.replace('.csv', '.png')
            plt.savefig(plot_filename)
            print(f"Plot saved to {plot_filename}")
            
            # Show plot
            plt.show()
        else:
            print("Warning: No visible AprilTag data points found.")
        
    except Exception as e:
        print(f"Error plotting: {e}")

if __name__ == "__main__":
    plot_latest()
