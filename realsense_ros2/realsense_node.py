#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from cv_bridge import CvBridge
from datetime import datetime
from typing import Dict, Optional
import yaml

class RealSenseNode(Node):
    """ROS2 Node for RealSense camera"""
    
    def __init__(self, serial_number: str, config_path: str = "config/Advanced_Mode.yaml"):
        """
        Initialize the RealSense node
        
        Args:
            serial_number (str): Camera serial number
            config_path (str): Path to advanced mode configuration
        """
        super().__init__(f'realsense_node_{serial_number}')
        
        self.serial_number = serial_number
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams
        self.config.enable_device(serial_number)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        
        # Create publishers
        self.color_pub = self.create_publisher(Image, f'camera/{serial_number}/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, f'camera/{serial_number}/depth/image_raw', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, f'camera/{serial_number}/pointcloud', 10)
        
        # Set up data storage paths
        self.setup_storage_paths()
        
        # Load advanced mode configuration
        self.load_advanced_config(config_path)
        
        # Create timer for frame callback
        self.create_timer(1.0/30, self.frame_callback)
        
        self.get_logger().info(f"RealSense node initialized for device {serial_number}")
    
    def setup_storage_paths(self):
        """Set up storage paths for different data types"""
        base_path = "sampled_data"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        self.storage_paths = {
            'rgb': os.path.join(base_path, 'rgb', timestamp),
            'depth': os.path.join(base_path, 'depth', timestamp),
            'pointcloud': os.path.join(base_path, 'pointcloud', timestamp)
        }
        
        for path in self.storage_paths.values():
            os.makedirs(path, exist_ok=True)
    
    def load_advanced_config(self, config_path: str):
        """
        Load advanced mode configuration from YAML file
        
        Args:
            config_path (str): Path to configuration file
        """
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                
            # Get device to configure advanced mode
            device = self.pipeline.get_active_profile().get_device()
            advanced_mode = rs.rs400_advanced_mode(device)
            
            if not advanced_mode.is_enabled():
                advanced_mode.toggle_advanced_mode(True)
                
            # Apply configurations
            for key, value in config.items():
                if hasattr(advanced_mode, f"set_{key}"):
                    getattr(advanced_mode, f"set_{key}")(value)
                    
        except Exception as e:
            self.get_logger().warning(f"Failed to load advanced config: {str(e)}")
    
    def frame_callback(self):
        """Process and publish camera frames"""
        frames = self.pipeline.wait_for_frames()
        
        # Process color frame
        color_frame = frames.get_color_frame()
        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            self.process_color(color_image)
            
        # Process depth frame
        depth_frame = frames.get_depth_frame()
        if depth_frame:
            depth_image = np.asanyarray(depth_frame.get_data())
            self.process_depth(depth_image)
            
        # Generate and process pointcloud
        if color_frame and depth_frame:
            self.process_pointcloud(color_frame, depth_frame)
    
    def process_color(self, color_image: np.ndarray):
        """Process and save color frame"""
        # Publish to ROS topic
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.color_pub.publish(msg)
        
        # Save to file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        cv2.imwrite(os.path.join(self.storage_paths['rgb'], f"{timestamp}.png"), color_image)
    
    def process_depth(self, depth_image: np.ndarray):
        """Process and save depth frame"""
        # Publish to ROS topic
        msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        self.depth_pub.publish(msg)
        
        # Save to file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        cv2.imwrite(os.path.join(self.storage_paths['depth'], f"{timestamp}.png"), depth_image)
    
    def process_pointcloud(self, color_frame, depth_frame):
        """Generate and save pointcloud"""
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)
        
        # Convert to numpy array
        vertices = np.asarray(points.get_vertices())
        
        # Save to file (as PLY)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        points.export_to_ply(
            os.path.join(self.storage_paths['pointcloud'], f"{timestamp}.ply"),
            color_frame
        )
        
        # Publish to ROS topic (implementation depends on specific requirements)
        # This is a simplified version
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f"camera_{self.serial_number}_optical_frame"
        
        # Create and publish PointCloud2 message (implementation needed)
        # This is a placeholder for the actual implementation
        # msg = create_point_cloud2(vertices, header)
        # self.pointcloud_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Get serial numbers from find_device.py
    from find_device import RealSenseDetector
    detector = RealSenseDetector()
    devices = detector.find_devices()
    
    nodes = []
    
    # Create a node for each detected device
    for device in devices:
        node = RealSenseNode(device.serial_number)
        nodes.append(node)
    
    # Spin all nodes
    try:
        rclpy.spin_once(nodes[0])  # Only need to spin one node
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()