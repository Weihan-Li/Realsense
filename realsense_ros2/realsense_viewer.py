#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d
import threading
import queue
from typing import Dict, Optional
import pyrealsense2 as rs

class RealSenseViewer(Node):
    """Real-time viewer for RealSense camera data"""
    
    def __init__(self):
        """Initialize the viewer node"""
        super().__init__('realsense_viewer')
        
        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=5)
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("PointCloud Viewer", width=800, height=600)
        
        # Store the latest frames
        self.latest_frames: Dict[str, Optional[np.ndarray]] = {
            'color': None,
            'depth': None,
            'pointcloud': None
        }
        
        # Create subscribers for each camera
        self.create_camera_subscribers()
        
        # Start visualization thread
        self.vis_thread = threading.Thread(target=self.visualization_loop)
        self.vis_thread.daemon = True
        self.vis_thread.start()
        
        self.get_logger().info("RealSense viewer initialized")
    
    def create_camera_subscribers(self):
        """Create subscribers for all connected cameras"""
        from find_device import RealSenseDetector
        
        detector = RealSenseDetector()
        devices = detector.find_devices()
        
        for device in devices:
            serial = device.serial_number
            # Color image subscriber
            self.create_subscription(
                Image,
                f'camera/{serial}/color/image_raw',
                lambda msg, s=serial: self.color_callback(msg, s),
                10
            )
            
            # Depth image subscriber
            self.create_subscription(
                Image,
                f'camera/{serial}/depth/image_raw',
                lambda msg, s=serial: self.depth_callback(msg, s),
                10
            )
            
            # PointCloud subscriber
            self.create_subscription(
                PointCloud2,
                f'camera/{serial}/pointcloud',
                lambda msg, s=serial: self.pointcloud_callback(msg, s),
                10
            )
    
    def color_callback(self, msg: Image, serial: str):
        """Process color image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_frames['color'] = cv_image
            self.update_display()
        except Exception as e:
            self.get_logger().error(f'Error processing color image: {str(e)}')
    
    def depth_callback(self, msg: Image, serial: str):
        """Process depth image"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            
            # Normalize depth for display
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            self.latest_frames['depth'] = depth_colormap
            self.update_display()
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def pointcloud_callback(self, msg: PointCloud2, serial: str):
        """Process pointcloud data"""
        try:
            # Convert PointCloud2 to numpy array
            points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
            
            # Add to frame queue for 3D visualization
            if not self.frame_queue.full():
                self.frame_queue.put(points)
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {str(e)}')
    
    def update_display(self):
        """Update 2D display with latest frames"""
        if self.latest_frames['color'] is not None and self.latest_frames['depth'] is not None:
            # Stack color and depth images horizontally
            display_image = np.hstack((
                self.latest_frames['color'],
                self.latest_frames['depth']
            ))
            
            # Add text labels
            cv2.putText(
                display_image,
                "Color",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2
            )
            cv2.putText(
                display_image,
                "Depth",
                (self.latest_frames['color'].shape[1] + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2
            )
            
            cv2.imshow("RealSense Viewer", display_image)
            cv2.waitKey(1)
    
    def visualization_loop(self):
        """3D visualization loop for pointcloud"""
        pcd = o3d.geometry.PointCloud()
        
        while rclpy.ok():
            try:
                # Get latest pointcloud data
                if not self.frame_queue.empty():
                    points = self.frame_queue.get()
                    pcd.points = o3d.utility.Vector3dVector(points)
                    
                    # Update visualizer
                    self.vis.clear_geometries()
                    self.vis.add_geometry(pcd)
                    self.vis.poll_events()
                    self.vis.update_renderer()
                    
            except Exception as e:
                self.get_logger().error(f'Error in visualization loop: {str(e)}')
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        cv2.destroyAllWindows()
        self.vis.destroy_window()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    viewer = RealSenseViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()