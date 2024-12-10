#!/usr/bin/env python3
import pyusbex
import logging
from typing import List, Optional
from dataclasses import dataclass

@dataclass
class RealSenseDevice:
    """Data class to store RealSense device information"""
    product_id: str
    vendor_id: str
    serial_number: str
    port: str

class RealSenseDetector:
    """Class to detect RealSense devices connected via USB"""
    
    # RealSense product IDs
    DS5_PRODUCT_IDS = [
        "0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", 
        "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", 
        "0B3A", "0B5C", "0B5B"
    ]
    
    INTEL_VENDOR_ID = "8086"  # Intel vendor ID
    
    def __init__(self, max_devices: int = 4):
        """
        Initialize the detector
        
        Args:
            max_devices (int): Maximum number of devices to detect
        """
        self.max_devices = max_devices
        self.logger = logging.getLogger(__name__)
        
    def find_devices(self) -> List[RealSenseDevice]:
        """
        Find all connected RealSense devices
        
        Returns:
            List[RealSenseDevice]: List of detected RealSense devices
        """
        devices = []
        
        try:
            # Get all USB devices
            usb_devices = pyusbex.find_all_devices()
            
            for device in usb_devices:
                if (device.vid == self.INTEL_VENDOR_ID and 
                    device.pid.upper() in self.DS5_PRODUCT_IDS):
                    
                    realsense_device = RealSenseDevice(
                        product_id=device.pid,
                        vendor_id=device.vid,
                        serial_number=device.serial_number,
                        port=device.port
                    )
                    devices.append(realsense_device)
                    
                    if len(devices) >= self.max_devices:
                        break
                        
        except Exception as e:
            self.logger.error(f"Error detecting RealSense devices: {str(e)}")
            
        return devices

    def find_devices_advanced_mode(self) -> List[RealSenseDevice]:
        """
        Find RealSense devices that support advanced mode
        
        Returns:
            List[RealSenseDevice]: List of detected devices supporting advanced mode
        """
        devices = self.find_devices()
        advanced_devices = []
        
        for device in devices:
            try:
                # Try to enable advanced mode (implementation depends on realsense SDK)
                if self._check_advanced_mode_support(device):
                    advanced_devices.append(device)
            except Exception as e:
                self.logger.warning(
                    f"Error checking advanced mode for device {device.serial_number}: {str(e)}"
                )
                
        return advanced_devices
    
    def _check_advanced_mode_support(self, device: RealSenseDevice) -> bool:
        """
        Check if device supports advanced mode
        
        Args:
            device (RealSenseDevice): Device to check
            
        Returns:
            bool: True if device supports advanced mode
        """
        # Implementation depends on realsense SDK
        # This is a placeholder that should be implemented based on SDK requirements
        return True

# Example usage
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    detector = RealSenseDetector(max_devices=4)
    devices = detector.find_devices()
    
    for device in devices:
        print(f"Found device: {device}")