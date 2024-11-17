import os
import cv2
import numpy as np
import ast
import re
class FileOperations:

    def get_intrinsic_parameters(UTILITIES_PATH, realsenseCamera):
        file_path = '/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities/logitech_intrinsic_parameters.txt' 
        if realsenseCamera:
            file_path = '/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities/realsense_intrinsic_parameters.txt'
        with open(os.path.join(UTILITIES_PATH, file_path), 'r') as file:
            lines = file.readlines()
        heightWidth = lines[0].strip().split(', ')
        return float(heightWidth[0]), float(heightWidth[1]), [float(val) for val in lines[1].strip().split(', ')], [float(val) for val in lines[2].strip().split(', ')], [float(val) for val in lines[3].strip().split(', ')]
    
    def get_lidar_extrinsic_calibration_data(UTILITIES_PATH):
        points = []
        with open('/home/dhruvagarwal/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities/coordinates_3d.txt', 'r') as file:
            lines = file.readlines()
            x, y, z = None, None, None
            for line in lines:
                line = line.strip()
                if line.startswith('x:'):
                    x = float(line.split(': ')[1])
                elif line.startswith('y:'):
                    y = float(line.split(': ')[1])
                elif line.startswith('z:'):
                    z = float(line.split(': ')[1])
                    points.append([x, y, z])
            return np.array(points, dtype=np.float32)
    
    def get_camera_extrinsic_calibration_data(UTILITIES_PATH):
        points = []
        file_path = "/home/dhruvagarwal/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities/coordinates_2d.txt"
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                # Extract the numbers after "IMG: (" and before the closing parenthesis ")"
                match = re.search(r'IMG:\s*\(([^,]+),\s*([^)]+)\)', line)
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    points.append([x, y])
        return np.array(points, dtype=np.float32)
    
    def get_extrinsic_parameters(UTILITIES_PATH, realsense = True):
        if realsense:
            data = np.load('/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities/extrinsics.npz')
        else:
            data = np.load('/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities/logitech_extrinsics.npz')
        return data['R'], data['T']