import os
import cv2
import numpy as np
import ast
class FileOperations:
    
    def get_intrinsic_parameters(UTILITIES_PATH, realsenseCamera = False):
        file_path = '/home/daniel/intrinsic_parameters.txt' 
        if realsenseCamera:
            file_path = '/home/daniel/realsense_intrinsic_parameters.txt'
        with open(os.path.join(UTILITIES_PATH, file_path), 'r') as file:
            lines = file.readlines()
        heightWidth = lines[0].strip().split(', ')
        return float(heightWidth[0]), float(heightWidth[1]), [float(val) for val in lines[1].strip().split(', ')], [float(val) for val in lines[2].strip().split(', ')], [float(val) for val in lines[3].strip().split(', ')]
    
    def get_lidar_extrinsic_calibration_data(UTILITIES_PATH):
        points = []
        with open(os.path.join(UTILITIES_PATH, 'coordinates_3d.txt'), 'r') as file:
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
        with open(os.path.join(UTILITIES_PATH, '/home/daniel/coordinates_2d.txt'), 'r') as file:
            content = file.read().strip()
            data = ast.literal_eval(content)
            return np.array(data, dtype=np.float32)
    
    def get_extrinsic_parameters(UTILITIES_PATH):
        data = np.load(os.path.join(UTILITIES_PATH, '/home/daniel/extrinsics.npz'))
        return data['R'], data['T']
