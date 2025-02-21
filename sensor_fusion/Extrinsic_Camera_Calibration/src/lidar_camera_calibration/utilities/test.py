import numpy as np

extrinsics = np.load('/home/dhruvagarwal/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities/extrinsics.npz')
print(extrinsics['R'])
print(extrinsics['T'])