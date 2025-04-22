import open3d as o3d
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
pcd_path = os.path.join(script_dir, "pointcloud.pcd")

pcd = o3d.io.read_point_cloud(pcd_path)

plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
                                         ransac_n=3,
                                         num_iterations=1000)

[a, b, c, d] = plane_model
print(f"Plane equation: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
