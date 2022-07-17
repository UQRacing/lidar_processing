import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import time
import os
start = time.time()
# load test data
pcdoc = o3d.io.read_point_cloud("test_data/1.pcd")
# load crop box
box = o3d.visualization.read_selection_polygon_volume("scripts/crop.json")

#crop data 
pcd = box.crop_point_cloud(pcdoc)

# cluster data and color it
labels = np.array(pcd.cluster_dbscan(eps=0.06, min_points=5, print_progress=True))
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
print(time.time() -start)

o3d.visualization.draw_geometries([pcd])