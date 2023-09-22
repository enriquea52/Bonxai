import open3d as o3d
import numpy as np
import argparse

parser = argparse.ArgumentParser(
                    prog='PCD Visualizer',
                    description='A Simple  Script to Visualize a Pointcloud Stored in a PCD File',
                    epilog='')

parser.add_argument('filename') # positional argument
args = parser.parse_args()


print("Load a ply point cloud, print it, and render it")
ply_point_cloud = args.filename
pcd = o3d.io.read_point_cloud(ply_point_cloud)
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
