import numpy as np
from open3d import * 
import pickle
# import open3d_tutorial as o3dtut

def main():
	pcd = open3d.geometry.PointCloud()
	with open('pcl.pickle','rb') as f:
		pc = pickle.load(f)
	for i in range(len(pc)):
		pc[i] = pc[i][0:3]
	pc = np.array(pc)


	print(pc[0])
	pcd.points = open3d.utility.Vector3dVector(pc)
	# cloud = open3d.io.read_point_cloud(pc) # Read the point cloud
	open3d.visualization.draw_geometries([pcd]) # Visualize the point cloud   





  

if __name__ == "__main__":
    main()

