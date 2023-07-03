import open3d as o3d
path = 'plug.stl'
number_of_points = 100

input = o3d.io.read_triangle_mesh(path)
path = path.replace('.stl', '')
new_pcd = o3d.geometry.TriangleMesh.sample_points_uniformly(input, number_of_points=number_of_points)
o3d.io.write_point_cloud(f"{path}_{number_of_points}.pcd", new_pcd)