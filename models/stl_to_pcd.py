import open3d as o3d
path = 'models/plug.stl'
number_of_points = 100

input = o3d.io.read_triangle_mesh(path)
path = path.replace('.stl', '')
for number_of_points in [10000, 15000, 25000, 50000, 100000]:
    new_pcd = o3d.geometry.TriangleMesh.sample_points_uniformly(input, number_of_points=number_of_points).scale(scale=0.001, center=[0,0,0])
    print(new_pcd, new_pcd.get_max_bound()-new_pcd.get_min_bound())
    o3d.io.write_point_cloud(f"{path}_{number_of_points}.pcd", new_pcd)