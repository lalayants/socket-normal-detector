import open3d as o3d

input = o3d.io.read_triangle_mesh("models/plug.stl")
new_pcd = o3d.geometry.sample_points_uniformly(input, number_of_points=100)
o3d.io.write_point_cloud("copy_of_fragment.pcd", new_pcd)