import json
import open3d as o3d
import cv2
import numpy as np
import copy

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])
    
    
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    # source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
    # target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

config_filename = 'open3d_test_py/cfg.json'
with open(config_filename) as cf:
    rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf))
extr = np.array([
    [1., 0., 0., 0.],
    [0., 1., 0., 0.],
    [0., 0., 1., 0.],
    [0., 0., 0., 1.]])
rs = o3d.t.io.RealSenseSensor()
rs.init_sensor(rs_cfg, 0)
rs.start_capture(True)  # true: start recording with capture
# print(o3d.t.io.RealSenseSensor.list_devices())
meta = rs.get_metadata()
intrinsics = o3d.camera.PinholeCameraIntrinsic(meta.width, meta.height, *meta.intrinsics.get_focal_length(), *meta.intrinsics.get_principal_point())
# target = o3d.io.read_point_cloud('models/plug_50000.pcd').scale(scale=0.001, center=[0,0,0])
# o3d.io.write_point_cloud("plug_50000_mini.pcd", target)
target = o3d.io.read_point_cloud('rec_code/milk.pcd')

print(target.get_max_bound()-target.get_min_bound())
while True:
    im_rgbd = rs.capture_frame(True, True) # wait for frames and align them
    # print(rs.get_metadata().intrinsics.intrinsic_matrix, rs.get_metadata().depth_scale)
    # source = o3d.geometry.PointCloud.create_from_depth_image(im_rgbd.depth.to_legacy(), rs.get_metadata().intrinsics, depth_scale=rs.get_metadata().depth_scale, extrinsic=extr)
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(im_rgbd.color.to_legacy(), im_rgbd.depth.to_legacy())
    # source = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, rs.get_metadata().intrinsics, extrinsic=extr)
    # print(source.get_max_bound()-source.get_min_bound())
    source = o3d.io.read_point_cloud('rec_code/milk_cartoon_all_small_clorox.pcd')
    
    # cv2.imwrite('1.jpg', np.array(im_rgbd.depth))
    # voxel_size = 0.01  # means 5cm for this dataset
    # source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
    #     voxel_size)
    # result_ransac = execute_global_registration(source_down, target_down,
    #                                         source_fpfh, target_fpfh,
    #                                         voxel_size)
    # draw_registration_result(source_down, target_down, result_ransac.transformation)
    # o3d.io.write_point_cloud("copy_of_fragment.pcd", source)
    print(source)
    source.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([source])
                                #       zoom=0.3412,
                                #       front=[0, 0, 1],
                                #   lookat=[0, 0, 0],
                                #   up=[0, -1, 0])

rs.stop_capture()