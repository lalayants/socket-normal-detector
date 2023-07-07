import json
import open3d as o3d
import cv2
import numpy as np

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

meta = rs.get_metadata()
intrinsics = o3d.camera.PinholeCameraIntrinsic(meta.width, meta.height, *meta.intrinsics.get_focal_length(), *meta.intrinsics.get_principal_point())
while True:
    im_rgbd = rs.capture_frame(True, True) # wait for frames and align them
    # print(rs.get_metadata().intrinsics.intrinsic_matrix, rs.get_metadata().depth_scale)

    # cv2.imwrite('1.jpg', np.array(im_rgbd.depth))
    pcd = o3d.geometry.PointCloud.create_from_depth_image(im_rgbd.depth.to_legacy(), rs.get_metadata().intrinsics)
    # pcd = o3d.geometry.PointCloud.create_from_depth_image(im_rgbd.depth, rs.get_metadata().intrinsics, extrinsic=extr, depth_scale=rs.get_metadata().depth_scale)
    # break
    hull = pcd.compute_convex_hull()
    o3d.visualization.draw([{'name': 'eagle', 'geometry': pcd}, {'name': 'convex hull', 'geometry': hull}])
    cv2.imwrite('1.jpg', np.array(im_rgbd.depth))

rs.stop_capture()