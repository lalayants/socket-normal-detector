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
socket_pcd = o3d.io.read_point_cloud('models/plug_50000.pcd')
while True:
    im_rgbd = rs.capture_frame(True, True) # wait for frames and align them
    # print(rs.get_metadata().intrinsics.intrinsic_matrix, rs.get_metadata().depth_scale)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(im_rgbd.depth.to_legacy(), rs.get_metadata().intrinsics)
    cv2.imwrite('1.jpg', np.array(im_rgbd.depth))
    o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)
    o3d.visualization.draw_geometries([pcd],
                                      zoom=0.3412,
                                      front=[0, 0, 1],
                                  lookat=[0, 0, 0],
                                  up=[0, -1, 0])

rs.stop_capture()