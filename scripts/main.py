import copy
import ctypes
import struct

import numpy as np
import open3d as o3d
import probreg
import rospy
import sensor_msgs.point_cloud2 as pc2
import small_gicp
from cube_experimental_python_api import CubeExperimentalCommander as Soar
from cube_python_api.proxies import ProxyTransformListener
from cube_python_api.utils import message_conversion  # noqa: F401
from moveit_msgs.msg import CollisionObject
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tf2_sensor_msgs import do_transform_cloud


def get_current_o3d() -> o3d.geometry.PointCloud:
    msg = rospy.wait_for_message('/soar/head_camera/depth_registered/points_voxel_filtered', PointCloud2)
    transform = tf2.lookup_transform('map', msg.header.frame_id)
    new_msg = do_transform_cloud(msg, transform)
    new_msg2 = filter_points(new_msg, 0.05)
    new_msg3 = pc2.create_cloud_xyz32(new_msg.header.frame_id, list(new_msg2))
    return pointcloud2_to_o3d(new_msg3).paint_uniform_color([1, 0, 0])


def create_o3d_box(co: CollisionObject, points=1000) -> o3d.geometry.PointCloud:
    size = co.primitives[0].dimensions
    mesh = o3d.geometry.TriangleMesh.create_box(*size)
    mesh.translate([-size[0] / 2, -size[1] / 2, -size[2] / 2], relative=True)
    pcd = mesh.sample_points_poisson_disk(number_of_points=points)
    pcd.transform(co.pose.mat())
    return pcd.paint_uniform_color([0, 1, 0])


def create_o3d_mesh(co: CollisionObject, filename: str, points=1000) -> o3d.geometry.PointCloud:
    mesh = o3d.io.read_triangle_mesh(filename)
    pcd = mesh.sample_points_poisson_disk(number_of_points=points)
    pcd.transform(co.pose.mat())
    return pcd.paint_uniform_color([0, 1, 0])


def filter_points(pcl_msg: PointCloud2, z_threshold=0.2) -> pc2.PointCloud2:
    """
    Filter points based on the z-value threshold.

    :param pcl_msg: PointCloud2 message
    :param z_threshold: float, points with z less than this value will be removed
    :return: generator of filtered points
    """
    for p in pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True):
        if p[2] >= z_threshold:
            yield p


def pointcloud2_to_o3d(pcl_msg: PointCloud2) -> o3d.geometry.PointCloud:
    """
    Convert sensor_msgs/PointCloud2 to open3d.geometry.PointCloud

    :param pcl_msg: sensor_msgs/PointCloud2 message
    :return: open3d.geometry.PointCloud
    """
    # Read the point cloud data from PointCloud2 message
    cloud = o3d.geometry.PointCloud()

    # Determine if there are RGB colors
    has_color = 'rgb' in [field.name for field in pcl_msg.fields]

    # Calculate the type of the byte-packed point
    fmt = '>' if pcl_msg.is_bigendian else '<'
    fmt += 'f' * (pcl_msg.point_step // 4)

    # Unpack point data
    points = []
    colors = []
    for p in range(0, pcl_msg.data.__len__(), pcl_msg.point_step):
        unpacked_data = struct.unpack(fmt, pcl_msg.data[p:p + pcl_msg.point_step])
        points.append(unpacked_data[:3])
        if has_color:
            # Unpack RGB into three separate float values for Open3D
            rgb = unpacked_data[3]
            # Struct assumes packed as int
            packed_color = ctypes.c_uint32(int(rgb)).value
            r = (packed_color >> 16) & 0x0000ff
            g = (packed_color >> 8) & 0x0000ff
            b = (packed_color) & 0x0000ff
            colors.append([r / 255.0, g / 255.0, b / 255.0])

    cloud.points = o3d.utility.Vector3dVector(np.array(points))

    if has_color:
        cloud.colors = o3d.utility.Vector3dVector(np.array(colors))

    return cloud


def o3d_to_ros_pointcloud2(pcd: o3d.geometry.PointCloud, frame_id="map") -> PointCloud2:
    """
    Convert an Open3D PointCloud to a sensor_msgs/PointCloud2 message.

    :param pcd: open3d.geometry.PointCloud
    :param frame_id: str, the frame ID to be used in the header of the PointCloud2
    :return: sensor_msgs/PointCloud2
    """
    # Create PointCloud2 message
    ros_pcl = PointCloud2()
    ros_pcl.header = Header(frame_id=frame_id, stamp=rospy.Time.now())

    # Assume pcd.points is not empty and is of type open3d.utility.Vector3dVector
    points = np.asarray(pcd.points)

    # Check if the point cloud contains colors
    if pcd.colors:
        colors = np.asarray(pcd.colors)
        fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyzrgb')]
        points = np.hstack([points, colors])
    else:
        fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]

    # Flatten array
    points = points.astype(np.float32)
    points = np.ravel(points)

    # Fill PointCloud2 message
    ros_pcl.fields = fields
    ros_pcl.is_bigendian = False
    ros_pcl.point_step = points.itemsize * (6 if pcd.colors else 3)  # fixed point step calculation
    ros_pcl.row_step = ros_pcl.point_step * len(pcd.points)
    ros_pcl.height = 1
    ros_pcl.width = len(pcd.points)
    ros_pcl.is_dense = True
    ros_pcl.data = np.asarray(points).tobytes()

    return ros_pcl


def align_cpd(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    result = probreg.cpd.registration_cpd(source, target, update_scale=False)
    output = copy.deepcopy(source)
    output.points = result.transformation.transform(output.points)
    return output.paint_uniform_color([0, 0, 1])


def align_small_gicp(
        source: o3d.geometry.PointCloud,
        target: o3d.geometry.PointCloud,
        registration_type="GICP") -> o3d.geometry.PointCloud:
    result = small_gicp.align(
        np.asarray(target.points), np.asarray(source.points), registration_type=registration_type)
    output = copy.deepcopy(source)
    output.transform(result.T_target_source)
    return output.paint_uniform_color([0, 0, 1])


def filter_points_within_bbox(point_cloud: o3d.geometry.PointCloud,
                              bbox: o3d.geometry.AxisAlignedBoundingBox) -> o3d.geometry.PointCloud:
    # Convert point cloud to numpy array
    points = np.asarray(point_cloud.points)
    # Get the bounding box min and max vectors
    min_bound = bbox.get_min_bound()
    max_bound = bbox.get_max_bound()
    # Create a mask for points within the bounding box
    mask = np.all((points >= min_bound) & (points <= max_bound), axis=1)
    # Filter points
    filtered_points = points[mask]
    # Create a new point cloud for the filtered points
    filtered_point_cloud = o3d.geometry.PointCloud()
    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
    if point_cloud.colors:
        colors = np.asarray(point_cloud.colors)
        filtered_colors = colors[mask]
        filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)
    return filtered_point_cloud


def raycasting(mesh: o3d.geometry.TriangleMesh, pose: np.ndarray) -> o3d.geometry.PointCloud:
    mesh_legacy = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_legacy)
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=90,
        center=[0, 0, 2],
        eye=[2, 3, 0],
        up=[0, 1, 0],
        width_px=640,
        height_px=480,
    )
    ans = scene.cast_rays(rays)
    hit = ans['t_hit'].isfinite()
    points = rays[hit][:, :3] + rays[hit][:, 3:] * ans['t_hit'][hit].reshape((-1, 1))
    pcd = o3d.t.geometry.PointCloud(points).to_legacy()
    pcd.transform(pose)
    pcd.paint_uniform_color([0, 1, 1])
    return pcd


if __name__ == '__main__':
    rospy.init_node('test')
    tf2 = ProxyTransformListener()
    scene = Soar.scene()
    co = scene.get_objects()
    sensor_cloud = get_current_o3d()
    co_cloud = create_o3d_box(co['Box_1'], points=10000)
    # output_cloud = align_cpd(sensor_cloud, co_cloud)
    # output_cloud = align_small_gicp(sensor_cloud, co_cloud, registration_type="GICP")
    # o3d.visualization.draw_geometries([sensor_cloud, co_cloud, output_cloud])
