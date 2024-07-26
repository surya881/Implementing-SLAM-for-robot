#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl
from pcl import PointCloud
import tf
import tf_conversions
from pcl.registration import icp
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Define global variables
cloud = pcl.PointCloud()
cloud_filtered = pcl.PointCloud()
pcl_cloud = pcl.PointCloud()
pcl_cloud_filtered = pcl.PointCloud()
trans = np.identity(4, dtype=np.float32)

def downsample(cloud):
    sor = cloud.make_voxel_grid_filter()
    sor.set_leaf_size(0.6, 0.6, 0.6)
    return sor.filter()

def sensor_data(msg):
    global pcl_cloud, pcl_cloud_filtered, cloud_filtered, trans

    pcl_cloud = PointCloud(np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32))

    pcl_cloud_filtered = downsample(pcl_cloud)

    # ICP
    icp_instance = pcl.IterativeClosestPoint()
    icp_instance.set_input_source(pcl_cloud_filtered)
    icp_instance.set_input_target(cloud_filtered)
    converged, transf, estimate, fitness = icp_instance.icp(pcl_cloud_filtered, cloud_filtered, max_iter=50)
    print(f'has converged: {converged}, score: {fitness}')
    print(transf)
    trans = transf

def tf_broadcast(trans):
    br = TransformBroadcaster()
    t = TransformStamped()
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "scan"
    
    q = tf.transformations.quaternion_from_matrix(trans)
    t.transform.translation.x = trans[0, 3]
    t.transform.translation.y = trans[1, 3]
    t.transform.translation.z = trans[2, 3]
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def main():
    global cloud, cloud_filtered, trans

    rospy.init_node('hw5', anonymous=True)

    # Load map
    try:
        cloud = pcl.load_XYZRGB('dlo_map.pcd')
    except:
        rospy.logerr("Couldn't read file dlo_map.pcd")
        return

    print("cloud Loaded")
    cloud_filtered = downsample(cloud)

    rospy.Subscriber('/points_raw', PointCloud2, sensor_data)
    map_pub = rospy.Publisher('/map', PointCloud2, queue_size=1000)
    scan_pub = rospy.Publisher('/scan', PointCloud2, queue_size=1000)

    rate = rospy.Rate(30)

    # Initialization
    degree = -135.0
    initial_guess = np.array([
        [np.cos(degree * np.pi / 180.0), np.sin(degree * np.pi / 180.0), 0.0, 0.0],
        [-np.sin(degree * np.pi / 180.0), np.cos(degree * np.pi / 180.0), 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float32)

    trans = initial_guess

    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        cloud_points = [(p[0], p[1], p[2], 0, 255, 0) for p in cloud_filtered.to_list()]
        scan_points = [(p[0], p[1], p[2], 255, 0, 0) for p in pcl_cloud_filtered.to_list()]

        cloud_rgb = pc2.create_cloud(header, pc2.PointFieldXYZRGB, cloud_points)
        scan_rgb = pc2.create_cloud(header, pc2.PointFieldXYZRGB, scan_points)

        map_pub.publish(cloud_rgb)
        scan_pub.publish(scan_rgb)
        tf_broadcast(trans)
        rospy.spinOnce()
        rate.sleep()

if __name__ == '__main__':
    main()

