#! /usr/bin/python
"""
    This script will sub to all convex_hull outputs and fit a plane to them. Finally it will return the rotation of that plane wrt to base_link

"""
import numpy as np
import rospy
from geometry_msgs.msg import PolygonStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf_conversions
from surface_normals.msg import SurfaceElement

class PlaneFitter():
    def __init__(self, polygon_topic, idx):
        self.poly_sub = rospy.Subscriber(polygon_topic, PolygonStamped, self.fit_plane)
        self.name = "plane_fit_{}".format(idx)
        # self.poly_sub = rospy.Subscriber(polygon_topic, PointCloud2, self.fit_plane)
        self.plane_pub = rospy.Publisher("/plane_fit_{}".format(idx), MarkerArray, queue_size=10)
        self.surfel_pub = rospy.Publisher("/surfels", SurfaceElement, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # self.plane_pub = rospy.Publisher("/plane_fit_{}".format(idx), Marker, queue_size=10)
        self.t = TransformStamped()
        self.t.header.frame_id = 'rgb_camera_link'
        self.t.child_frame_id = self.name
        self.mean = None
        self.rotation = None

    def publish_transform(self):
        if self.mean is None or self.rotation is None:
            return
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation.x = self.mean[0]
        self.t.transform.translation.y = self.mean[1]
        self.t.transform.translation.z = self.mean[2]
        q = tf_conversions.transformations.quaternion_about_axis(self.rotation, (0,0,1))
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(self.t)

    def fit_plane(self, data):
        # points =
        surfel_msg = SurfaceElement()
        surfel_msg.header.frame_id = "rgb_camera_link"
        points = np.zeros(shape=(3,len(data.polygon.points)))
        for i, p in enumerate(data.polygon.points):
            points[0,i] = p.x
            points[1,i] = p.y
            points[2,i] = p.z
        surfel_msg.min_x = np.min(points[0,:])
        surfel_msg.max_x = np.max(points[0,:])
        surfel_msg.min_y = np.min(points[1,:])
        surfel_msg.max_y = np.max(points[1,:])
        surfel_msg.min_z = np.min(points[2,:])
        surfel_msg.max_z = np.max(points[2,:])
        mean = np.mean(points, axis=1, keepdims=True)
        self.mean = mean
        svd = np.linalg.svd(points - mean)
        left = svd[0]
        # the corresponding left singular vector is the normal vector of the best-fitting plane
        norm_vecz = left[:,-1]
        surfel_msg.slope = norm_vecz[2]*100
        # surfel_msg.normal_x = norm_vecz[0]
        # surfel_msg.normal_y = norm_vecz[1]
        # surfel_msg.normal_z = norm_vecz[2]
        if surfel_msg.slope > 5:
            self.surfel_pub.publish(surfel_msg)
        origin_y = np.array([0,0,1]).T
        c = np.dot(origin_y, norm_vecz)
        self.rotation = np.arccos(c)
        

        norm_vecx = left[:, 0]
        norm_vecy = left[:,1]
        start = Point()
        endx = Point()
        endy = Point()
        endz = Point()
        start.x = mean[0]
        start.y = mean[1]
        start.z = mean[2]
        endx.x = start.x + norm_vecx[0]
        endx.y = start.y + norm_vecx[1]
        endx.z = start.z + norm_vecx[2]
        xax = Marker()
        xax.id = 0
        xax.header.frame_id = data.header.frame_id
        xax.type = xax.LINE_LIST
        xax.points = [start, endx]
        xax.scale.x = 0.2
        xax.scale.y = 0.2
        xax.scale.z = 0.2
        xax.color.r = 1.0
        xax.color.a = 0.7
        
        endy.x = start.x + norm_vecy[0]
        endy.y = start.y + norm_vecy[1]
        endy.z = start.z + norm_vecy[2]
        yax = Marker()
        yax.id = 1
        yax.header.frame_id = data.header.frame_id
        yax.type = yax.LINE_LIST
        yax.points = [start, endy]
        yax.scale.x = 0.2
        yax.scale.y = 0.2
        yax.scale.z = 0.2
        yax.color.g = 1.0
        yax.color.a = 0.7
        
        endz.x = start.x + norm_vecz[0]
        endz.y = start.y + norm_vecz[1]
        endz.z = start.z + norm_vecz[2]
        zax = Marker()
        zax.id = 2
        zax.header.frame_id = data.header.frame_id
        zax.type = zax.LINE_LIST
        zax.points = [start, endz]
        zax.scale.x = 0.2
        zax.scale.y = 0.2
        zax.scale.z = 0.2
        zax.color.b = 1.0
        zax.color.a = 0.7
        msg = MarkerArray()
        msg.markers = [xax, yax, zax]
        # lineobj = Marker()
        # lineobj.header.frame_id = data.header.frame_id
        # lineobj.points = [start, end]
        # lineobj.type = lineobj.LINE_LIST
        # lineobj.scale.x = 0.2
        # lineobj.scale.y = 0.2
        # lineobj.scale.z = 0.2
        # lineobj.color.g = 1.0
        # lineobj.color.a = 0.7
        self.plane_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("kisailus_plane_fit_node", anonymous=True)
    foo = PlaneFitter(rospy.get_param('~polygon_topic'), rospy.get_param('~id'))
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        foo.publish_transform()
        r.sleep()


