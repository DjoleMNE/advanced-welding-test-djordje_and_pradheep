#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class TrajectoryInteractiveMarkers(object):
	def __init__(self):
		self.count = 0 
		rospy.Subscriber("/arm_1/arm_controller/cartesian_velocity_command",TwistStamped, self.event_in_cb)
		rospy.sleep(0.5)

	def event_in_cb(self,msg):
		self.waypoints = msg
		self.a = list()
		self.a.append(self.waypoints.twist.linear.x)
		self.a.append(self.waypoints.twist.linear.y)
		self.a.append(self.waypoints.twist.linear.z)
		self.show_text_in_rviz()

	def show_text_in_rviz(self):
		self.marker = Marker()
		self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
		self.marker = Marker(
	                type=Marker.SPHERE,
	                id=0,
	                lifetime=rospy.Duration(1000),
	                pose=Pose(Point(self.a[0]/10**2,self.a[1]/10**2,self.a[2]/10**2), Quaternion(0, 0, 0, 0)),
	                scale=Vector3(0.05, 0.05, 0.05),
	                header=Header(frame_id='arm_link_5'),
	                color=ColorRGBA(0.0, 2.0, 0.0, 0.8))
		self.count+=1
		self.marker.id = self.count
		self.marker_publisher.publish(self.marker)

if __name__ == '__main__':
	rospy.init_node("trajectory_interactive_markers_node", anonymous=True)
	trajectory_interactive_markers = TrajectoryInteractiveMarkers()
	rospy.sleep(0.5)
	rospy.spin()