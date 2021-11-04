# ===============================================================
#	Course  :   Legged robots
# 	Alumno  :   Jhon Charaja
# 	Info	:	create ball markers on rviz
# ===============================================================

# ======================
#   required libraries
# ======================
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import rospy

# ==========
#   Colors
# ==========
color = dict()
color['RED']       = (1.0, 0.0, 0.0)
color['GREEN']     = (0.0, 1.0, 0.0)
color['BLUE']      = (0.0, 0.0, 1.0)
color['YELLOW']    = (1.0, 1.0, 0.0)
color['PINK']      = (1.0, 0.0, 1.0)
color['CYAN']      = (0.0, 1.0, 1.0)
color['BLACK']     = (0.0, 0.0, 0.0)
color['DARKGRAY']  = (0.2, 0.2, 0.2)
color['LIGHTGRAY'] = (0.5, 0.5, 0.5)
color['WHITE']     = (1.0, 1.0, 1.0)

# =====================
#   Class ball marker
# =====================
class BallMarker(object):
    """
    Info : class to visualize ball markers in RViz
    """
    id = 0

    def __init__(self, color, alpha=1.0, scale=0.05):
        """
        The color can be specified as a list with 3 elements or as the color
        dictionary (e.g. BLUE, RED, etc). Alpha sets the transparency and scale
        scales the size of the ball
        """
        reference_frame = rospy.get_param('reference_frame','base_link') # important 
        self.marker_pub = rospy.Publisher("visualization_marker", Marker,
                                          queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = reference_frame
        self.marker.ns = "ball_markers"
        self.marker.id = BallMarker.id
        BallMarker.id += 1
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale
        self.setColor(color, alpha)
        self.marker.lifetime = rospy.Duration()


    def setColor(self, color, alpha=1.0):
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = alpha

    def position(self, T):
        """
        Info: set position (4x4 NumPy homogeneous matrix) for the ball and publish it

        """
        self.marker.pose.position.x = T[0,3]
        self.marker.pose.position.y = T[1,3]
        self.marker.pose.position.z = T[2,3]
        self.publish()

    def xyz(self, position):
        """
        Info: set position (list) for the ball and publish it

        """
        self.marker.pose.position.x = position[0]
        self.marker.pose.position.y = position[1]
        self.marker.pose.position.z = position[2]
        self.publish()

    def publish(self):
        self.marker_pub.publish(self.marker)
