"""
Fundamentos de Robotica 2019-1 - UTEC
Prof. Oscar E. Ramos
File: markers.py

"""

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import rospy


class BallMarker(object):
    """
    Class to visualize ball markers in RViz

    """
    id = 0

    def __init__(self, color, alpha=1.0, scale=0.05):
        """
        The color can be specified as a list with 3 elements or as the color
        dictionary (e.g. BLUE, RED, etc). Alpha sets the transparency and scale
        scales the size of the ball

        """
        reference_frame = rospy.get_param('reference_frame','/base')
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
        Set position (4x4 NumPy homogeneous matrix) for the ball and publish it

        """
        self.marker.pose.position.x = T[0,3]
        self.marker.pose.position.y = T[1,3]
        self.marker.pose.position.z = T[2,3]
        self.publish()

    def xyz(self, position):
        """
        Set position (list) for the ball and publish it

        """
        self.marker.pose.position.x = position[0]
        self.marker.pose.position.y = position[1]
        self.marker.pose.position.z = position[2]
        self.publish()


    def publish(self):
        self.marker_pub.publish(self.marker)


#========================================================

class LineMarker(object):
    """
    Class to visualize Line markers in RViz

    """
    id = 0

    def __init__(self, color, alpha=1.0, scale=0.01):
        """
        The color can be specified as a list with 3 elements or as the color
        dictionary (e.g. BLUE, RED, etc). Alpha sets the transparency and scale
        scales the size of the Line

        """
        reference_frame = rospy.get_param('reference_frame','/base')
        self.marker_pub = rospy.Publisher("visualization_marker", Marker,
                                          queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = reference_frame
        self.marker.ns = "line_markers"
        self.marker.id = LineMarker.id
        LineMarker.id += 1
        self.marker.type = self.marker.LINE_STRIP#self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = scale
        self.marker.scale.y = 0.0
        self.marker.scale.z = 0.0
        self.setColor(color, alpha)
        self.marker.lifetime = rospy.Duration()
        self.marker.points = []

    def setColor(self, color, alpha=1.0):
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = alpha

    def position(self, T):
        """
        Set position (4x4 NumPy homogeneous matrix) for the ball and publish it

        """
        self.marker.pose.position.x = T[0,3]
        self.marker.pose.position.y = T[1,3]
        self.marker.pose.position.z = T[2,3]
        self.publish()

    def addpoint(self, position):
        """
        Set position (list) for the ball and publish it

        """
        new_point = Point()
        new_point.x = position[0]
        new_point.y = position[1]
        new_point.z = position[2]
        self.marker.points.append(new_point)
        self.publish()


    def publish(self):
        
        
        self.marker_pub.publish(self.marker)






"""
List for colors in BallMarker

"""
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




class FrameMarker(object):
    """
    Class to visualize frames as markers in RViz

    """
    id = 0

    def __init__(self, color_saturation=1.0, alpha=1.0, scale=0.1):
        """
        The color saturation ranges from 0 to 1. Alpha sets the transparency
        and scale scales the size of the ball

        """
        reference_frame = rospy.get_param('reference_frame','/map')
        self.marker_pub = rospy.Publisher("visualization_marker", Marker,
                                          queue_size=10)
        self.markerx = Marker()
        self.markery = Marker()
        self.markerz = Marker()
        self.markerx.header.frame_id = reference_frame
        self.markery.header.frame_id = reference_frame
        self.markerz.header.frame_id = reference_frame
        self.markerx.ns = "frame_markers"
        self.markery.ns = "frame_markers"
        self.markerz.ns = "frame_markers"
        self.markerx.id = FrameMarker.id; FrameMarker.id += 1
        self.markery.id = FrameMarker.id; FrameMarker.id += 1
        self.markerz.id = FrameMarker.id; FrameMarker.id += 1
        self.markerx.type = self.markerx.ARROW
        self.markery.type = self.markery.ARROW
        self.markerz.type = self.markerz.ARROW
        self.markerx.action = self.markerx.ADD
        self.markery.action = self.markery.ADD
        self.markerz.action = self.markerz.ADD
        self.markerx.pose.position.x = 0.0
        self.markerx.pose.position.y = 0.0
        self.markerx.pose.position.z = 0.0
        self.markerx.pose.orientation.w = 1.0
        self.markerx.pose.orientation.x = 0.0
        self.markerx.pose.orientation.y = 0.0
        self.markerx.pose.orientation.z = 0.0
        self.markery.pose.position.x = 0.0
        self.markery.pose.position.y = 0.0
        self.markery.pose.position.z = 0.0
        self.markery.pose.orientation.w = np.cos(np.pi/4.0)
        self.markery.pose.orientation.x = 0.0
        self.markery.pose.orientation.y = 0.0
        self.markery.pose.orientation.z = np.sin(np.pi/4.0)
        self.markerz.pose.position.x = 0.0
        self.markerz.pose.position.y = 0.0
        self.markerz.pose.position.z = 0.0
        self.markerz.pose.orientation.w = np.cos(-np.pi/4.0)
        self.markerz.pose.orientation.x = 0.0
        self.markerz.pose.orientation.y = np.sin(-np.pi/4.0)
        self.markerz.pose.orientation.z = 0.0
        self.markerx.scale.x = scale
        self.markerx.scale.y = 0.01
        self.markerx.scale.z = 0.01
        self.markery.scale.x = scale
        self.markery.scale.y = 0.01
        self.markery.scale.z = 0.01
        self.markerz.scale.x = scale
        self.markerz.scale.y = 0.01
        self.markerz.scale.z = 0.01
        self.markerx.color.r = color_saturation
        self.markerx.color.g = 0.0
        self.markerx.color.b = 0.0
        self.markerx.color.a = alpha
        self.markery.color.r = 0.0
        self.markery.color.g = color_saturation
        self.markery.color.b = 0.0
        self.markery.color.a = alpha
        self.markerz.color.r = 0.0
        self.markerz.color.g = 0.0
        self.markerz.color.b = color_saturation
        self.markerz.color.a = alpha
        self.markerx.lifetime = rospy.Duration()
        self.markery.lifetime = rospy.Duration()
        self.markerz.lifetime = rospy.Duration()

    def setPose(self, pose):
        """
        Set the pose (7x1 NumPy matrix) for the ball and publish it. If only
        position is passed, a canonical orientation is used.

        """
        self.markerx.pose.position.x = pose[0][0]
        self.markerx.pose.position.y = pose[1][0]
        self.markerx.pose.position.z = pose[2][0]
        self.markery.pose.position.x = pose[0][0]
        self.markery.pose.position.y = pose[1][0]
        self.markery.pose.position.z = pose[2][0]
        self.markerz.pose.position.x = pose[0][0]
        self.markerz.pose.position.y = pose[1][0]
        self.markerz.pose.position.z = pose[2][0]
        if (len(pose)==7):
            # X is aligned and has no rotation
            self.markerx.pose.orientation.w = pose[3][0]
            self.markerx.pose.orientation.x = pose[4][0]
            self.markerx.pose.orientation.y = pose[5][0]
            self.markerx.pose.orientation.z = pose[6][0]

            # Y is rotated 90 wrt current Z
            q1 = np.matrix([[np.cos(np.pi/4.0)],[0.],[0.],[np.sin(np.pi/4.0)]])
            q = quaternionMult(pose[np.ix_([3,4,5,6])],q1)
            self.markery.pose.orientation.w = q[0][0]
            self.markery.pose.orientation.x = q[1][0]
            self.markery.pose.orientation.y = q[2][0]
            self.markery.pose.orientation.z = q[3][0]

            # Z is rotated -90 wrt current Y
            q1 = np.matrix([[np.cos(-np.pi/4.0)],[0.],[np.sin(-np.pi/4.0)],[0.]])
            q = quaternionMult(pose[np.ix_([3,4,5,6])],q1)
            self.markerz.pose.orientation.w = q[0][0]
            self.markerz.pose.orientation.x = q[1][0]
            self.markerz.pose.orientation.y = q[2][0]
            self.markerz.pose.orientation.z = q[3][0]

        self.publish()

    def publish(self):
        self.marker_pub.publish(self.markerx)
        self.marker_pub.publish(self.markery)
        self.marker_pub.publish(self.markerz)


def quaternionMult(q1, q2):
    quat = 4*[0.,]
    quat[0] = -q1[1,0]*q2[1,0]-q1[2,0]*q2[2,0]-q1[3,0]*q2[3,0]+q1[0,0]*q2[0,0]
    quat[1] =  q1[0,0]*q2[1,0]-q1[3,0]*q2[2,0]+q1[2,0]*q2[3,0]+q1[1,0]*q2[0,0]
    quat[2] =  q1[3,0]*q2[1,0]+q1[0,0]*q2[2,0]-q1[1,0]*q2[3,0]+q1[2,0]*q2[0,0]
    quat[3] = -q1[2,0]*q2[1,0]+q1[1,0]*q2[2,0]+q1[0,0]*q2[3,0]+q1[3,0]*q2[0,0]
    return np.matrix([quat]).transpose()


def vtotuple(v):
    return [val[0,0] for val in v]
