#!/usr/bin/env python
import numpy
import tf
import rospy
from geometry_msgs.msg import Twist
from view_controller_msgs.msg import CameraPlacement

class RvizSpacenav:
    def __init__(self):
        # parameters
        self.hz = 20  # hz fot CameraPlacement Topic
        self.Kp = 0.3 # Coefficient for position control
        self.Kr = 2   # Coefficient for rotation control

        # local variable
        self.publisher = rospy.Publisher('camera', CameraPlacement, queue_size=10)
        self.twist = None # subscribed message
        self.lock = False
        self.camera = CameraPlacement()

        # First Publishe
        rospy.sleep(0.1) # wait for connection
        self.initialize()

        # start listening
        self.camera.time_from_start = rospy.Duration.from_sec(1/self.hz/2)
        self.subscriber = rospy.Subscriber('spacenav_twist', Twist, self.spacenav_callback)
    def initialize(self):
        # set initial camera pose
        self.camera.interpolation_mode = CameraPlacement.LINEAR
        self.camera.up.vector.z = 1
        self.camera.eye.point.x = 4
        self.camera.eye.point.y = 4
        self.camera.eye.point.z = 1
        wait_duration = rospy.Duration(1)
        self.camera.time_from_start = wait_duration
        self.publisher.publish(self.camera)
        rospy.sleep(wait_duration+rospy.Duration(0.1))


    def spacenav_callback(self, msg):
        if not self.lock:
            self.twist = msg

    def get_transformation(self):
        '''
        return R
        x: vector in camera frame
        y: vector in fixed frame
        y = R*x
        '''
        camera = numpy.array([self.camera.eye.point.x,\
                                  self.camera.eye.point.y,\
                                  self.camera.eye.point.z])
        focus = numpy.array([self.camera.focus.point.x,\
                             self.camera.focus.point.y,\
                             self.camera.focus.point.z])
        up = numpy.array([self.camera.up.vector.x,\
                          self.camera.up.vector.y,\
                          self.camera.up.vector.z])
        ex_ = tf.transformations.unit_vector(focus - camera)
        ey_ = tf.transformations.unit_vector(numpy.cross(up, ex_))
        ez_ = numpy.cross(ex_, ey_)
        trans = numpy.array([ex_, ey_, ez_]).transpose()
        return trans

    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            if self.twist is None:
                continue
            self.lock = True
            msg = self.twist

            linear_ = numpy.array([[msg.linear.x, msg.linear.y, msg.linear.z]]).transpose()
            linear = numpy.dot(self.get_transformation(), linear_)

            self.camera.eye.point.x += self.Kp*linear[0][0]
            self.camera.eye.point.y += self.Kp*linear[1][0]
            self.camera.eye.point.z += self.Kp*linear[2][0]
            self.camera.focus.point.x += self.Kp*linear[0][0]
            self.camera.focus.point.y += self.Kp*linear[1][0]
            self.camera.focus.point.z += self.Kp*linear[2][0]

            rot = numpy.dot(self.get_transformation(), numpy.array([[0, 0.1*msg.angular.z, -0.1*msg.angular.y]]).transpose())
            self.camera.focus.point.x += self.Kr*rot[0][0]
            self.camera.focus.point.y += self.Kr*rot[1][0]
            self.camera.focus.point.z += self.Kr*rot[2][0]

            self.publisher.publish(self.camera)
            self.lock = False
            rate.sleep()

if __name__=='__main__':
    try:
        rospy.init_node('rospy_spacenav', anonymous=True)
        runner = RvizSpacenav()
        runner.run()
    except rospy.ROSInterruptException:
        pass
