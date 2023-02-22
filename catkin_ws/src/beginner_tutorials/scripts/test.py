#!/usr/bin/env python
import math
import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller')
        self.rate = rospy.Rate(10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.stop_distance = 1.0 # distance in meters
        self.goal = None # the goal point

    def laser_callback(self, msg):
        # stop if there's an obstacle within stop_distance
        if min(msg.ranges) < self.stop_distance:
            self.stop()

    def stop(self):
        # stop the robot
        self.cmd_vel_pub.publish(Twist())

    def go_to_goal(self, goal):
        self.goal = goal
        while not rospy.is_shutdown():
            # calculate the distance to the goal
            distance = self.get_distance_to_goal()
            print(distance < self.stop_distance +- 0.1)
            print(distance)
            print(self.stop_distance +- 0.1)
            if distance < self.stop_distance +- 0.1:
                # stop if we're close enough to the goal
                self.stop()
                return
            # calculate the direction to the goal
            angle = self.get_angle_to_goal()
            # create a Twist message to move the robot
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = angle
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def get_distance_to_goal(self):
        # calculate the distance to the goal
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        result = get_model_state('turtlebot3_burger', '')
        turtlebot_position = result.pose.position
        distance = ((goal.x - turtlebot_position.x) ** 2 + (goal.y - turtlebot_position.y) ** 2) ** 0.5
        return distance

    def get_angle_to_goal(self):
        # calculate the angle to the goal
        x, y = self.goal
        return math.atan2(y, x)

if __name__ == '__main__':
    try:
        tb = TurtleBot()
        # wait for the first goal 
        goal = rospy.wait_for_message('/goal', Point)
        tb.go_to_goal((goal.x, goal.y))
    except rospy.ROSInterruptException:
        pass
