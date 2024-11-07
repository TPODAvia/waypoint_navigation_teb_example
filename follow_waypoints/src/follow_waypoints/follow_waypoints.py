#!/usr/bin/env python3

import rospy
import actionlib
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from std_msgs.msg import Empty, Bool
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time

# Global variables
waypoints = []
path_save = False
start_journey_bool = False
poseArray_publisher = None
output_file_path = rospkg.RosPack().get_path('follow_waypoints') + "/saved_path/pose.csv"
loop_requested = False  # Track if path_loop was requested
path_loop_state = loop_requested

# Function to change Pose to the correct frame
def changePose(waypoint, target_frame):
    if waypoint.header.frame_id == target_frame:
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except Exception as e:
        rospy.logerr("Can't transform pose to {} frame: {}".format(target_frame, e))
        raise

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id', 'map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

# Callback functions
def addpose_callback(pose):
    global waypoints, poseArray_publisher
    rospy.loginfo("Received new waypoint")
    waypoints.append(changePose(pose, "map"))
    poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

def path_save_callback(msg):
    global path_save, waypoints, output_file_path
    rospy.loginfo('Received path READY message')
    path_save = True
    with open(output_file_path, 'w') as file:
        for current_pose in waypoints:
            file.write(
                f"{current_pose.pose.pose.position.x},"
                f"{current_pose.pose.pose.position.y},"
                f"{current_pose.pose.pose.position.z},"
                f"{current_pose.pose.pose.orientation.x},"
                f"{current_pose.pose.pose.orientation.y},"
                f"{current_pose.pose.pose.orientation.z},"
                f"{current_pose.pose.pose.orientation.w}\n"
            )
    rospy.loginfo('Poses written to ' + output_file_path)

def start_journey_callback(msg):
    global start_journey_bool, waypoints, poseArray_publisher, output_file_path, loop_requested, path_loop_state
    if path_loop_state:
        loop_requested = True
    rospy.loginfo('Received start_journey message')
    with open(output_file_path, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            current_pose = PoseWithCovarianceStamped()
            current_pose.pose.pose.position.x = float(row[0])
            current_pose.pose.pose.position.y = float(row[1])
            current_pose.pose.pose.position.z = float(row[2])
            current_pose.pose.pose.orientation.x = float(row[3])
            current_pose.pose.pose.orientation.y = float(row[4])
            current_pose.pose.pose.orientation.z = float(row[5])
            current_pose.pose.pose.orientation.w = float(row[6])
            waypoints.append(current_pose)
        poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
    start_journey_bool = True

def path_reset_callback(msg):
    global waypoints, poseArray_publisher, loop_requested
    if waypoints == []:
        rospy.loginfo('Received path RESET message. But waypoints == []. No response')
        return
    rospy.loginfo('Received path RESET message')
    waypoints = []  # Reset the waypoint queue
    loop_requested = False  # Reset the loop request status
    poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

def path_loop_callback(msg):
    global loop_requested, path_loop_state
    rospy.loginfo(f'Received path LOOP message: {str(msg.data)}')
    if msg.data != path_loop_state:
        path_loop_state = msg.data
        loop_requested = msg.data  # True or False

class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.frame_id = rospy.get_param('~goal_frame_id', 'map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listener.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 1.0)
        self.final_waypoint_tolerance = rospy.get_param('final_waypoint_tolerance', 0.0) # This should be 

    def execute(self, userdata):
        global waypoints
        if waypoints == []:
            rospy.loginfo('The waypoint queue has been reset.')
            return 'success'
        for i, waypoint in enumerate(waypoints):
            is_last_waypoint = (i == len(waypoints) - 1)
            current_tolerance = self.final_waypoint_tolerance if is_last_waypoint else self.distance_tolerance
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            self.client.send_goal(goal)
            if not current_tolerance > 0.0:
                self.client.wait_for_result()
                rospy.loginfo("Waiting for %f sec..." % self.duration)
                time.sleep(self.duration)
            else:
                distance = 10
                while distance > current_tolerance:
                    if waypoints == []:
                        rospy.loginfo('The waypoint queue has been reset.')
                        return 'success'
                    now = rospy.Time.now()
                    self.listener.waitForTransform(
                        self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    trans, rot = self.listener.lookupTransform(
                        self.odom_frame_id, self.base_frame_id, now)
                    distance = math.sqrt(
                        (waypoint.pose.pose.position.x - trans[0]) ** 2 +
                        (waypoint.pose.pose.position.y - trans[1]) ** 2)
                    rospy.sleep(0.5)
        return 'success'

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        global path_save, start_journey_bool
        while not rospy.is_shutdown() and not path_save and not start_journey_bool:
            rospy.sleep(1.0)
        return 'success'

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'loop'])

    def execute(self, userdata):
        global waypoints, path_save, start_journey_bool, poseArray_publisher, loop_requested
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        rospy.sleep(5)
        if loop_requested:
            rospy.loginfo('##### RETURN loop #####')
            return 'loop'
        waypoints = []
        path_save = False
        start_journey_bool = False
        rospy.loginfo('##### RETURN success #####')
        return 'success'

def main():
    global poseArray_publisher, addpose_topic, output_file_path
    rospy.init_node('follow_waypoints')

    addpose_topic = rospy.get_param('~addpose_topic', '/initialpose')
    posearray_topic = rospy.get_param('~posearray_topic', '/waypoints')
    frame_id = rospy.get_param('~goal_frame_id', 'map')
    output_file_path = rospkg.RosPack().get_path('follow_waypoints') + "/saved_path/pose.csv"

    poseArray_publisher = rospy.Publisher(posearray_topic, PoseArray, queue_size=1)

    rospy.Subscriber(addpose_topic, PoseWithCovarianceStamped, addpose_callback)
    rospy.Subscriber('/path_save', Empty, path_save_callback)
    rospy.Subscriber('/start_journey', Empty, start_journey_callback)
    rospy.Subscriber('/path_reset', Empty, path_reset_callback)
    rospy.Subscriber('/path_loop', Bool, path_loop_callback)

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                         transitions={'success': 'FOLLOW_PATH'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                         transitions={'success': 'PATH_COMPLETE'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                         transitions={'success': 'GET_PATH', 'loop': 'FOLLOW_PATH'})

    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
