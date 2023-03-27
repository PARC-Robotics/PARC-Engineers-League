#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Vector3, Point, Quaternion
import sys
from routes import ROUTE_1, ROUTE_2, ROUTE_3
from std_msgs.msg import String


class RobotController:
    '''
    Interpolates path at 30 Hz to move the robot
    '''

    def __init__(self, speed=0.1, route=ROUTE_1) -> None:
        rospy.init_node("robot_controller")
        self.pub = rospy.Publisher(
            "/gazebo/set_model_state", ModelState, queue_size=30)
        self.state_pub = rospy.Publisher(
            "/parc_robot/robot_status", String, queue_size=10)
        self.path = route
        self.speed = speed
        self.frequency = 30
        self.states = self.get_states()
        self.current_state = 0
        self.rate = rospy.Rate(self.frequency)

    def get_distance(self, state1, state2):
        # Calculate the distance between two states - we are only interested in the x and y coordinates
        x1 = state1["pose"]["position"]["x"]
        y1 = state1["pose"]["position"]["y"]
        x2 = state2["pose"]["position"]["x"]
        y2 = state2["pose"]["position"]["y"]
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def get_states(self):
        new_states = []
        # Generate all states required to move from one state to the next at 30 Hz at constant speed
        for i in range(len(self.path) - 1):
            state1 = self.path[i]
            state2 = self.path[i+1]
            distance = self.get_distance(state1, state2)
            steps = distance / self.speed * self.frequency
            for j in range(int(steps)):
                new_state = self.interpolate(state1, state2, j / steps)
                new_states.append(new_state)
        return new_states

    def interpolate(self, state1, state2, t):
        """Interpolate between two states at time t

        Args:
            state1  (dict): State 1
            state2  (dict): State 2
            t  (float): Time between 0 and 1

        Returns:
            dict: Interpolated state
        """
        new_state = {}
        new_state["pose"] = self.interpolate_pose(
            state1["pose"], state2["pose"], t)
        new_state["twist"] = {
            "linear": {
                "x": 0,
                "y": 0,
                "z": 0
            },
            "angular": {
                "x": 0,
                "y": 0,
                "z": 0
            }}
        return new_state

    def interpolate_pose(self, pose1, pose2, t):
        # Interpolate between two poses at time t
        new_pose = {}
        new_pose["position"] = self.interpolate_position(
            pose1["position"], pose2["position"], t)
        new_pose["orientation"] = self.interpolate_orientation(
            pose1["orientation"], pose2["orientation"], t)
        return new_pose

    def interpolate_position(self, position1, position2, t):
        # Interpolate between two positions at time t
        new_position = {}
        new_position["x"] = position1["x"] + \
            (position2["x"] - position1["x"]) * t
        new_position["y"] = position1["y"] + \
            (position2["y"] - position1["y"]) * t
        new_position["z"] = position1["z"] + \
            (position2["z"] - position1["z"]) * t
        return new_position

    def interpolate_orientation(self, orientation1, orientation2, t):
        # Interpolate between two orientations at time t
        new_orientation = {}
        new_orientation["x"] = orientation1["x"] + \
            (orientation2["x"] - orientation1["x"]) * t
        new_orientation["y"] = orientation1["y"] + \
            (orientation2["y"] - orientation1["y"]) * t
        new_orientation["z"] = orientation1["z"] + \
            (orientation2["z"] - orientation1["z"]) * t
        new_orientation["w"] = orientation1["w"] + \
            (orientation2["w"] - orientation1["w"]) * t
        return new_orientation

    def interpolate_twist(self, twist1, twist2, t):
        # Interpolate between two twists at time t
        new_twist = {}
        new_twist["linear"] = self.interpolate_linear(
            twist1["linear"], twist2["linear"], t)
        new_twist["angular"] = self.interpolate_angular(
            twist1["angular"], twist2["angular"], t)
        return new_twist

    def interpolate_linear(self, linear1, linear2, t):
        # Interpolate between two linear velocities at time t
        new_linear = {}
        new_linear["x"] = linear1["x"] + (linear2["x"] - linear1["x"]) * t
        new_linear["y"] = linear1["y"] + (linear2["y"] - linear1["y"]) * t
        new_linear["z"] = linear1["z"] + (linear2["z"] - linear1["z"]) * t
        return new_linear

    def interpolate_angular(self, angular1, angular2, t):
        # Interpolate between two angular velocities at time t
        new_angular = {}
        new_angular["x"] = angular1["x"] + (angular2["x"] - angular1["x"]) * t
        new_angular["y"] = angular1["y"] + (angular2["y"] - angular1["y"]) * t
        new_angular["z"] = angular1["z"] + (angular2["z"] - angular1["z"]) * t
        return new_angular

    def run(self):
        while self.pub.get_num_connections() == 0:
            rospy.loginfo(
                f"Waiting for subscriber to {self.pub.resolved_name}")
            rospy.sleep(1)

        rospy.loginfo(f"Publishing to {self.pub.resolved_name}")
        self.state_pub.publish("started")
        while not rospy.is_shutdown():
            if self.current_state >= len(self.states):
                self.state_pub.publish("finished")
                rospy.signal_shutdown("At the end of path")
                continue

            current_state = self.states[self.current_state]
            msg = ModelState(
                model_name="robot",
                pose=Pose(
                    position=Point(
                        x=current_state["pose"]["position"]["x"],
                        y=current_state["pose"]["position"]["y"],
                        z=current_state["pose"]["position"]["z"]
                    ),
                    orientation=Quaternion(
                        x=current_state["pose"]["orientation"]["x"],
                        y=current_state["pose"]["orientation"]["y"],
                        z=current_state["pose"]["orientation"]["z"],
                        w=current_state["pose"]["orientation"]["w"]
                    )
                ),
                twist=Twist(
                    linear=Vector3(
                        x=current_state["twist"]["linear"]["x"],
                        y=current_state["twist"]["linear"]["y"],
                        z=current_state["twist"]["linear"]["z"]
                    ),
                    angular=Vector3(
                        x=current_state["twist"]["angular"]["x"],
                        y=current_state["twist"]["angular"]["y"],
                        z=current_state["twist"]["angular"]["z"]
                    )
                ),
                reference_frame="world"
            )
            self.pub.publish(msg)
            self.current_state += 1
            self.rate.sleep()


if __name__ == "__main__":
    try:
        speed = float(sys.argv[1])
        route_arg = sys.argv[2]
    except IndexError:
        print(
            "\033[91m" + "Usage: python robot_controller.py <speed> <route>" + "\033[0m")
        sys.exit(1)

    route = ROUTE_1

    if route_arg == "route2":
        route = ROUTE_2
    elif route_arg == "route3":
        route = ROUTE_3

    # Sleep for 5 seconds to allow gazebo to start
    rospy.sleep(5)

    controller = RobotController(speed=speed, route=route)
    controller.run()
