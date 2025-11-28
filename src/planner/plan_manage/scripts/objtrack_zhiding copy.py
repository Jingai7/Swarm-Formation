#!/usr/bin/env python
import rospy
import math
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

NUM_DRONES = 6
NUM_TARGETS = 2
RADIUS = 0.8

class DroneDirectEncirclement:
    def __init__(self):
        rospy.init_node("drone_direct_encirclement")

        self.drone_positions = {}   # drone_id: (x, y, z)
        self.target_positions = {}  # target_id: (x, y, z)

        for i in range(NUM_DRONES):
            rospy.Subscriber(f"/drone_{i}_visual_slam/odom", Odometry, self.drone_callback, callback_args=i)

        for j in range(NUM_TARGETS):
            rospy.Subscriber(f"/target_{j}_visual_slam/odom", Odometry, self.target_callback, callback_args=j)

        self.odom_publishers = [
            rospy.Publisher(f"/drone_{i}_target/odom", PoseStamped, queue_size=1)
            for i in range(NUM_DRONES)
        ]

        self.dir_pub = [
            rospy.Publisher(f"/drone_{i}_rel/pose", PoseStamped, queue_size=1)
            for i in range(NUM_DRONES)
        ]

        rospy.Timer(rospy.Duration(1.0), self.assign_and_publish)

    def drone_callback(self, msg, drone_id):
        pos = msg.pose.pose.position
        self.drone_positions[drone_id] = (pos.x, pos.y, pos.z)

    def target_callback(self, msg, target_id):
        pos = msg.pose.pose.position
        self.target_positions[target_id] = (pos.x, pos.y, pos.z)

    def assign_and_publish(self, event):
        if len(self.target_positions) < NUM_TARGETS:
            rospy.loginfo_throttle(5, "Waiting for all targets to report positions...")
            return

        # Static assignment
        assignment = {
            0: 0,
            1: 0,
            2: 0,
            3: 1,
            4: 1,
            5: 1
        }

        grouped = {tid: [] for tid in range(NUM_TARGETS)}
        for drone_id, target_id in assignment.items():
            grouped[target_id].append(drone_id)

        for target_id, drone_list in grouped.items():
            total_agents = len(drone_list)
            target_pos = self.target_positions[target_id]
            directions = self.fibonacci_sphere(total_agents)

            for dir_vector, drone_id in zip(directions, drone_list):
                dx, dy, dz = dir_vector
                # goal_x = target_pos[0] + RADIUS * dx
                # goal_y = target_pos[1] + RADIUS * dy
                # goal_z = target_pos[2] + RADIUS * dz

                goal_x = target_pos[0] 
                goal_y = target_pos[1] 
                goal_z = target_pos[2] 

                d_x = target_pos[0] - goal_x
                d_y = target_pos[1] - goal_y
                d_z = target_pos[2] - goal_z
                yaw = math.atan2(d_y, d_x)
                pitch = math.atan2(d_z, math.hypot(d_x, d_y))
                quat = tf.transformations.quaternion_from_euler(0, pitch, yaw)

                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "world"

                pose_msg.pose.position.x = goal_x
                pose_msg.pose.position.y = goal_y
                pose_msg.pose.position.z = goal_z

                dir_msg = PoseStamped()
                dir_msg.header.stamp = rospy.Time.now()
                dir_msg.header.frame_id = "world"
                dir_msg.pose.position.x  = dx
                dir_msg.pose.position.y = dy
                dir_msg.pose.position.z = 0
                # 如果需要朝向：
                # pose_msg.pose.orientation = Quaternion(*quat)

                # 发布话题（注意：话题名要与 planner 订阅的一致）
                self.odom_publishers[drone_id].publish(pose_msg)
                self.dir_pub[drone_id].publish(dir_msg)

                rospy.loginfo(f"[Drone{drone_id}] encircling Target{target_id} at ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f})")


    def fibonacci_sphere(self, samples=1):
        points = []
        offset = 2.0 / samples
        increment = math.pi * (3.0 - math.sqrt(5.0))

        for i in range(samples):
            y = ((i * offset) - 1) + (offset / 2)
            r = math.sqrt(1 - y * y)
            phi = i * increment
            x = math.cos(phi) * r
            z = math.sin(phi) * r
            points.append((x, y, z))

        return points

if __name__ == "__main__":
    try:
        DroneDirectEncirclement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
