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
RADIUS = 1.0
MAX_DRONES_PER_TARGET = 3
EPSILON = 0.01


class DroneAuctionEncirclement:
    def __init__(self):
        rospy.init_node("drone_auction_encirclement")

        self.drone_positions = {}   # drone_id: (x, y)
        self.target_positions = {}  # target_id: (x, y)

        # 订阅无人机和目标的位置
        for i in range(NUM_DRONES):
            rospy.Subscriber(f"/drone_{i}_visual_slam/odom", Odometry, self.drone_callback, callback_args=i)

        for j in range(NUM_TARGETS):
            rospy.Subscriber(f"/target_{j}_visual_slam/odom", Odometry, self.target_callback, callback_args=j)

        self.odom_publishers = [
            rospy.Publisher(f"/drone_{i}_target/odom", PoseStamped, queue_size=1)
            for i in range(NUM_DRONES)
        ]

        # 启动拍卖 + 控制定时器
        rospy.Timer(rospy.Duration(1.0), self.assign_and_publish)

    def drone_callback(self, msg, drone_id):
        pos = msg.pose.pose.position
        self.drone_positions[drone_id] = (pos.x, pos.y,pos.z)
        rospy.loginfo_throttle(5, f"[Drone{drone_id}] position: ({pos.x:.2f}, {pos.y:.2f})")

    def target_callback(self, msg, target_id):
        pos = msg.pose.pose.position
        self.target_positions[target_id] = (pos.x, pos.y,pos.z)

    def assign_and_publish(self, event):
        if len(self.drone_positions) < NUM_DRONES or len(self.target_positions) < NUM_TARGETS:
            rospy.loginfo_throttle(5, "Waiting for all drones and targets to report positions...")
            rospy.loginfo_throttle(5, f"[Target{target_id}] position: ({pos.x:.2f}, {pos.y:.2f})")
            return

        # Step 1: Run auction
        assignment = self.run_auction()
        print("assignment",assignment)

        # Step 2: Group by target
        grouped = {tid: [] for tid in range(NUM_TARGETS)}
        for drone_id, target_id in assignment.items():
            grouped[target_id].append(drone_id)

        # Step 3: For each drone, compute expected odom and publish
        for target_id, drone_list in grouped.items():
            total_agents = len(drone_list)
            target_pos = self.target_positions[target_id]

            for angle_index, drone_id in enumerate(drone_list):
                angle = 2 * math.pi * angle_index / total_agents

                # Desired position
                goal_x = target_pos[0] + RADIUS * math.cos(angle)
                goal_y = target_pos[1] + RADIUS * math.sin(angle)
                goal_z = 1.0

                # Face the target
                dx = target_pos[0] - goal_x
                dy = target_pos[1] - goal_y
                yaw = math.atan2(dy, dx)
                quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

                # Linear velocity to simulate encirclement (optional)
                vx = 0.5 * -math.sin(angle)
                vy = 0.5 * math.cos(angle)
                print(" For each drone, compute expected odom and publish")
                # Create and publish odom
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "world"

                pose_msg.pose.position.x = goal_x
                pose_msg.pose.position.y = goal_y
                pose_msg.pose.position.z = goal_z
                # 如果需要朝向：
                # pose_msg.pose.orientation = Quaternion(*quat)

                # 发布话题（注意：话题名要与 planner 订阅的一致）
                self.odom_publishers[drone_id].publish(pose_msg)
                rospy.loginfo(f"[Drone{drone_id}] encircle Target{target_id} at ({goal_x:.2f}, {goal_y:.2f})")

    def run_auction(self):
        """
        拍卖算法：每架无人机对最近目标出价，目标最多接受 MAX_DRONES_PER_TARGET 个出价。
        """
        rospy.loginfo("run_auction")
        values = {}
        for drone_id, drone_pos in self.drone_positions.items():
            values[drone_id] = {}
            for target_id, target_pos in self.target_positions.items():
                dist = self.euclidean(drone_pos, target_pos)
                values[drone_id][target_id] = -dist  # 越近越好

        assignments = {}
        target_bids = {j: [] for j in range(NUM_TARGETS)}
        unassigned = set(self.drone_positions.keys())

        while unassigned:
            drone_id = unassigned.pop()
            drone_values = values[drone_id]
            sorted_targets = sorted(drone_values.items(), key=lambda x: x[1], reverse=True)
            best_target, best_value = sorted_targets[0]
            second_best_value = sorted_targets[1][1] if len(sorted_targets) > 1 else -float('inf')
            bid = best_value - second_best_value + EPSILON

            target_bids[best_target].append((drone_id, bid))
            rospy.loginfo(f"Drone{drone_id} bids for Target{best_target}")

            # If too many bidders, remove lowest
            target_bids[best_target].sort(key=lambda x: x[1], reverse=True)
            if len(target_bids[best_target]) > MAX_DRONES_PER_TARGET:
                removed = target_bids[best_target].pop()
                unassigned.add(removed[0])
                # rospy.loginfo(f"Drone{removed[0]} was outbid from Target{best_target}")

        for target_id, bids in target_bids.items():
            for drone_id, _ in bids:
                assignments[drone_id] = target_id

        return assignments

    def euclidean(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])


if __name__ == "__main__":
    try:
        DroneAuctionEncirclement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
