#!/usr/bin/env python
import rospy
import math
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped

# ⭐ 新增：导入你自定义的消息
from traj_utils.msg import FormationAssignment, GroupInfo

NUM_DRONES = 24
NUM_TARGETS = 6
RADIUS = 0.8


class DroneDirectEncirclement:
    def __init__(self):
        rospy.init_node("drone_task_assigner")

        self.drone_positions = {}   # drone_id: (x, y, z)
        self.target_positions = {}  # target_id: (x, y, z)

        # 订阅无人机和目标位置信息
        for i in range(NUM_DRONES):
            rospy.Subscriber(f"/drone_{i}_visual_slam/odom",
                             Odometry, self.drone_callback, callback_args=i)

        for j in range(NUM_TARGETS):
            rospy.Subscriber(f"/target_{j}_visual_slam/odom",
                             Odometry, self.target_callback, callback_args=j)

        # ⭐ 仍然发布给你现有 planner 的目标点
        self.odom_publishers = [
            rospy.Publisher(f"/drone_{i}_target/odom", PoseStamped, queue_size=1)
            for i in range(NUM_DRONES)
        ]

        # ⭐ 仍然发布方向（如果你的 planner 用）
        self.dir_pub = [
            rospy.Publisher(f"/drone_{i}_rel/pose", PoseStamped, queue_size=1)
            for i in range(NUM_DRONES)
        ]

        # ⭐ 新增：发布小群编队与偏移
        self.assign_pub = rospy.Publisher(
            "/formation_assignment", FormationAssignment, queue_size=1
        )

        rospy.Timer(rospy.Duration(2.0), self.assign_and_publish)

    def drone_callback(self, msg, drone_id):
        pos = msg.pose.pose.position
        self.drone_positions[drone_id] = (pos.x, pos.y, pos.z)

    def target_callback(self, msg, target_id):
        pos = msg.pose.pose.position
        self.target_positions[target_id] = (pos.x, pos.y, pos.z)

    # -------------------------------
    # ⭐ 主要修改集中在这里
    # -------------------------------
    def assign_and_publish(self, event):
        if len(self.target_positions) < NUM_TARGETS:
            rospy.loginfo_throttle(5, "Waiting for all targets...")
            return

        # ================================
        # 1. 任务分配（你可替换成匈牙利算法）
        # ================================
        assignment = {
            0: 0,
            1: 0,
            2: 2,
            3: 2,
            4: 2,
            5: 0,
            6: 1,
            7: 1,
            8: 1,
            9: 1,
            10: 0,
            11: 2,
            12: 3,
            13: 3,
            14: 5,
            15: 5,
            16: 5,
            17: 3,
            18: 4,
            19: 4,
            20: 4,
            21: 4,
            22: 3,
            23: 5,
        }

        grouped = {tid: [] for tid in range(NUM_TARGETS)}
        for drone_id, target_id in assignment.items():
            grouped[target_id].append(drone_id)

        # ============================================
        # 2. 构造 FormationAssignment 消息
        # ============================================
        assign_msg = FormationAssignment()
        assign_msg.header.stamp = rospy.Time.now()
        assign_msg.num_groups = NUM_TARGETS
        group_msgs = []

        for target_id, drone_list in grouped.items():
            target_pos = self.target_positions[target_id]
            total_agents = len(drone_list)

            # ⭐ 自动生成 3D Fibonacci 阵型方向
            directions = self.fibonacci_sphere(total_agents)

            # -------------------------
            # 构造 GroupInfo 消息
            # -------------------------
            gmsg = GroupInfo()
            gmsg.group_id = target_id
            gmsg.members = drone_list

            # 每个成员对应的偏移（相对目标）
            for dx, dy, dz in directions:
                relpose = PoseStamped()
                relpose.pose.position.x = dx
                relpose.pose.position.y = dy
                relpose.pose.position.z = 0
                gmsg.formation_offsets.append(relpose)

            # 小群目标
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "world"
            goal.pose.position.x = target_pos[0]
            goal.pose.position.y = target_pos[1]
            goal.pose.position.z = target_pos[2]
            gmsg.goals.append(goal)

            group_msgs.append(gmsg)

        assign_msg.groups = group_msgs

        # ⭐ 发布给所有无人机（供 C++ formation manager 使用）
        self.assign_pub.publish(assign_msg)

        # ============================================
        # 3. （保持你的原功能）给每个 drone 发布目标点和方向
        # ============================================
        for target_id, drone_list in grouped.items():
            target_pos = self.target_positions[target_id]
            directions = self.fibonacci_sphere(len(drone_list))

            for dir_vector, drone_id in zip(directions, drone_list):
                dx, dy, dz = dir_vector

                # 发布目标点（保持你的原行为）
                goal_msg = PoseStamped()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.header.frame_id = "world"
                goal_msg.pose.position.x = target_pos[0]
                goal_msg.pose.position.y = target_pos[1]
                goal_msg.pose.position.z = target_pos[2]

                self.odom_publishers[drone_id].publish(goal_msg)

                # 发布方向
                dir_msg = PoseStamped()
                dir_msg.header.stamp = rospy.Time.now()
                dir_msg.header.frame_id = "world"
                dir_msg.pose.position.x = dx
                dir_msg.pose.position.y = dy
                dir_msg.pose.position.z = 0
                self.dir_pub[drone_id].publish(dir_msg)

    # =======================
    # Fibonacci 点集 (3D)
    # =======================
    def fibonacci_sphere(self, samples=1):
        points = []
        offset = 2.0 / samples
        increment = math.pi * (3.0 - math.sqrt(5.0))

        for i in range(samples):
            y = ((i * offset) - 1) + (offset / 2)
            r = math.sqrt(max(0.0, 1 - y * y))
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
