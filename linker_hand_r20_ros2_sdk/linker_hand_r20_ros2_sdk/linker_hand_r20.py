#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy,time, threading, json, sys
from typing import List
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import String
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState

from .utils.dexterous_hand_controller import DexterousHandController, JOINT_DEFINITIONS


class LinkerHandR20(Node):

    def __init__(self):
        super().__init__('linker_hand_r20')
        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'right')
        self.declare_parameter('hand_joint', 'R20')
       

        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        # ros时间获取
        self.stamp_clock = Clock()
        self.ctl = DexterousHandController()
        self.ctl.connect() # 连接设备
        self.ctl.start_monitoring() # 开启监听线程
        self.hand_cmd_sub = self.create_subscription(JointState, f'/cb_{self.hand_type}_hand_control_cmd', self.hand_control_cb,10)
        self.hand_state_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state',10)
        self.hand_state_arc_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state_arc',10)
        self.hand_info_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_info', 10)
        self.motor_list = None
        self.position_to_motor_map = [0, 15, 5, 10, 6, 1, 16, 7, 2, 17, 8, 11, 12, 13, 14, 3, 18, 9, 4, 19]
        self.run_thread = threading.Thread(target=self._run, daemon=True)
        self.run_thread.start()
        if self.ctl.comm.is_connected:
            self.get_logger().info("Linker Hand R20 ROS2 SDK 连接成功")
        else:
            self.get_logger().error("Linker Hand R20 ROS2 SDK 连接失败")


    def hand_control_cb(self, msg):
        #print(msg, flush=True)
        position = list(msg.position)
        tmp_list = [position[i] for i in self.position_to_motor_map]
        # 删除预留元素
        motor_tmp = tmp_list[:11] + tmp_list[15:]
        # 将接收到的范围值转角度值
        self.motor_list = [self.uint8_to_angle(int(v), JOINT_DEFINITIONS[i]) for i, v in enumerate(motor_tmp)]
        

    def to_uint8(self, p, p_min, p_max):
        """角度值转范围值"""
        return int(round((p - p_min) / (p_max - p_min) * 255))
    
    def uint8_to_angle(self, val: int, joint) -> float:
        """按照对照表将范围值转角度值"""
        angle = joint.min_pos + (val / 255.0) * (joint.max_pos - joint.min_pos)
        return int(angle)
    
    def reorder_by_map(self, src: List, map_table: List[int], reverse: bool = False) -> List:
        """
        按给定映射表重排列表
        :param src: 原始列表（长度必须与映射表一致）
        :param map_table: 映射表（长度 20）
        :param reverse: False=正向（position→motor），True=反向（motor→position）
        :return: 重排后的新列表
        """
        if reverse:                       # 建逆映射
            inv = [0] * len(map_table)
            for from_idx, to_idx in enumerate(map_table):
                inv[to_idx] = from_idx
            map_table = inv

        return [src[map_table[i]] for i in range(len(map_table))]
    
    def _run(self):
        while True:
            if self.motor_list != None:
                self.ctl.set_joint_positions(self.motor_list+[0])
                time.sleep(0.05)
                self.motor_list = None
            
            tmp_range=[]
            tmp_angle = []
            for k, v in self.ctl.model.joints.items():
                if k < 17:
                    # print(f"K:{k} V:{v}", flush=True)
                    tmp_angle.append(v.current_pos)
                    tmp_range.append(self.to_uint8(v.current_pos, v.min_pos, v.max_pos))
            # 按照映射表重排为位置顺序motor→position_cmd
            state_range = self.reorder_by_map(tmp_range[:11] + [0, 0, 0, 0] + tmp_range[11:], self.position_to_motor_map, reverse=True)
            
            state_angle = self.reorder_by_map(tmp_angle[:11] + [0, 0, 0, 0] + tmp_angle[11:], self.position_to_motor_map, reverse=True)
            state_range_msg = self.joint_state_msg(pose=state_range)
            state_angle_msg = self.joint_state_msg(pose=state_angle)
            self.hand_state_pub.publish(state_range_msg)
            self.hand_state_arc_pub.publish(state_angle_msg)
            time.sleep(0.05)

    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["thumb_base", "index_base", "middle_base", "ring_base", "pinky_base",
"thumb_abd", "index_abd", "middle_abd", "ring_abd", "pinky_abd",
"thumb_rot", "rsv", "rsv", "rsv", "rsv",
"thumb_tip", "index_tip", "middle_tip", "ring_tip", "pinky_tip"]
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state
        



def main(args=None):
    rclpy.init(args=args)
    node = LinkerHandR20()
    try:
        #node.run()
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:      # 关闭 CAN 或其他硬件资源
        #node.close()
        #node.destroy_node()      # 销毁 ROS 节点
        #rclpy.shutdown()         # 关闭 ROS
        print("程序已退出。")
