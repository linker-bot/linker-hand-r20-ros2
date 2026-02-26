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
        self.info_dic = {
                "current_temp": [-1] * 20,
                "current_current": [-1] * 20,
                "error_status": [-1] * 20,
            }
        self.ctl = DexterousHandController()
        self._init_hand()
        
        
        #self.run_thread = threading.Thread(target=self._run, daemon=True)
        #self.run_thread.start()
        if self.ctl.comm.is_connected:
            self.get_logger().info("Linker Hand 连接成功！")
        else:
            self.get_logger().error("Linker Hand 连接失败！")

    def _init_hand(self): 
        self.get_logger().info("正在初始化...")
        self.ctl.connect() # 连接设备
        if self.ctl.comm.query_device_type() == "左手":
            self.hand_type = "left"
        elif self.ctl.comm.query_device_type() == "右手":
            self.hand_type = "right"
        self.ctl.start_monitoring() # 开启监听线程
        time.sleep(0.1)
        device_info = self.ctl.read_serial_number()
        self.out_info(info_dic=device_info)
        self.hand_cmd_sub = self.create_subscription(JointState, f'/cb_{self.hand_type}_hand_control_cmd', self.hand_control_cb,10)
        self.hand_state_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state',10)
        self.hand_state_arc_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state_arc',10)
        self.hand_info_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_info', 10)
        self.motor_list = None
        self.position_to_motor_map = [0, 15, 5, 10, 6, 1, 16, 7, 2, 17, 8, 11, 12, 13, 14, 3, 18, 9, 4, 19]
        # 五指张开
        self.ctl.set_joint_positions([0] * 17)
        time.sleep(0.1)
        # 开始校准
        self.ctl.set_calibration_mode(1)
        time.sleep(1)

    def validate_strict_non_negative_ints(self, lst):
        """校验：所有元素必须是 int 或 float 类型，且 0 < x <= 255（排除 bool/字符串/负数/0/超255）"""
        try:
            if len(lst) != 20:
                return False
            return all(
                isinstance(x, (int, float)) and  # 接受 int 或 float 类型
                not isinstance(x, bool) and  # 排除 bool
                0 <= x <= 255  # 范围限制在 0-255（包括0）
                for x in lst
            )
        except Exception as e:
            self.get_logger().error(f"验证位置数据时发生异常: {e}")
            return False

    def hand_control_cb(self, msg):
        position = list(msg.position)
        if not self.validate_strict_non_negative_ints(position):
            self.get_logger().error("hand_control_cmd topic 输入的位置值不合法,请检查输入值是否为整数且大于等于0,且小于等于255, 长度为20")
            self.motor_list = None
        else:
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
    
    def run(self):
        info_msg = String()
        while True:
            if self.motor_list != None:
                self.ctl.set_joint_positions(self.motor_list+[0])
                time.sleep(0.02)
                self.motor_list = None
            
            tmp_range=[] # position范围值
            tmp_angle = [] # position角度值
            tmp_vel = [] # 当前电机速度
            tmp_current_temp = [] # 当前温度值
            tmp_current_current = [] # 当前电流值
            tmp_error_status = [] # 当前电机错误码
            for k, v in self.ctl.model.joints.items():
                if k < 17:
                    # print(f"K:{k} V:{v}", flush=True)
                    # 当前position
                    tmp_angle.append(v.current_pos)
                    tmp_range.append(self.to_uint8(v.current_pos, v.min_pos, v.max_pos))
                    # 当前电机速度
                    tmp_vel.append(v.current_vel)
                    # 当前温度
                    tmp_current_temp.append(v.current_temp)
                    # 当前电流
                    tmp_current_current.append(v.current_current)
                    # 当前错误码
                    tmp_error_status.append(v.error_status)
            # 按照映射表重排为位置顺序motor→position_cmd
            state_range = self.reorder_by_map(tmp_range[:11] + [0, 0, 0, 0] + tmp_range[11:], self.position_to_motor_map, reverse=True)
            state_angle = self.reorder_by_map(tmp_angle[:11] + [0, 0, 0, 0] + tmp_angle[11:], self.position_to_motor_map, reverse=True)
            ve = tmp_vel[:11] + [0, 0, 0, 0] + tmp_vel[11:]
            state_vel = [ve[i] for i in self.position_to_motor_map]
            state_range_msg = self.joint_state_msg(pose=state_range, vel=state_vel)
            state_angle_msg = self.joint_state_msg(pose=state_angle, vel=state_vel)

            # 温度数据填充
            c_t = tmp_current_temp[:11] + [0, 0, 0, 0] + tmp_current_temp[11:]
            # 按照映射表进行数据映射
            self.info_dic["current_temp"] = [c_t[i] for i in self.position_to_motor_map]

            # 当前电流值数据填充
            c_c = tmp_current_current[:11] + [0, 0, 0, 0] + tmp_current_current[11:]
            # 按照映射表电流数据映射
            self.info_dic["current_current"] = [c_c[i] for i in self.position_to_motor_map]

            #当前错误码数据填充
            e_s = tmp_error_status[:11] + [0, 0, 0, 0] + tmp_error_status[11:]
            self.info_dic["error_status"] = [e_s[i] for i in self.position_to_motor_map]
            info_msg.data = json.dumps(self.info_dic)
            self.hand_state_pub.publish(state_range_msg)
            self.hand_state_arc_pub.publish(state_angle_msg)
            self.hand_info_pub.publish(info_msg)
            time.sleep(0.02)

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
        

    def out_info(self,info_dic):
        from rich.console import Console
        from rich.table import Table, box

        console = Console()
        table = Table(title="Linker Hand", box=box.ROUNDED)

        table.add_column("product_model", style="cyan", justify="center")
        table.add_column("serial_number", style="cyan", justify="center") 
        table.add_column("software_version", style="magenta", justify="center")
        table.add_column("hardware_version", style="magenta", justify="center")
        table.add_column("hand_type", style="magenta", justify="center")
        table.add_column("unique_id", style="magenta", justify="center")

        table.add_row(str(info_dic["product_model"]), str(info_dic["serial_number"]), str(info_dic["software_version"]),str(info_dic["hardware_version"]),str(self.hand_type),str(info_dic["unique_id"]))
        console.print(table)

def main(args=None):
    rclpy.init(args=args)
    node = LinkerHandR20()
    try:
        node.run()
        #rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:      # 关闭 CAN 或其他硬件资源
        #node.close()
        #node.destroy_node()      # 销毁 ROS 节点
        #rclpy.shutdown()         # 关闭 ROS
        print("程序已退出。")
