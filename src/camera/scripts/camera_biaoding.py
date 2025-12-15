#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import numpy as np
import threading
import os
import time
from datetime import datetime
from geometry_msgs.msg import PointStamped

class HandEyeDataCollector:
    def __init__(self):
        rospy.init_node('hand_eye_data_collector', log_level=rospy.INFO)
        
        # 数据存储列表
        self.data_pairs = []            # 完整的数据点对
        
        # 当前数据缓存
        self.current_arm_pos = None
        self.current_camera_pos = None
        self.arm_data_ready = False
        self.camera_data_ready = False
        
        # 数据锁和同步控制
        self.data_lock = threading.Lock()
        self.exit_flag = False
        self.last_print_time = 0  # 控制打印频率
        
        # 订阅话题
        rospy.Subscriber('/arm_end_effector/relative_position', PointStamped, self.arm_callback)
        rospy.Subscriber('/filtered_cube_position', PointStamped, self.camera_callback)
        
        # 文件保存路径
        self.save_dir = os.path.join(os.path.expanduser('~'), 'hand_eye_calibration_data')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # 使用更稳定的输入监听方式
        self.start_input_listener()
        
        rospy.loginfo("手眼标定数据采集节点已启动")
        rospy.loginfo("数据保存目录: %s", self.save_dir)
        rospy.loginfo("按 'a' 保存当前数据点对")
        rospy.loginfo("按 's' 显示当前数据状态") 
        rospy.loginfo("按 'q' 退出并保存所有数据到TXT文件")
        rospy.loginfo("")  # 空行分隔[1,2](@ref)
    
    def start_input_listener(self):
        """启动输入监听线程"""
        try:
            self.input_listener_thread = threading.Thread(target=self._input_listener_worker)
            self.input_listener_thread.daemon = True
            self.input_listener_thread.start()
            rospy.loginfo("输入监听线程已启动")
        except Exception as e:
            rospy.logwarn("启动输入监听线程失败: %s", str(e))
            self._fallback_save_method()
    
    def arm_callback(self, msg):
        """机械臂末端位置回调函数"""
        with self.data_lock:
            self.current_arm_pos = [msg.point.x, msg.point.y, msg.point.z]
            self.arm_data_ready = True
            self._check_data_pair()
    
    def camera_callback(self, msg):
        """相机位置回调函数"""
        with self.data_lock:
            self.current_camera_pos = [msg.point.x, msg.point.y, msg.point.z]
            self.camera_data_ready = True
            self._check_data_pair()
    
    def _check_data_pair(self):
        """检查是否收到完整的数据点对"""
        if self.arm_data_ready and self.camera_data_ready:
            rospy.logdebug("收到完整数据点对")
            self.arm_data_ready = False
            self.camera_data_ready = False
            
            # 限制位置信息打印频率（最多每5秒打印一次）
            current_time = time.time()
            if current_time - self.last_print_time > 5.0:
                self.last_print_time = current_time
                rospy.loginfo("末端执行器位置: x=%.3f, y=%.3f, z=%.3f", 
                            self.current_arm_pos[0], self.current_arm_pos[1], self.current_arm_pos[2])
    
    def save_current_pair(self):
        """保存当前数据点对 - 使用统一的日志格式确保换行正常"""
        with self.data_lock:
            if self.current_arm_pos is not None and self.current_camera_pos is not None:
                timestamp = rospy.Time.now().to_sec()
                data_pair = {
                    'timestamp': timestamp,
                    'arm_position': self.current_arm_pos[:],  # 修改这里
                    'camera_position': self.current_camera_pos[:]  # 修改这里
                }
                self.data_pairs.append(data_pair)
                
                # 使用统一的日志输出，避免换行问题[1,2](@ref)
                rospy.loginfo("\n" + "="*50)
                rospy.loginfo("保存第 %d 个数据点对:", len(self.data_pairs))
                rospy.loginfo("机械臂: [%.6f, %.6f, %.6f]", 
                            self.current_arm_pos[0], self.current_arm_pos[1], self.current_arm_pos[2])
                rospy.loginfo("相机:   [%.6f, %.6f, %.6f]", 
                            self.current_camera_pos[0], self.current_camera_pos[1], self.current_camera_pos[2])
                rospy.loginfo("="*50 + "\n")
            else:
                rospy.logwarn("无法保存: 数据不完整")
    
    def show_status(self):
        """显示当前数据状态 - 使用格式化的输出"""
        with self.data_lock:
            rospy.loginfo("\n" + "="*50)
            rospy.loginfo("数据采集状态")
            rospy.loginfo("已保存点对数: %d", len(self.data_pairs))
            rospy.loginfo("当前机械臂数据: %s", self.current_arm_pos)
            rospy.loginfo("当前相机数据: %s", self.current_camera_pos)
            rospy.loginfo("="*50 + "\n")
    
    def save_to_txt(self):
        """保存所有数据到TXT文件"""
        if not self.data_pairs:
            rospy.logwarn("没有数据可保存")
            return False
        
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        txt_file = os.path.join(self.save_dir, "hand_eye_data_{}.txt".format(timestamp))
        
        try:
            with open(txt_file, 'w') as f:
                # 写入文件头
                f.write("# 手眼标定数据文件\n")
                f.write("# 格式: 时间戳 机械臂X 机械臂Y 机械臂Z 相机X 相机Y 相机Z\n")
                f.write("# 采集时间: {}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
                f.write("# 数据点数: {}\n\n".format(len(self.data_pairs)))
                
                # 写入数据，确保每行正常换行[1](@ref)
                for i, pair in enumerate(self.data_pairs):
                    f.write("{:.3f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
                        pair['timestamp'],
                        pair['arm_position'][0], pair['arm_position'][1], pair['arm_position'][2],
                        pair['camera_position'][0], pair['camera_position'][1], pair['camera_position'][2]
                    ))
            
            rospy.loginfo("\n数据已保存到: {}".format(txt_file))
            rospy.loginfo("共保存 {} 个数据点对\n".format(len(self.data_pairs)))
            return True
            
        except Exception as e:
            rospy.logerr("保存文件时出错: {}".format(str(e)))
            return False
    
    def _input_listener_worker(self):
        """输入监听工作线程 - 简化版本避免终端冲突"""
        try:
            # 使用更简单的方法检测输入，避免tty.setraw()的副作用[5](@ref)
            import select
            import sys
            
            rospy.loginfo("输入监听就绪，等待输入...")
            
            while not rospy.is_shutdown() and not self.exit_flag:
                # 使用select检测输入，避免阻塞[5](@ref)
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    try:
                        char = sys.stdin.read(1)
                        self._process_input(char)
                    except (IOError, KeyboardInterrupt):
                        break
                time.sleep(0.05)  # 小幅延迟减少CPU占用
                        
        except Exception as e:
            rospy.logwarn("输入监听异常: %s", str(e))
            rospy.loginfo("切换到自动保存模式")
            self._fallback_save_method()
    
    def _process_input(self, char):
        """处理输入字符"""
        if char == 'a' or char == 'A':
            self.save_current_pair()
        elif char == 's' or char == 'S':
            self.show_status()
        elif char == 'q' or char == 'Q':
            rospy.loginfo("退出信号接收，保存数据...")
            self.save_to_txt()
            self.exit_flag = True
            rospy.signal_shutdown("用户退出")
        elif char == '\n':  # 处理回车键
            pass  # 忽略回车键
        else:
            rospy.loginfo("未知命令: '%s' (可用命令: a-保存, s-状态, q-退出)", char)
    
    def _fallback_save_method(self):
        """备用保存方案"""
        rospy.loginfo("启动自动保存模式：每30秒检查一次")
        
        def periodic_check():
            last_save_time = time.time()
            save_interval = 30  # 30秒
            
            while not rospy.is_shutdown() and not self.exit_flag:
                current_time = time.time()
                
                # 每10秒显示状态
                if current_time - last_save_time >= 10:
                    with self.data_lock:
                        if self.data_pairs:
                            rospy.loginfo("当前已保存 %d 个数据点对", len(self.data_pairs))
                
                # 自动保存
                if current_time - last_save_time >= save_interval:
                    if self.data_pairs:
                        rospy.loginfo("自动保存数据...")
                        self.save_to_txt()
                    last_save_time = current_time
                
                time.sleep(1)
        
        save_thread = threading.Thread(target=periodic_check)
        save_thread.daemon = True
        save_thread.start()
    
    def run(self):
        """主循环"""
        rospy.loginfo("数据采集节点运行中...\n")
        
        try:
            rospy.spin()
        finally:
            # 节点关闭时自动保存数据
            self.exit_flag = True
            if self.data_pairs:
                rospy.loginfo("节点关闭，保存最终数据...")
                self.save_to_txt()

def main():
    try:
        collector = HandEyeDataCollector()
        collector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr("节点运行出错: %s", str(e))

if __name__ == '__main__':
    main()