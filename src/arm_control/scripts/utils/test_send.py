#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
import time
import serial

class RobotDataSender:
    def __init__(self, port='/dev/ttyS0', baudrate=115200):
        # 协议格式（大端字节序）
        self.format_str = '>B3h3i3h4hI'  # B:STX, 3h:速度, 3i:位置, 3h:姿态, 4h:关节角, I:时间戳[3](@ref)
        
        # 字段类型映射表（索引对应数据元组顺序）
        self.field_types = [
            'B',        # 0: STX
            'h','h','h',# 1-3: VX/VY/VZ
            'i','i','i',# 4-6: PTX/PTY/PTZ
            'h','h','h',# 7-9: pitch/roll/HDG
            'h','h','h','h', # 10-13: Q0-Q3
            'I'         # 14: 时间戳
        ]

        # 数据标签配置（含转换参数）
        self.data_labels = [
            ('帧头', 'STX', 0xAA, 1),         # uint8直接发送
            ('速度X', 'VX', 0.01, 1e3),        # 放大1000倍转int16
            ('速度Y', 'VY', 0.102, 1e3),
            ('速度Z', 'VZ', 0.0, 1e3),
            ('目标位置X', 'PTX', 11.4074, 1e4),  # 放大10000倍转int32[10](@ref)
            ('目标位置Y', 'PTY', -15.01, 1e4),   # 支持负坐标
            ('目标位置Z', 'PTZ', -5.5, 1e4),     # 支持负坐标
            ('俯仰角', 'pitch', 2.22, 1e3),     # 放大1000倍转int16
            ('滚转角', 'roll', -2.5, 1e3),      # 支持负角度
            ('航向角', 'HDG', 1.8, 1e3),
            ('关节角0', 'Q0', 0.211, 1e3),
            ('关节角1', 'Q1', -0.15, 1e3),      # 支持负角度
            ('关节角2', 'Q2', 0.211, 1e3),
            ('关节角3', 'Q3', 0.211, 1e3)
        ]
        
        # 串口初始化
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"已连接串口: {self.ser.name}")
        except serial.SerialException as e:
            print(f"串口打开失败: {e}")
            exit(1)

    def _convert_value(self, label, index):
        """根据字段索引进行类型转换[7,9](@ref)"""
        field_type = self.field_types[index]
        raw_value = label[2]
        scale = label[3] if len(label)>3 else 1
        
        try:
            scaled = int(round(raw_value * scale))
            if field_type == 'h':  # int16检查
                if not (-32768 <= scaled <= 32767):
                    raise ValueError(f"{label[1]}={raw_value} 缩放后={scaled} 超出int16范围")
            elif field_type == 'i':  # int32检查
                if not (-2147483648 <= scaled <= 2147483647):
                    raise ValueError(f"{label[1]}={raw_value} 缩放后={scaled} 超出int32范围")
            return scaled
        except TypeError:
            raise ValueError(f"无效的数值类型: {label[1]}={raw_value}")

    def pack_data(self):
        """构造带时间戳的数据包[2,4](@ref)"""
        # 更新系统时间戳（秒级）
        timestamp = int(time.time())
        
        # 构造数据元组
        data_tuple = (
            self.data_labels[0][2],  # STX
            self._convert_value(self.data_labels[1], 1),
            self._convert_value(self.data_labels[2], 2),
            self._convert_value(self.data_labels[3], 3),
            self._convert_value(self.data_labels[4], 4),
            self._convert_value(self.data_labels[5], 5),
            self._convert_value(self.data_labels[6], 6),
            self._convert_value(self.data_labels[7], 7),
            self._convert_value(self.data_labels[8], 8),
            self._convert_value(self.data_labels[9], 9),
            self._convert_value(self.data_labels[10], 10),
            self._convert_value(self.data_labels[11], 11),
            self._convert_value(self.data_labels[12], 12),
            self._convert_value(self.data_labels[13], 13),
            timestamp  # uint32时间戳
        )
        
        try:
            return struct.pack(self.format_str, *data_tuple)
        except struct.error as e:
            print(f"数据打包失败: {e}")
            print(f"格式字符串: {self.format_str}")
            print(f"预期长度: {struct.calcsize(self.format_str)}字节")
            exit(1)

    def send_data(self):
        """带调试信息的周期发送"""
        try:
            packed = self.pack_data()
            self.ser.write(packed)
            
            # 调试输出
            print(f"\n[{time.strftime('%H:%M:%S')}] 数据包发送成功")
            print("参数验证:")
            for i, label in enumerate(self.data_labels):
                field_type = self.field_types[i]
                scaled = label[2] * (label[3] if len(label)>3 else 1)
                print(f"{label[1]:<8} {label[2]:<8} → {field_type}({int(scaled)})")
            print(f"时间戳: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")
            
        except Exception as e:
            print(f"发送失败: {e}")
            self.ser.close()
            exit(1)

if __name__ == "__main__":
    sender = RobotDataSender()
    
    try:
        while True:
            sender.send_data()
            time.sleep(0.1)  # 100ms发送周期
    except KeyboardInterrupt:
        sender.ser.close()
        print("串口已安全关闭")