# C:\Users\92151\Desktop\ros_hand\orca_core-main\orca_core-main\orca_core\hardware\waveshare_client.py

import time
import serial
import numpy as np
from typing import List, Tuple
from .motor_client import MotorClient


class WaveshareClient(MotorClient):
    """Waveshare ST3215-HS 总线舵机客户端"""

    def __init__(self, motor_ids: List[int], port: str, baudrate: int):
        self.motor_ids = motor_ids
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self._is_connected = False

    @property
    def is_connected(self) -> bool:
        """返回连接状态"""
        return self._is_connected and self.serial is not None and self.serial.is_open

    def connect(self):
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=8,
                parity='N',
                stopbits=1
            )
            self._is_connected = True
            print(f"Connected to Waveshare servos on {self.port} at {self.baudrate} baud")
        except Exception as e:
            self._is_connected = False
            raise Exception(f"Failed to connect to {self.port}: {e}")

    def disconnect(self):
        """断开连接"""
        if self.serial:
            self.serial.close()
        self._is_connected = False

    def _send_command(self, motor_id: int, cmd_type: int, params: bytes = b'') -> bytes:
        """发送指令并接收响应"""
        if not self.is_connected:
            return b''

        length = 4 + len(params)
        cmd = bytes([0x55, 0x55, motor_id, length - 2, cmd_type]) + params
        checksum = sum(cmd[2:]) & 0xFF
        cmd += bytes([checksum])

        self.serial.write(cmd)
        self.serial.flush()
        time.sleep(0.01)

        if self.serial.in_waiting > 0:
            return self.serial.read(10)
        return b''

    def set_torque_enabled(self, motor_ids: List[int], enabled: bool):
        """设置扭矩使能"""
        for motor_id in motor_ids:
            value = 0x01 if enabled else 0x00
            self._send_command(motor_id, 0x01, bytes([value]))

    def write_desired_pos(self, motor_ids: List[int], positions: np.ndarray):
        """写入目标位置"""
        for motor_id, pos in zip(motor_ids, positions):
            if pos is None or np.isnan(pos):
                continue

            pos_deg = pos * 180.0 / np.pi
            pos_raw = int(pos_deg * 100)
            pos_raw = max(0, min(36000, pos_raw))

            time_ms = 1000
            params = bytes([
                pos_raw & 0xFF,
                (pos_raw >> 8) & 0xFF,
                time_ms & 0xFF,
                (time_ms >> 8) & 0xFF
            ])
            self._send_command(motor_id, 0x03, params)

    def write_desired_current(self, motor_ids: List[int], currents: np.ndarray):
        """写入目标电流（不支持）"""
        pass

    def set_operating_mode(self, motor_ids: List[int], mode: int):
        """设置运行模式"""
        for motor_id in motor_ids:
            self._send_command(motor_id, 0x04, bytes([mode]))

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """读取位置、速度、电流"""
        positions = []
        velocities = []
        currents = []

        for motor_id in self.motor_ids:
            resp = self._send_command(motor_id, 0x02)

            if len(resp) >= 8:
                pos_raw = (resp[5] << 8) | resp[4]
                pos_deg = pos_raw / 100.0
                pos_rad = pos_deg * np.pi / 180.0
                positions.append(pos_rad)

                if len(resp) >= 10:
                    vel_raw = (resp[7] << 8) | resp[6]
                    vel_deg_s = vel_raw / 100.0
                    vel_rad_s = vel_deg_s * np.pi / 180.0
                    velocities.append(vel_rad_s)

                    cur_raw = (resp[9] << 8) | resp[8]
                    currents.append(float(cur_raw))
                else:
                    velocities.append(0.0)
                    currents.append(0.0)
            else:
                positions.append(0.0)
                velocities.append(0.0)
                currents.append(0.0)

        return np.array(positions), np.array(velocities), np.array(currents)

    def read_temperature(self) -> np.ndarray:
        """读取温度"""
        temperatures = []
        for motor_id in self.motor_ids:
            resp = self._send_command(motor_id, 0x02)
            if len(resp) >= 11:
                temperatures.append(float(resp[10]))
            else:
                temperatures.append(25.0)
        return np.array(temperatures)

    @property
    def requires_offset_calibration(self) -> bool:
        return False