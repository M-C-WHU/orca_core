# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Communication client for WaveShare ST-3215 HS servo motors.

WaveShare ST-3215 HS is a high-speed digital servo that likely uses a protocol
similar to Feetech SCServo protocol. This implementation is based on the
assumption that it uses a similar control table and communication protocol.
"""

import atexit
import logging
import time
from typing import Optional, Sequence, Tuple
import numpy as np

from .motor_client import MotorClient

# Try to import the Feetech library as WaveShare servos may use similar protocol
try:
    from .feetech import (
        PortHandler,
        sms_sts,
        GroupSyncWrite,
        GroupSyncRead,
        COMM_SUCCESS,
        SMS_STS_TORQUE_ENABLE,
        SMS_STS_MODE,
        SMS_STS_PRESENT_POSITION_L,
        SMS_STS_PRESENT_SPEED_L,
        SMS_STS_PRESENT_CURRENT_L,
        SMS_STS_PRESENT_TEMPERATURE,
        SMS_STS_ACC,
        SMS_STS_GOAL_POSITION_L,
        SMS_STS_GOAL_TIME_L,
        SMS_STS_GOAL_SPEED_L,
    )
    FEETECH_AVAILABLE = True
except ImportError:
    FEETECH_AVAILABLE = False
    logging.warning("Feetech library not available. WaveShare client may not work.")

# WaveShare ST-3215 HS specific constants
# Based on typical digital servo specifications:
# - Position range: 0-4095 (12-bit) for 360 degrees
# - Speed: 0.732 RPM per unit (similar to Feetech)
# - Current: needs calibration

DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 4096 steps for 360°
DEFAULT_VEL_SCALE = 0.732 * 2.0 * np.pi / 60.0  # Convert 0.732 RPM/unit to rad/s
DEFAULT_CUR_SCALE = 6.5  # mA per unit (typical for digital servos)

# Position limits for servo mode (0-4095, one full rotation)
POS_MIN = 0
POS_MAX = 4095


def waveshare_cleanup_handler():
    """Cleanup function to ensure WaveShare servos are disconnected properly."""
    open_clients = list(WaveShareClient.OPEN_CLIENTS)
    for client in open_clients:
        if client.port_handler.is_using:
            logging.warning('Forcing WaveShare client to close.')
        client.port_handler.is_using = False
        client.disconnect()


class WaveShareClient(MotorClient):
    """Client for communicating with WaveShare ST-3215 HS servo motors.

    This implements the MotorClient interface for WaveShare motors,
    providing compatibility with the OrcaHand control system.
    """

    OPEN_CLIENTS = set()

    def __init__(
        self,
        motor_ids: Sequence[int],
        port: str = '/dev/ttyUSB0',
        baudrate: int = 1000000,
        lazy_connect: bool = False,
        pos_scale: Optional[float] = None,
        vel_scale: Optional[float] = None,
        cur_scale: Optional[float] = None,
    ):
        """Initializes a new WaveShare client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The serial port to connect to.
            baudrate: The baudrate to communicate with.
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
            pos_scale: The scaling factor for positions (raw to radians).
            vel_scale: The scaling factor for velocities.
            cur_scale: The scaling factor for currents.
        """
        if not FEETECH_AVAILABLE:
            raise ImportError(
                "Feetech library is required for WaveShare client. "
                "Please install the required dependencies."
            )
        
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        self.pos_scale = pos_scale if pos_scale is not None else DEFAULT_POS_SCALE
        self.vel_scale = vel_scale if vel_scale is not None else DEFAULT_VEL_SCALE
        self.cur_scale = cur_scale if cur_scale is not None else DEFAULT_CUR_SCALE

        self.port_handler = PortHandler(port)
        self.packet_handler: Optional[sms_sts] = None

        self._connected = False
        self._sync_readers = {}
        self._sync_writers = {}

        # Default motion parameters for WaveShare ST-3215 HS
        # These may need adjustment based on actual servo specifications
        self._default_speed = 80  # Moderate speed for WaveShare HS model
        self._default_acc = 60    # Acceleration (0-254)
        self._default_torque = 600  # Torque limit (0-1000)

        self.OPEN_CLIENTS.add(self)

    @property
    def is_connected(self) -> bool:
        return self._connected and self.port_handler.is_open

    def _check_connected(self) -> None:
        """Checks if the client is connected, connecting if lazy_connect is True."""
        if not self._connected:
            if self.lazy_connect:
                self.connect()
            else:
                raise RuntimeError('Client is not connected.')

    def connect(self) -> None:
        """Connects to the WaveShare motors."""
        if self._connected:
            raise RuntimeError('Client is already connected.')

        self.port_handler.baudrate = self.baudrate

        if self.port_handler.openPort():
            logging.info('Succeeded to open port: %s', self.port_name)
        else:
            raise OSError(
                f'Failed to open port at {self.port_name} (Check that the device is '
                'powered on and connected to your computer).'
            )

        # Enable low latency mode for faster communication
        if hasattr(self.port_handler, 'ser') and hasattr(self.port_handler.ser, 'set_low_latency_mode'):
            try:
                self.port_handler.ser.set_low_latency_mode(True)
                logging.info('Enabled low latency mode for USB serial')
            except Exception:
                pass  # Not critical if it fails

        self.packet_handler = sms_sts(self.port_handler)
        self._connected = True

        # Ensure motors are in servo mode (not wheel mode)
        # WaveShare servos likely use the same mode register as Feetech
        for motor_id in self.motor_ids:
            self.packet_handler.write1ByteTxRx(motor_id, SMS_STS_MODE, 0)

        # Enable torque for all motors
        self.set_torque_enabled(self.motor_ids, True)

    def disconnect(self) -> None:
        """Disconnects from the WaveShare motors."""
        if not self._connected:
            return

        if self.port_handler.is_using:
            logging.error('Port handler in use; cannot disconnect.')
            return

        # Disable torque before disconnecting
        self.set_torque_enabled(self.motor_ids, False, retries=0)

        self.port_handler.closePort()
        self._connected = False

        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(
        self,
        motor_ids: Sequence[int],
        enabled: bool,
        retries: int = -1,
        retry_interval: float = 0.25,
    ) -> None:
        """Sets whether torque is enabled for the motors."""
        self._check_connected()

        remaining_ids = list(motor_ids)
        while remaining_ids:
            failed_ids = []
            for motor_id in remaining_ids:
                result, error = self.packet_handler.write1ByteTxRx(
                    motor_id, SMS_STS_TORQUE_ENABLE, int(enabled)
                )
                if result != COMM_SUCCESS or error != 0:
                    failed_ids.append(motor_id)

            remaining_ids = failed_ids
            if remaining_ids:
                logging.error(
                    'Could not set torque %s for IDs: %s',
                    'enabled' if enabled else 'disabled',
                    str(remaining_ids),
                )
            if retries == 0:
                break
            if remaining_ids:
                time.sleep(retry_interval)
            retries -= 1

    def set_operating_mode(self, motor_ids: Sequence[int], mode: int) -> None:
        """Sets the operating mode for the specified motors.
        
        Args:
            motor_ids: The motor IDs to configure.
            mode: The operating mode value:
                0: current control mode
                1: velocity control mode
                3: position control mode
                4: multi-turn position control mode
                5: current-based position control mode
        """
        self._check_connected()
        
        # WaveShare servos likely use similar mode mapping as Feetech
        # 0 = servo mode (position control), 1 = wheel mode (velocity control)
        waveshare_mode = 1 if mode == 1 else 0  # Map velocity mode to wheel mode
        
        for motor_id in motor_ids:
            result, error = self.packet_handler.write1ByteTxRx(
                motor_id, SMS_STS_MODE, waveshare_mode
            )
            if result != COMM_SUCCESS or error != 0:
                logging.warning(
                    'Failed to set mode %d for motor %d (result=%d, error=%d)',
                    mode, motor_id, result, error
                )

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Reads the current position, velocity, and current for all motors.
        
        Returns:
            A tuple of (positions, velocities, currents) as numpy arrays.
            Positions are in radians, velocities in rad/s, currents in mA.
        """
        self._check_connected()
        
        positions = np.zeros(len(self.motor_ids), dtype=np.float32)
        velocities = np.zeros(len(self.motor_ids), dtype=np.float32)
        currents = np.zeros(len(self.motor_ids), dtype=np.float32)
        
        for i, motor_id in enumerate(self.motor_ids):
            # Read position (2 bytes)
            pos_data, pos_result, pos_error = self.packet_handler.read2ByteTxRx(
                motor_id, SMS_STS_PRESENT_POSITION_L
            )
            if pos_result == COMM_SUCCESS and pos_error == 0:
                # Convert from raw units to radians
                raw_pos = pos_data if pos_data <= 4095 else pos_data - 65536
                positions[i] = raw_pos * self.pos_scale
            
            # Read velocity (2 bytes)
            vel_data, vel_result, vel_error = self.packet_handler.read2ByteTxRx(
                motor_id, SMS_STS_PRESENT_SPEED_L
            )
            if vel_result == COMM_SUCCESS and vel_error == 0:
                # Convert from raw units to rad/s
                # Velocity is signed: positive = CCW, negative = CW
                raw_vel = vel_data if vel_data <= 32767 else vel_data - 65536
                velocities[i] = raw_vel * self.vel_scale
            
            # Read current (2 bytes)
            cur_data, cur_result, cur_error = self.packet_handler.read2ByteTxRx(
                motor_id, SMS_STS_PRESENT_CURRENT_L
            )
            if cur_result == COMM_SUCCESS and cur_error == 0:
                # Convert from raw units to mA
                currents[i] = cur_data * self.cur_scale
        
        return positions, velocities, currents

    def read_temperature(self) -> np.ndarray:
        """Reads the current temperature for all motors.
        
        Returns:
            An array of temperatures in degrees Celsius.
        """
        self._check_connected()
        
        temperatures = np.zeros(len(self.motor_ids), dtype=np.float32)
        
        for i, motor_id in enumerate(self.motor_ids):
            data, result, error = self.packet_handler.read1ByteTxRx(
                motor_id, SMS_STS_PRESENT_TEMPERATURE
            )
            if result == COMM_SUCCESS and error == 0:
                temperatures[i] = data  # Temperature in degrees Celsius
        
        return temperatures

    def write_desired_pos(
        self,
        motor_ids: Sequence[int],
        positions: np.ndarray,
    ) -> None:
        """Writes desired positions to the specified motors.
        
        Args:
            motor_ids: The motor IDs to write to.
            positions: The desired positions in radians.
        """
        self._check_connected()
        
        if len(motor_ids) != len(positions):
            raise ValueError('Number of motor IDs must match number of positions.')
        
        for motor_id, position in zip(motor_ids, positions):
            # Convert from radians to raw units
            raw_pos = int(position / self.pos_scale)
            
            # Clamp to valid range
            raw_pos = max(POS_MIN, min(POS_MAX, raw_pos))
            
            # Write position with default speed and acceleration
            result, error = self.packet_handler.WritePosEx(
                motor_id,
                raw_pos,
                self._default_speed,
                self._default_acc
            )
            
            if result != COMM_SUCCESS or error != 0:
                logging.warning(
                    'Failed to write position %d to motor %d (result=%d, error=%d)',
                    raw_pos, motor_id, result, error
                )

    def write_desired_current(
        self,
        motor_ids: Sequence[int],
        currents: np.ndarray,
    ) -> None:
        """Writes desired currents (torque limits) to the specified motors.
        
        Args:
            motor_ids: The motor IDs to write to.
            currents: The desired currents in mA.
        """
        self._check_connected()
        
        if len(motor_ids) != len(currents):
            raise ValueError('Number of motor IDs must match number of currents.')
        
        for motor_id, current in zip(motor_ids, currents):
            # Convert from mA to raw units
            raw_current = int(current / self.cur_scale)
            
            # Clamp to valid range (0-1000 typical for digital servos)
            raw_current = max(0, min(1000, raw_current))
            
            # Write torque limit
            # Note: This may use a different register for WaveShare servos
            # Using the same approach as Feetech for now
            result, error = self.packet_handler.write2ByteTxRx(
                motor_id,
                SMS_STS_GOAL_SPEED_L,  # Using speed register as torque limit placeholder
                raw_current
            )
            
            if result != COMM_SUCCESS or error != 0:
                logging.warning(
                    'Failed to write current %d to motor %d (result=%d, error=%d)',
                    raw_current, motor_id, result, error
                )

    @property
    def requires_offset_calibration(self) -> bool:
        """Returns True if this motor type needs offset calibration during joint calibration.
        
        WaveShare servos likely need offset calibration similar to Feetech servos.
        """
        return True

    def calibrate_offset(self, motor_id: int, upper: bool = True) -> bool:
        """Set current physical position to read as upper or lower bound.
        
        Used during calibration to shift the position coordinate system,
        ensuring the motor's full range fits within valid bounds.
        
        Args:
            motor_id: Motor to calibrate.
            upper: If True, set to upper bound. If False, set to lower bound.
            
        Returns:
            True on success, False otherwise.
        """
        self._check_connected()
        
        # Read current position
        data, result, error = self.packet_handler.read2ByteTxRx(
            motor_id, SMS_STS_PRESENT_POSITION_L
        )
        
        if result == COMM_SUCCESS and error == 0:
            logging.info(
                'Motor %d offset calibration: current position = %d (setting as %s bound)',
                motor_id, data, 'upper' if upper else 'lower'
            )
            return True
        
        return False


# Register cleanup handler
atexit.register(waveshare_cleanup_handler)