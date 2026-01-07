#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.ctrlcomp import *
from FSM.FSM import *
from typing import Union, Optional
import numpy as np
import time

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHG
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmdGo
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo
from unitree_sdk2py.utils.crc import CRC

from common.command_helper import create_damping_cmd, create_zero_cmd, init_cmd_hg, MotorMode
from common.rotation_helper import get_gravity_orientation_real
from common.remote_controller import RemoteController, KeyMap
from config import Config


# -----------------------------
# Optional keyboard support
# -----------------------------
def _is_headless_linux() -> bool:
    """Rough check: headless if DISPLAY/WAYLAND_DISPLAY not set on Linux."""
    if sys.platform.startswith("linux"):
        disp = os.environ.get("DISPLAY", "")
        way = os.environ.get("WAYLAND_DISPLAY", "")
        return (disp.strip() == "") and (way.strip() == "")
    return False


try:
    # NOTE: pynput requires X11/GUI on Linux; will fail on headless/SSH without X forwarding
    from pynput import keyboard as pynput_keyboard  # type: ignore
except Exception as e:
    pynput_keyboard = None
    _PYNPUT_IMPORT_ERR = e
else:
    _PYNPUT_IMPORT_ERR = None


class KeyboardState:
    def __init__(self):
        self.pressed_keys = set()
        self.just_pressed = set()

    def update(self):
        self.just_pressed.clear()

    def is_just_pressed(self, char: str) -> bool:
        return char in self.just_pressed

    def press(self, char: str):
        if char not in self.pressed_keys:
            self.pressed_keys.add(char)
            self.just_pressed.add(char)

    def release(self, char: str):
        self.pressed_keys.discard(char)


class Controller:
    def __init__(self, config: Config, enable_keyboard: bool = True):
        self.config = config
        self.remote_controller = RemoteController()
        self.num_joints = config.num_joints
        self.control_dt = config.control_dt

        # -----------------------------
        # Keyboard init (optional)
        # -----------------------------
        self.kb_state = KeyboardState()
        self.enable_keyboard = enable_keyboard
        self._kb_listener = None

        # Disable keyboard automatically in headless Linux
        if self.enable_keyboard:
            if pynput_keyboard is None:
                print(f"[WARN] pynput unavailable; keyboard control disabled. Reason: {_PYNPUT_IMPORT_ERR}")
                self.enable_keyboard = False
            elif _is_headless_linux():
                print("[WARN] No DISPLAY/WAYLAND_DISPLAY detected (headless). Keyboard control disabled.")
                self.enable_keyboard = False

        if self.enable_keyboard:
            self._kb_listener = self.setup_keyboard_listener()

        # -----------------------------
        # DDS init
        # -----------------------------
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.mode_pr_ = MotorMode.PR
        self.mode_machine_ = 0

        self.lowcmd_publisher_ = ChannelPublisher(config.lowcmd_topic, LowCmdHG)
        self.lowcmd_publisher_.Init()

        self.lowstate_subscriber = ChannelSubscriber(config.lowstate_topic, LowStateHG)
        self.lowstate_subscriber.Init(self.LowStateHgHandler, 10)

        self.wait_for_low_state()
        init_cmd_hg(self.low_cmd, self.mode_machine_, self.mode_pr_)

        # -----------------------------
        # Buffers / state
        # -----------------------------
        self.policy_output_action = np.zeros(self.num_joints, dtype=np.float32)
        self.kps = np.zeros(self.num_joints, dtype=np.float32)
        self.kds = np.zeros(self.num_joints, dtype=np.float32)
        self.qj = np.zeros(self.num_joints, dtype=np.float32)
        self.dqj = np.zeros(self.num_joints, dtype=np.float32)
        self.quat = np.zeros(4, dtype=np.float32)
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.gravity_orientation = np.array([0, 0, -1], dtype=np.float32)

        self.state_cmd = StateAndCmd(self.num_joints)
        self.policy_output = PolicyOutput(self.num_joints)
        self.FSM_controller = FSM(self.state_cmd, self.policy_output)

        self.running = True
        self.counter_over_time = 0

    # -----------------------------
    # Keyboard
    # -----------------------------
    def setup_keyboard_listener(self):
        """Set up pynput keyboard listener. Requires GUI (X11/Wayland)."""
        if pynput_keyboard is None:
            return None

        def on_press(key):
            try:
                if hasattr(key, "char") and key.char:
                    self.kb_state.press(key.char)

                    # Skill commands via keyboard (same mapping as your original)
                    c = key.char
                    if c == "r":
                        self.state_cmd.skill_cmd = FSMCommand.PASSIVE
                    elif c == "0":
                        self.state_cmd.skill_cmd = FSMCommand.POS_RESET
                    elif c == "1":
                        self.state_cmd.skill_cmd = FSMCommand.LOCO
                    elif c == "2":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_1
                    elif c == "3":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_2
                    elif c == "4":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_3
                    elif c == "5":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_4
                    elif c == "6":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_5
                    elif c == "7":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_6
                    elif c == "8":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_7
                    elif c == "9":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_8
                    elif c == "p":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_9
                    elif c == "o":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_10
                    elif c == "i":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_11
                    elif c == "u":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_12
                    elif c == "y":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_13
                    elif c == "t":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_14
                    elif c == "m":
                        self.state_cmd.skill_cmd = FSMCommand.SKILL_15
            except Exception:
                pass

        def on_release(key):
            try:
                if hasattr(key, "char") and key.char:
                    self.kb_state.release(key.char)
            except Exception:
                pass

        listener = pynput_keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        print("[INFO] Keyboard listener started (pynput).")
        return listener

    def update_velocity_from_keyboard(self):
        """Update velocity command from keyboard (LOCO mode only)."""
        if not self.enable_keyboard:
            # keep behavior consistent
            self.kb_state.update()
            return

        vel_increment = 0.1

        if self.kb_state.is_just_pressed("w"):
            self.state_cmd.vel_cmd[0] += vel_increment
            print(f"\nW: Forward {self.state_cmd.vel_cmd[0]:.2f}")
        if self.kb_state.is_just_pressed("s"):
            self.state_cmd.vel_cmd[0] -= vel_increment
            print(f"\nS: Backward {self.state_cmd.vel_cmd[0]:.2f}")
        if self.kb_state.is_just_pressed("a"):
            self.state_cmd.vel_cmd[1] += vel_increment
            print(f"\nA: Left {self.state_cmd.vel_cmd[1]:.2f}")
        if self.kb_state.is_just_pressed("d"):
            self.state_cmd.vel_cmd[1] -= vel_increment
            print(f"\nD: Right {self.state_cmd.vel_cmd[1]:.2f}")
        if self.kb_state.is_just_pressed("q"):
            self.state_cmd.vel_cmd[2] += vel_increment
            print(f"\nQ: Rotate Left {self.state_cmd.vel_cmd[2]:.2f}")
        if self.kb_state.is_just_pressed("e"):
            self.state_cmd.vel_cmd[2] -= vel_increment
            print(f"\nE: Rotate Right {self.state_cmd.vel_cmd[2]:.2f}")
        if self.kb_state.is_just_pressed(" "):
            self.state_cmd.vel_cmd = np.zeros(3, dtype=np.float32)
            print("\nSpace: Reset velocities")

        self.kb_state.update()

    # -----------------------------
    # DDS handlers
    # -----------------------------
    def LowStateHgHandler(self, msg: LowStateHG):
        self.low_state = msg
        self.mode_machine_ = self.low_state.mode_machine
        self.remote_controller.set(self.low_state.wireless_remote)

    def LowStateGoHandler(self, msg: LowStateGo):
        self.low_state = msg
        self.remote_controller.set(self.low_state.wireless_remote)

    def send_cmd(self, cmd: Union[LowCmdGo, LowCmdHG]):
        cmd.crc = CRC().Crc(cmd)
        self.lowcmd_publisher_.Write(cmd)

    def wait_for_low_state(self):
        while self.low_state.tick == 0:
            time.sleep(self.config.control_dt)
        print("Successfully connected to the robot.")

    def zero_torque_state(self):
        print("Enter zero torque state.")
        print("Waiting for the start signal...")
        while self.remote_controller.button[KeyMap.start] != 1:
            create_zero_cmd(self.low_cmd)
            self.send_cmd(self.low_cmd)
            time.sleep(self.config.control_dt)

    # -----------------------------
    # Main control loop step
    # -----------------------------
    def run_once(self):
        loop_start_time = time.time()

        # Combination key states
        r1_pressed = self.remote_controller.is_button_pressed(KeyMap.R1)
        l1_pressed = self.remote_controller.is_button_pressed(KeyMap.L1)
        l2_pressed = self.remote_controller.is_button_pressed(KeyMap.L2)
        r2_pressed = self.remote_controller.is_button_pressed(KeyMap.R2)

        # Basic control (no combo)
        if self.remote_controller.is_button_pressed(KeyMap.X) and not (r1_pressed or l1_pressed or l2_pressed or r2_pressed):
            self.state_cmd.skill_cmd = FSMCommand.PASSIVE
        if self.remote_controller.is_button_pressed(KeyMap.A) and not (r1_pressed or l1_pressed or l2_pressed or r2_pressed):
            self.state_cmd.skill_cmd = FSMCommand.POS_RESET
        if self.remote_controller.is_button_pressed(KeyMap.B) and not (r1_pressed or l1_pressed or l2_pressed or r2_pressed):
            self.state_cmd.skill_cmd = FSMCommand.LOCO

        # R1 group
        if r1_pressed and not (l1_pressed or l2_pressed or r2_pressed):
            if self.remote_controller.is_button_pressed(KeyMap.X):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_1
            elif self.remote_controller.is_button_pressed(KeyMap.Y):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_2
            elif self.remote_controller.is_button_pressed(KeyMap.B):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_3

        # L1 group
        elif l1_pressed and not (r1_pressed or l2_pressed or r2_pressed):
            if self.remote_controller.is_button_pressed(KeyMap.X):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_4
            elif self.remote_controller.is_button_pressed(KeyMap.Y):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_5
            elif self.remote_controller.is_button_pressed(KeyMap.B):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_6
            elif self.remote_controller.is_button_pressed(KeyMap.A):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_7

        # L2 group
        elif l2_pressed and not (r1_pressed or l1_pressed or r2_pressed):
            if self.remote_controller.is_button_pressed(KeyMap.X):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_8
            elif self.remote_controller.is_button_pressed(KeyMap.Y):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_9
            elif self.remote_controller.is_button_pressed(KeyMap.B):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_10
            elif self.remote_controller.is_button_pressed(KeyMap.A):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_11

        # R2 group
        elif r2_pressed and not (r1_pressed or l1_pressed or l2_pressed):
            if self.remote_controller.is_button_pressed(KeyMap.X):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_12
            elif self.remote_controller.is_button_pressed(KeyMap.Y):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_13
            elif self.remote_controller.is_button_pressed(KeyMap.B):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_14
            elif self.remote_controller.is_button_pressed(KeyMap.A):
                self.state_cmd.skill_cmd = FSMCommand.SKILL_15

        # Gamepad velocity control
        self.state_cmd.vel_cmd[0] = self.remote_controller.ly
        self.state_cmd.vel_cmd[1] = self.remote_controller.lx * -1
        self.state_cmd.vel_cmd[2] = self.remote_controller.rx * -1

        # Keyboard velocity control (LOCO only)
        is_loco_mode = (self.FSM_controller.cur_policy.name == FSMStateName.LOCOMODE)
        if is_loco_mode:
            self.update_velocity_from_keyboard()
        else:
            self.kb_state.update()

        # Joint states
        for i in range(self.num_joints):
            self.qj[i] = self.low_state.motor_state[i].q
            self.dqj[i] = self.low_state.motor_state[i].dq

        # IMU
        quat = self.low_state.imu_state.quaternion  # w, x, y, z
        # NOTE: your original code did np.array([gyroscope]) -> shape (1,3). Fix to shape (3,)
        ang = self.low_state.imu_state.gyroscope
        ang_vel = np.array([ang[0], ang[1], ang[2]], dtype=np.float32)

        gravity_orientation = get_gravity_orientation_real(quat)

        self.state_cmd.q = self.qj.copy()
        self.state_cmd.dq = self.dqj.copy()
        self.state_cmd.gravity_ori = gravity_orientation.copy()
        self.state_cmd.ang_vel = ang_vel.copy()
        self.state_cmd.base_quat = quat

        # FSM / policy
        self.FSM_controller.run()
        policy_output_action = self.policy_output.actions.copy()
        kps = self.policy_output.kps.copy()
        # print("kps is",kps)
        kds = self.policy_output.kds.copy()
        # print("kds is",kds)

        # Build low cmd
        for i in range(self.num_joints):
            self.low_cmd.motor_cmd[i].q = float(policy_output_action[i])
            self.low_cmd.motor_cmd[i].qd = 0.0
            self.low_cmd.motor_cmd[i].kp = float(kps[i])
            self.low_cmd.motor_cmd[i].kd = float(kds[i])
            self.low_cmd.motor_cmd[i].tau = 0.0

        # Send command
        self.send_cmd(self.low_cmd)

        # Timing
        loop_end_time = time.time()
        delta_time = loop_end_time - loop_start_time
        if delta_time < self.control_dt:
            time.sleep(self.control_dt - delta_time)
            self.counter_over_time = 0
        else:
            print("control loop over time.")
            self.counter_over_time += 1


def main():
    config = Config()
    ChannelFactoryInitialize(0, config.net)

    # enable_keyboard=True 默认会在 headless 环境自动禁用（不再报 X 连接错误）
    controller = Controller(config, enable_keyboard=True)

    try:
        while True:
            controller.run_once()

            # Press Y button to exit (no combo keys)
            if controller.remote_controller.is_button_pressed(KeyMap.Y) and \
               not controller.remote_controller.is_button_pressed(KeyMap.R1) and \
               not controller.remote_controller.is_button_pressed(KeyMap.L1) and \
               not controller.remote_controller.is_button_pressed(KeyMap.L2) and \
               not controller.remote_controller.is_button_pressed(KeyMap.R2):
                print("\nY button pressed, exiting...")
                break

    except KeyboardInterrupt:
        pass
    finally:
        # Damping on exit
        create_damping_cmd(controller.low_cmd)
        controller.send_cmd(controller.low_cmd)
        print("Exit")


if __name__ == "__main__":
    main()
