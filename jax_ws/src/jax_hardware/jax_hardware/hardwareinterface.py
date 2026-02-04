#!/usr/bin/env python3
import numpy as np
import math as m
import logging

class HardwareInterface():
    def __init__(self, link):
        self.pwm_max = 2400
        self.pwm_min = 370
        self.link = link
        self.servo_angles = np.zeros((3,4))
        
        try:
            from adafruit_servokit import ServoKit
            self.kit = ServoKit(channels=16)
            self.hardware_connected = True
            self.create()
        except Exception as e:
            self.kit = None
            self.hardware_connected = False
            print(f"PC Mode: {e}")
        
        self.pins = np.array([[14,10,2,6], [13,9,1,5], [12,8,0,4]])
        self.servo_multipliers = np.array([[-1, 1, 1, -1], [1, -1, 1, -1], [1, 1, 1, 1]])
        self.complementary_angle = np.array([[180, 0, 0, 180], [0, 180, 0, 180], [0, 0, 0, 0]])
        self.physical_calibration_offsets = np.array([[90, 90, 90, 90], [90, 90, 90, 90], [90, 90, 90, 90]])

    def create(self):
        if self.hardware_connected and self.kit:
            for i in range(16):
                self.kit.servo[i].actuation_range = 180
                self.kit.servo[i].set_pulse_width_range(self.pwm_min, self.pwm_max)

    def set_actuator_postions(self, joint_angles):
        possible_joint_angles = impose_physical_limits(joint_angles)
        self.joint_angles_to_servo_angles(possible_joint_angles)

        for leg_index in range(4):
            for axis_index in range(3):
                target_angle = self.servo_angles[axis_index, leg_index]
                pin = self.pins[axis_index, leg_index]
                if self.hardware_connected and self.kit:
                    try:
                        self.kit.servo[pin].angle = target_angle
                    except:
                        pass

    def joint_angles_to_servo_angles(self, joint_angles):
        for leg in range(4):
            THETA2, THETA3 = joint_angles[1:, leg]
            THETA0 = lower_leg_angle_to_servo_angle(self.link, m.pi/2 - THETA2, THETA3 + np.pi/2)

            self.servo_angles[0, leg] = m.degrees(joint_angles[0, leg]) 
            self.servo_angles[1, leg] = m.degrees(THETA2) 
            
            # --- THE FIX ---
            # Convert to degrees and normalize
            raw_deg = m.degrees(THETA0) % 360
            if raw_deg > 180:
                raw_deg = 360 - raw_deg
            
            self.servo_angles[2, leg] = raw_deg

        self.servo_angles = self.servo_angles + self.physical_calibration_offsets
        self.servo_angles = np.multiply(self.servo_angles, self.servo_multipliers) + self.complementary_angle
        self.servo_angles = np.round(np.clip(self.servo_angles, 0, 180), 1)

def calculate_4_bar(th2, a, b, c, d):
    x_b = a * np.cos(th2)
    y_b = a * np.sin(th2)
    f = np.sqrt((d - x_b)**2 + y_b**2)
    ratio = (f**2 + c**2 - b**2) / (2 * f * c)
    beta = np.arccos(np.clip(ratio, -1.0, 1.0))
    gamma = np.arctan2(y_b, d - x_b)
    th4 = np.pi - gamma - beta
    x_c = c * np.cos(th4) + d
    y_c = c * np.sin(th4)
    th3 = np.arctan2((y_c - y_b), (x_c - x_b))
    return np.pi - th2 + th3, th4 - th3, np.pi * 2 - th2 - (np.pi - th2 + th3) - (th4 - th3)

def lower_leg_angle_to_servo_angle(link, THETA2, THETA3):
    GDE, DEF, EFG = calculate_4_bar(THETA3 + link.lower_leg_bend_angle, link.i, link.h, link.f, link.g)
    CDH = 1.5 * np.pi - THETA2 - GDE - link.EDC
    CDA = CDH + link.gamma
    DAB, ABC, BCD = calculate_4_bar(CDA, link.d, link.a, link.b, link.c)
    return DAB + link.gamma

def impose_physical_limits(desired_joint_angles):
    possible_joint_angles = np.zeros((3,4))
    for i in range(4):
        hip, upper, lower = np.degrees(desired_joint_angles[:, i])
        hip = np.clip(hip, -20, 20)
        upper = np.clip(upper, 0, 120)
        if upper < 20: lower = np.clip(lower, -40, 40)
        elif upper >= 110: lower = np.clip(lower, -70, -60)
        else: lower = np.clip(lower, -70, 0)
        possible_joint_angles[:, i] = hip, upper, lower
    return np.radians(possible_joint_angles)