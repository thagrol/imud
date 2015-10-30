#!/usr/bin/env python

##    imud.py
##    Copyright (C) 2015 Simon Osborne
##
##    This program is free software: you can redistribute it and/or modify
##    it under the terms of the GNU Affero General Public License as
##    published by the Free Software Foundation, either version 3 of the
##    License, or (at your option) any later version.
##
##    This program is distributed in the hope that it will be useful,
##    but WITHOUT ANY WARRANTY; without even the implied warranty of
##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##    GNU Affero General Public License for more details.
##
##    You should have received a copy of the GNU Affero General Public License
##    along with this program.  If not, see <http://www.gnu.org/licenses/>.

import argparse
import atexit
import logging
import math
import os
import select
import smbus
import socket
import sys
import threading
import time

from RPi.GPIO import RPI_REVISION

# Choose i2c bus according to PI revision
if RPI_REVISION == 1:
    # Rev 1 Pi
    I2CBUS = 0
else:
    # Everything else
    I2CBUS = 1

# BerryIMU adresses, registers, etc
# Accelerometer & magnetometer
IMU_ACCMAG = 0x1e # device address
CTRL_REG1_XM = 0x20
CTRL_REG2_XM = 0x21
CTRL_REG3_XM = 0x22
CTRL_REG4_XM = 0x23
CTRL_REG5_XM = 0x24 # mag
CTRL_REG6_XM = 0x25 # mag
CTRL_REG7_XM = 0x26 # mag
# accelerometer output registers
OUT_X_L_A = 0x28
OUT_X_H_A = 0x29
OUT_Y_L_A = 0x2a
OUT_Y_H_A = 0x2b
OUT_Z_L_A = 0x2c
OUT_Z_H_A = 0x2d
# magnetometer output registers
OUT_X_L_M = 0x08
OUT_X_H_M = 0x09
OUT_Y_L_M = 0x0a
OUT_Y_H_M = 0x0b
OUT_Z_L_M = 0x8c
OUT_Z_H_M = 0x8d
# fifo
FIFO_CTRL_REG = 0x2e

# accelerometer control reg1 settings
# add one of each group to get final value
# see datasheet for more info
XM_REG1_AODR_POWER_DOWN = 0b00000000
XM_REG1_AODR_3HZ125     = 0b00010000
XM_REG1_AODR_6HZ25      = 0b00100000
XM_REG1_AODR_12HZ5      = 0b00110000
XM_REG1_AODR_25HZ       = 0b01000000
XM_REG1_AODR_50HZ       = 0b01010000
XM_REG1_AODR_100HZ      = 0b01100000
XM_REG1_AODR_200HZ      = 0b01110000
XM_REG1_AODR_400HZ      = 0b10000000
XM_REG1_AODR_800HZ      = 0b10010000
XM_REG1_AODR_1600HZ     = 0b10100000
XM_REG1_BDU_ON          = 0b00001000
XM_REG1_AZEN            = 0b00000100
XM_REG1_AYEN            = 0b00000010
XM_REG1_AXEN            = 0b00000001
# accelerometer control reg2 settings
# add one of each group to get final value
# see datasheet for more info
XM_RGE2_ABW_773HZ       = 0b00000000
XM_REG2_ABW_194HZ       = 0b01000000
XM_REG2_ABW_362HZ       = 0b10000000
XM_REG2_ABW_50HZ        = 0b11000000
XM_REG2_AFS_2G          = 0b00000000
XM_REG2_AFS_4G          = 0b00001000
XM_REG2_AFS_6G          = 0b00010000
XM_REG2_AFS_8G          = 0b00011000
XM_REG2_AFS_16G         = 0B00100000
# magnetometer control reg5 settings
# add one of each group to get final value
# see datasheet for more info
XM_REG5_TEMP_ON         = 0b10000000
XM_REG5_TEMP_OFF        = 0b00000000
XM_REG5_RES_LOW         = 0b00000000
XM_REG5_RES_HIGH        = 0b01100000
XM_REG5_ODR_3HZ125      = 0b00000000
XM_REG5_ODR_6HZ25       = 0b00000100
XM_REG5_ODR_12HZ5       = 0b00001000
XM_REG5_ODR_25HZ        = 0b00001100
XM_REG5_ODR_50HZ        = 0b00010000
XM_REG5_ODR_100HZ       = 0b00010100 # SEE DATASHEET
# magnetometer control reg6 settings
# see datasheet for more info
XM_REG6_MFS_2           = 0b00000000
XM_REG6_MFS_4           = 0b00100000
XM_REG6_MFS_8           = 0b01000000
XM_REG6_MFS_12          = 0b01100000
# magnetometer control reg7 settings
# see datasheet for more info
XM_REG7_MD_CONT         = 0b00000000
XM_REG7_MD_SINGLE       = 0b00000001
XM_REG7_MD_POWERDOWN    = 0b00000011 # or 0b00000010
# fifo control reg settings
# see datasheet for more info
# also used for gyro
FIFO_BYPASS     = 0b00000000
FIFO_STREAM     = 0b01000000

# defaults for above
XM_REG1_DEFAULT = (XM_REG1_AODR_50HZ +
                   XM_REG1_BDU_ON +
                   XM_REG1_AZEN +
                   XM_REG1_AYEN +
                   XM_REG1_AXEN)
XM_REG2_DEFAULT = XM_RGE2_ABW_773HZ + XM_REG2_AFS_16G
XM_REG5_DEFAULT = XM_REG5_TEMP_OFF + XM_REG5_RES_HIGH + XM_REG5_ODR_50HZ
XM_REG6_DEFAULT = XM_REG6_MFS_12
XM_REG7_DEFAULT = XM_REG7_MD_CONT
FIFO_CTRL_DEFAULT = FIFO_BYPASS

# gyro
IMU_GYRO = 0x6A
# control registers
CTRL_REG1_G = 0x20
CTRL_REG2_G = 0x21
CTRL_REG3_G = 0x22
CTRL_REG4_G = 0x23
CTRL_REG5_G = 0x24
# output registers
OUT_X_L_G = 0x28
OUT_X_H_G = 0x29
OUT_Y_L_G = 0x2a
OUT_Y_H_G = 0x2b
OUT_Z_L_G = 0x2c
OUT_Z_H_G = 0x2d
#fifo
FIFO_CTRL_REG_G = 0x2e
# settings & flags
# CTRL_REG1_G, usage as above
CTRL_REG1_G_PD_ENABLE  = 0b00000000
CTRL_REG1_G_PD_DISABLE = 0b00001000
CTRL_REG1_G_ZEN        = 0b00000100
CTRL_REG1_G_YEN        = 0b00000010
CTRL_REG1_G_XEN        = 0b00000001
CTRL_REG1_G_SLEEP      = 0b00001000
# CTRL_REG2_G, usage as above
CTRL_REG2_G_HPM_NORMAL = 0b00000000
# CTRL_REG4_G, usage as above
CTRL_REG4_G_BDU_ENABLE = 0b10000000
CTRL_REG4_G_BDU_DISABLE= 0b00000000
CTRL_REG4_G_FS_245     = 0b00000000
CTRL_REG4_G_FS_500     = 0b00010000
CTRL_REG4_G_FS_2000    = 0b00110000
G_GAIN_OPTS = {CTRL_REG4_G_FS_245:0.00875, CTRL_REG4_G_FS_500:0.0175,
               CTRL_REG4_G_FS_2000:0.07, 0b00100000:0.07}
# defaults for above
FIFO_CTRL_REG_G_DEFAULT = FIFO_BYPASS
CTRL_REG1_G_DEFAULT = (CTRL_REG1_G_PD_DISABLE +
                       CTRL_REG1_G_ZEN +
                       CTRL_REG1_G_YEN +
                       CTRL_REG1_G_XEN)
CTRL_REG2_G_DEFAULT = CTRL_REG2_G_HPM_NORMAL
CTRL_REG4_G_DEFAULT = CTRL_REG4_G_BDU_ENABLE + CTRL_REG4_G_FS_2000
# calculate gain
G_GAIN = G_GAIN_OPTS[CTRL_REG4_G_DEFAULT & CTRL_REG4_G_FS_2000]
G_MINTIME = 0.05 # minimum interval between gyro checks (ms)

# pressure
IMU_PRESS = 0x77
# registers
P_ID = 0xd0
P_SOFT_RESET_REG = 0xe0
P_CTRL_MEAS = 0xf4
P_OUT_MSB = 0xf6
P_OUT_LSB = 0xf7
P_OUT_XLSB = 0xf8
# calibration data
P_AC1_H = 0xaa
P_AC1_L = 0xab
P_AC2_H = 0xac
P_AC2_L = 0xad
P_AC3_H = 0xae
P_AC3_L = 0xaf
P_AC4_H = 0xb0
P_AC4_L = 0xb1
P_AC5_H = 0xb2
P_AC5_L = 0xb3
P_AC6_H = 0xb4
P_AC6_L = 0xb5
P_B1_H = 0xb6
P_B1_L = 0xb7
P_B2_H = 0xb8
P_B2_L = 0xb9
P_MB_H = 0xba
P_MB_L = 0xbb
P_MC_H = 0xbc
P_MC_L = 0xbd
P_MD_H = 0xbe
P_MD_L = 0xbf
P_CAL_BASE = P_AC1_H
P_CAL_LEN = 22
# ctrl_meas bit masks
P_CTRL_MEAS_SCO = 0b00100000
# commands
P_SOFT_RESET = 0xb6
P_READ_TEMP = 0x2e
P_READ_PRESS_ULP = 0x34
P_READ_PRESS_STD = 0x74
P_READ_PRESS_HR = 0xb4
P_READ_PRESS_UHR = 0xf4
# settings & flags
P_READ_DELAY = {P_READ_TEMP: 0.0045,
                P_READ_PRESS_ULP: 0.0045,
                P_READ_PRESS_STD: 0.0075,
                P_READ_PRESS_HR: 0.0135,
                P_READ_PRESS_UHR: 0.0255}

# Sea level temp and pressure reference values
# based on ICAO 1964
MSL_PRESSURE = 1013.25 # hPa
MSL_TEMP = 15.0 # degC

# socket server
DEFAULT_PORT = 7691
LISTEN_HOST = ''

# broadcast server
BROADCAST_ADDRESS = '<broadcast>'
BROADCAST_INTERVAL = 1.0

#MISC
AG_FILTER = 0.80
CARDINALS_N_S_E_W = 4
CARDINALS_N_NE_ETC = 8
CARDINALS_N_NNE_ETC = 16
LOG_FILE = '/var/log/imud.log'
FALLBACK_LOG_FILE = '/tmp/imud.log'
PID_FILE = '/var/run/imud.pid'
FALLBACK_PID_FILE = '/tmp/imud.pid'
CMD_VERSION = 'v0.1\n'

# globals - need to be visible to all threads
acc_raw = {'x': 0.0, 'y': 0.0, 'z': 0.0}
acc_deg = {'x': 0.0, 'y': 0.0, 'z': 0.0}
gyro_raw = {'x': 0.0, 'y': 0.0, 'z': 0.0}
gyro_dps = {'x': 0.0, 'y': 0.0, 'z': 0.0}
gyro_angles = {'x': 0.0, 'y': 0.0, 'z': 0.0}
mag_raw = {'x': 0.0, 'y': 0.0, 'z': 0.0}
mag_comp = {'x': 0.0, 'y': 0.0, 'z': 0.0}
combined_angles = {'x': 0.0, 'y': 0.0, 'z': 0.0}
heading_raw = None
heading_comp = None
pressure = None
temp_p = None
b5 = 0 # scratch pad for temp/pressure calculations
temp_xm = None
p_calibration = {}

# locks
acc_raw_lock = threading.Lock()
acc_deg_lock = threading.Lock()
gyro_raw_lock = threading.Lock()
gyro_dps_lock = threading.Lock()
gyro_angles_lock = threading.Lock()
mag_raw_lock = threading.Lock()
mag_comp_lock = threading.Lock()
heading_raw_lock = threading.Lock()
heading_comp_lock = threading.Lock()
combined_angles_lock = threading.Lock()
p_device_lock = threading.Lock()

# flags
imu_inverted = False
acc_enabled = False
mag_enabled = False
gyro_enabled = False
press_enabled = False
stop_threads = False

# Misc
active_i2cbus = None


# start of function definitions

# Accelerometer stuff
def start_acc(bus=active_i2cbus, device=IMU_ACCMAG, flags=XM_REG1_DEFAULT,
              sense=XM_REG2_DEFAULT, fifo=FIFO_CTRL_DEFAULT):
    """configure and enable the accelerometer"""
    if bus is not None:
        try:
            # enable accelerometer
            bus.write_byte_data(device, CTRL_REG1_XM, flags)
            bus.write_byte_data(device, CTRL_REG2_XM, sense)
            bus.write_byte_data(device, FIFO_CTRL_REG, fifo)
        except IOError as e:
            logging.error('Unable to configure accelerometer: [%s] %s' %
                          (e.errno, e.strerror))
            return False
        return True
    return acc_enabled

def stop_acc(bus, device=IMU_ACCMAG):
    """disable accelerometer"""
    if bus is not None:
        try:
            bus.write_byte_data(device, CTRL_REG1_XM, XM_REG1_AODR_POWER_DOWN)
        except IOError:
            pass
        return False
    return acc_enabled
    
def read_raw_acc(bus, device=IMU_ACCMAG):
    """Read raw accelerometer data"""
    global acc_raw
    if bus is not None and device is not None:
        # read raw data
        try:
            x_l = bus.read_byte_data(device, OUT_X_L_A)
            x_h = bus.read_byte_data(device, OUT_X_H_A)
            y_l = bus.read_byte_data(device, OUT_Y_L_A)
            y_h = bus.read_byte_data(device, OUT_Y_H_A)
            z_l = bus.read_byte_data(device, OUT_Z_L_A)
            z_h = bus.read_byte_data(device, OUT_Z_H_A)
        except IOError as e:
            logging.error('Unable to read accelerometer: ', e.strerror)
            return
        # update globals
        with acc_raw_lock:
            acc_raw['x'] = (x_l | x_h << 8)
            acc_raw['y'] = (y_l | y_h << 8)
            acc_raw['z'] = (z_l | z_h << 8)
            if acc_raw['x'] >= 32768:
                acc_raw['x'] -= 65536
            if acc_raw['y'] >= 32768:
                acc_raw['y'] -= 65536
            if acc_raw['z'] >= 32768:
                acc_raw['z'] -= 65536

def calc_acc_angles():
    """convert raw acelerometer data to degrees"""
    global acc_deg
    with acc_raw_lock:
        with acc_deg_lock:
            acc_deg['x'] = math.degrees(math.atan2(acc_raw['y'], acc_raw['z']) + math.pi)
            acc_deg['y'] = math.degrees(math.atan2(acc_raw['z'], acc_raw['x']) + math.pi)
            # not sure if this next line will give usefull data
            acc_deg['z'] = math.degrees(math.atan2(acc_raw['x'], acc_raw['y']) + math.pi)

            if imu_inverted:
                if acc_deg['x'] > 180:
                    acc_deg['x'] -= 360.0
                acc_deg['y'] -= 90.0
                if acc_deg['y'] > 180:
                    acc_deg['y'] -= 360.0
            else:
                acc_deg['x'] -= 180.0
                if acc_deg['y'] > 90:
                    acc_deg['y'] -= 270.0
                else:
                    acc_deg['y'] += 90.0
    
# magnetometer stuff
def start_mag(bus, device=IMU_ACCMAG, flags=XM_REG5_DEFAULT,
              scale=XM_REG6_DEFAULT, mode=XM_REG7_DEFAULT):
    """Configure and enable magnetometer"""
    if bus is not None:
        # enable magnetometer
        try:
            bus.write_byte_data(device, CTRL_REG5_XM, flags)
            bus.write_byte_data(device, CTRL_REG6_XM, scale)
            bus.write_byte_data(device, CTRL_REG7_XM, mode)
        except IOError as e:
            logging.error('Unable to configure magnetometer: [%s] %s' %
                          (e.errno, e.strerror))
            return False
        return True
    return mag_enabled

def stop_mag(bus, device=IMU_ACCMAG):
    """Disable magnetometer"""
    return mag_enabled
    if bus is not None:
        try:
            bus.write_byte_data(device, CTRL_REG5_XM, 0)
        except IOError:
            pass
        return False
    return mag_enabled

def read_raw_mag(bus, device=IMU_ACCMAG, mag_buffer=None):
    """Read raw magnetometer data"""
    global mag_raw
    # read raw data
    try:
        x_l = bus.read_byte_data(device, OUT_X_L_M)
        x_h = bus.read_byte_data(device, OUT_X_H_M)
        y_l = bus.read_byte_data(device, OUT_Y_L_M)
        y_h = bus.read_byte_data(device, OUT_Y_H_M)
        z_l = bus.read_byte_data(device, OUT_Z_L_M)
        z_h = bus.read_byte_data(device, OUT_Z_H_M)
    except IOError as e:
        logging.error('Unable to read magnetometer: ', e.strerror)
        return
    # update globals
    with mag_raw_lock:
        mag_raw['x'] = (x_l | x_h << 8)
        mag_raw['y'] = (y_l | y_h << 8)
        mag_raw['z'] = (z_l | z_h << 8)
        if mag_raw['x'] >= 32768:
            mag_raw['x'] -= 65536
        if mag_raw['y'] >= 32768:
            mag_raw['y'] -= 65536
        if mag_raw['z'] >= 32768:
            mag_raw['z'] -= 65536
        if imu_inverted:
            mag_raw['y'] = mag_raw['y'] * -1

def calc_raw_heading():
    """Calculater raw (uncompensated) heading"""
    global heading_raw
    with mag_raw_lock:
        with heading_raw_lock:
            heading_raw = 180 * math.atan2(mag_raw['y'], mag_raw['x']) / math.pi
            if heading_raw < 0:
                heading_raw += 360

def calc_comp_heading():
    """Calculate tilt compensated heading"""
    global heading_comp
    global mag_comp
    if acc_enabled:
        # do tilt compensation
        # normalise x and y
        with acc_raw_lock:
            a_norm = math.sqrt(acc_raw['x'] ** 2 +
                               acc_raw['y'] ** 2 +
                               acc_raw['z'] ** 2)
            acc_x_norm = acc_raw['x'] / a_norm
            acc_y_norm = acc_raw['y'] / a_norm
        # calculate pitch and roll
        pitch = math.asin(acc_x_norm)
        roll = math.asin(acc_y_norm / math.cos(pitch))
        # apply compensation to raw mag readings
        with mag_raw_lock:
                with mag_comp_lock:
                    mag_comp['x'] = (mag_raw['x'] * math.cos(pitch) + mag_raw['z'] *
                                     math.sin(pitch))
                    mag_comp['y'] = (mag_raw['x'] * math.sin(roll) * math.sin(pitch) +
                                     mag_raw['y'] * math.cos(roll) -
                                     mag_raw['z'] * math.sin(roll) * math.cos(pitch))
                    mag_comp['z'] = mag_raw['z'] # don't know the calculation needed
                    # calculate heading
                    with heading_comp_lock:
                        heading_comp = 180 * math.atan2(mag_comp['y'], mag_comp['x']) / math.pi
                        if heading_comp < 0:
                            heading_comp += 360
    else:
        with heading_raw_lock:
            with heading_comp_lock:
                heading_comp = heading_raw

# gyro stuff
def start_gyro(bus=active_i2cbus, device=IMU_GYRO, flags=CTRL_REG1_G_DEFAULT,
               sense=CTRL_REG4_G_DEFAULT, fifo=FIFO_CTRL_REG_G_DEFAULT,
               hpm=CTRL_REG2_G_DEFAULT):
    """Configure and enable gyro"""
    if bus is not None:
        # enable gyro
        try:
            bus.write_byte_data(device, CTRL_REG1_G, flags)
            bus.write_byte_data(device, CTRL_REG2_G, hpm)
            bus.write_byte_data(device, CTRL_REG4_G, sense)
            bus.write_byte_data(device, FIFO_CTRL_REG_G, fifo)
        except IOError as e:
            logging.error('Unable to configure gyro: [%s] %s' %
                          (e.errno, e.strerror))
            return False
        return True
    return gyro_enabled

def stop_gyro(bus=active_i2cbus, device=IMU_GYRO):
    """Disable gyro"""
    if bus is not None:
        try:
            bus.write_byte_data(device, CTRL_REG1_G, 0)
        except IOError:
            pass
        return False
    return gyro_enabled

def read_raw_gyro(bus, device=IMU_GYRO):
    """Read raw gyro data"""
    global gyro_raw
    try:
        x_l = bus.read_byte_data(device, OUT_X_L_G)
        x_h = bus.read_byte_data(device, OUT_X_H_G)
        y_l = bus.read_byte_data(device, OUT_Y_L_G)
        y_h = bus.read_byte_data(device, OUT_Y_H_G)
        z_l = bus.read_byte_data(device, OUT_Z_L_G)
        z_h = bus.read_byte_data(device, OUT_Z_H_G)
    except IOError as e:
        logging.error('Unable to read gyro: ', e.strerror)
        return
    # update globals
    with gyro_raw_lock:
        gyro_raw['x'] = (x_l | x_h << 8)
        gyro_raw['y'] = (y_l | y_h << 8)
        gyro_raw['z'] = (z_l | z_h << 8)
        if gyro_raw['x'] >= 32768:
            gyro_raw['x'] -= 65536
        if gyro_raw['y'] >= 32768:
            gyro_raw['y'] -= 65536
        if gyro_raw['z'] >= 32768:
            gyro_raw['z'] -= 65536
                
def calc_gyro_dps():
    """Calculate gyro degrees per second"""
    global gyro_dps
    with gyro_raw_lock:
        with gyro_dps_lock:
            gyro_dps['x'] = gyro_raw['x'] * G_GAIN
            gyro_dps['y'] = gyro_raw['y'] * G_GAIN
            gyro_dps['z'] = gyro_raw['z'] * G_GAIN

# Pressure sensor stuff
def p_cal_short(msb, lsb):
    result = (msb << 8) + lsb
    if result > 32767:
        result -= 65536
    return result

def p_cal_ushort(msb, lsb):
    return(msb << 8) +lsb

def start_press(bus, device=IMU_PRESS):
    """do initialisation stuff for pressure sensor"""
    global p_calibration
    try:
        bus.read_byte_data(device, P_ID)
    except IOError as e:
        logging.info('Pressure sensor not found: ', e.strerror)
        return False
    # read calibration data
    try:
        cal_data = bus.read_i2c_block_data(device, P_CAL_BASE, P_CAL_LEN)
    except IOError as e:
        logging.error('Failed to read pressure sensor calibration data (%s). Using defaults instead', e.strerror)
        # from datasheet
        p_calibration['AC1'] = 408
        p_calibration['AC2'] = -72
        p_calibration['AC3'] = -14383
        p_calibration['AC4'] = 32741
        p_calibration['AC5'] = 32757
        p_calibration['AC6'] = 23153
        p_calibration['B1'] = 6190
        p_calibration['B2'] = 4
        p_calibration['MB'] = -32768
        p_calibration['MC'] = -8711
        p_calibration['MD'] = 2868
        return True
    p_calibration['AC1'] = p_cal_short(cal_data[P_AC1_H - P_CAL_BASE],
                                           cal_data[P_AC1_L - P_CAL_BASE])
    p_calibration['AC2'] = p_cal_short(cal_data[P_AC2_H - P_CAL_BASE],
                                           cal_data[P_AC2_L - P_CAL_BASE])
    p_calibration['AC3'] = p_cal_short(cal_data[P_AC3_H - P_CAL_BASE],
                                           cal_data[P_AC3_L - P_CAL_BASE])
    p_calibration['AC4'] = p_cal_ushort(cal_data[P_AC4_H - P_CAL_BASE],
                                            cal_data[P_AC4_L - P_CAL_BASE])
    p_calibration['AC5'] = p_cal_ushort(cal_data[P_AC5_H - P_CAL_BASE],
                                            cal_data[P_AC5_L - P_CAL_BASE])
    p_calibration['AC6'] = p_cal_ushort(cal_data[P_AC6_H - P_CAL_BASE],
                                            cal_data[P_AC6_L - P_CAL_BASE])
    p_calibration['B1'] = p_cal_short(cal_data[P_B1_H - P_CAL_BASE],
                                          cal_data[P_B1_L - P_CAL_BASE])
    p_calibration['B2'] = p_cal_short(cal_data[P_B2_H - P_CAL_BASE],
                                          cal_data[P_B2_L - P_CAL_BASE])
    p_calibration['MB'] = p_cal_short(cal_data[P_MB_H - P_CAL_BASE],
                                          cal_data[P_MB_L - P_CAL_BASE])
    p_calibration['MC'] = p_cal_short(cal_data[P_MC_H - P_CAL_BASE],
                                          cal_data[P_MC_L - P_CAL_BASE])
    p_calibration['MD'] = p_cal_short(cal_data[P_MD_H - P_CAL_BASE],
                                          cal_data[P_MD_L - P_CAL_BASE])
    return True

def stop_press(bus, device=IMU_PRESS):
    """dummy function. present for consistancy only"""
    return False

def p_read_temp(bus, device=IMU_PRESS):
    """read temp from pressure sensor"""
    global temp_p
    global b5
    with p_device_lock:
        # due to how the sensor handles reads, we need to be sure
        # we're the only one trying to use it

        try:
            # send read temp command
            bus.write_byte_data(device, P_CTRL_MEAS, P_READ_TEMP)
            time.sleep(P_READ_DELAY[P_READ_TEMP])
            msb = bus.read_byte_data(device, P_OUT_MSB)
            lsb = bus.read_byte_data(device, P_OUT_LSB)
        except IOError as e:
            logging.error('Unable to read temp: %s' % e.sterror)
    ut = (msb << 8) + lsb
    # temp compensation
    x1 = ((ut - p_calibration['AC6']) * p_calibration['AC5']) >> 15
    x2 = (p_calibration['MC'] << 11) / (x1 + p_calibration['MD'])
    b5 = x1 + x2
    comp_temp = (b5 + 8) >> 4
    temp_p = comp_temp / 10.0

def p_read_pressure(bus, device=IMU_PRESS, res=P_READ_PRESS_STD):
    """read current pressure"""
    global pressure
    with p_device_lock:
        # due to how the sensor handles reads, we need to be sure
        # we're the only one trying to use it
        try:
            # send read temp command
            bus.write_byte_data(device, P_CTRL_MEAS, res)
            time.sleep(P_READ_DELAY[res])
            msb = bus.read_byte_data(device, P_OUT_MSB)
            lsb = bus.read_byte_data(device, P_OUT_LSB)
            xlsb = bus.read_byte_data(device, P_OUT_XLSB)
        except IOError as e:
            logging.error('Unable to read pressure: %s' % e.sterror)
    oss = res >> 6
    raw_press = (msb << 16) + (lsb << 8) + xlsb
    raw_press = raw_press >> (8 - oss)
    # pressure compensation
    b6 = b5 - 4000 # b5 calculated when temp is read
    b6b = b6 * b6 >> 12
    x1 = (p_calibration['B2'] *b6b) >> 11
    x2 = p_calibration['AC2'] * b6 >> 11
    x3 = x1 + x2
    b3 = (((p_calibration['AC1'] * 4 + x3) << oss) + 2) >> 2
    
    x1 = p_calibration['AC3'] * b6 >> 13
    x2 = (p_calibration['B1'] * b6b) >> 16
    x3 = ((x1 + x2) + 2) >> 2
    b4 = (p_calibration['AC4'] * (x3 + 32768)) >> 15
    b7 = (raw_press - b3) * (50000 >> oss)
    
    if b7 < 0x80000000:
        p = (b7 * 2) / b4
    else:
        p = (b7 / b4) * 2
        
    x1 = (p / 2 ** 8) * (p / 2 ** 8)
    x1 = (x1 * 3038) / 2 ** 16
    x2 = (-7357 * p) / 2 ** 16
    p = p + ((x1 + x2 + 3791) >> 4)
    pressure = p / 100.0

def p_calc_alt(base_p = MSL_PRESSURE, p=None):
    """give base reference and pressure calculate altitude"""
    if p is None:
        p = pressure # use global
    altitude = 44330.0 * (1.0 - pow(p / base_p, (1.0 / 5.255)))
    return altitude

def p_calc_slp(p = None, alt=105.0):
    """ give pressure and altitude, calculate sea level pressure"""
    if p is None:
        p=pressure
    slp = p / pow(1.0 - alt/44330.0, 5.255)
    return slp

# threads. all are intended to be called by threading.Thread
def track_acc(bus, device=IMU_ACCMAG, interval=0.1):
    """read raw values from accelerometer and calculate angles"""
    while not stop_threads:
        read_raw_acc(bus, device)
        calc_acc_angles()
        time.sleep(interval)

def track_mag(bus, device=IMU_ACCMAG, interval=0.1):
    """read raw values for magnetometer
       calculate raw heading, tilt compensation and compensated heading
    """
    while not stop_threads:
        read_raw_mag(bus, device)
        calc_raw_heading()
        calc_comp_heading()
        time.sleep(interval)

def track_gyro(bus, device=IMU_GYRO, interval=G_MINTIME):
    """read raw values from gyro and apply needed calculations"""
    global gyro_angles
    global combined_angles
    old_time = time.time()
    while not stop_threads:
        new_time=time.time()
        lapsed_time = new_time - old_time
        if lapsed_time >= interval:
            old_time = new_time
            read_raw_gyro(bus, device)
            calc_gyro_dps()
            with gyro_angles_lock:
                with gyro_dps_lock:
                    gyro_angles['x'] += gyro_dps['x'] * lapsed_time
                    gyro_angles['y'] += gyro_dps['y'] * lapsed_time
                    gyro_angles['z'] += gyro_dps['z'] * lapsed_time
            with combined_angles_lock:
                # no case/switch in python :(
                if acc_enabled and gyro_enabled:
                    with acc_deg_lock:
                        with gyro_dps_lock:
                            combined_angles['x'] = AG_FILTER * (combined_angles['x'] + gyro_dps['x'] * lapsed_time) + (1 - AG_FILTER) * acc_deg['x']
                            combined_angles['y'] = AG_FILTER * (combined_angles['y'] + gyro_dps['y'] * lapsed_time) + (1 - AG_FILTER) * acc_deg['y']
                            combined_angles['z'] = AG_FILTER * (combined_angles['z'] + gyro_dps['z'] * lapsed_time) + (1 - AG_FILTER) * acc_deg['z']
                elif acc_enabled and not gyro_enabled:
                    with acc_deg_lock:
                        combined_angles = acc_deg
                elif gyro_enabled and not acc_enabled:
                    with gyro_angles_lock:
                        combined_angles = gyro_angles
                else:
                    pass
        time.sleep(interval / 3.0)

def track_temp(bus, device=IMU_PRESS, interval=0.5):
    """poll temprature sensor"""
    while not stop_threads:
        p_read_temp(bus, device)
        time.sleep(interval)

def track_press(bus, device=IMU_PRESS, res=P_READ_PRESS_STD, interval=0.5):
    """poll pressure sensor"""
    while not stop_threads:
        p_read_pressure(bus, device, res)
        time.sleep(interval)

# Misc
def _cleanup():
    """stop threads and general cleanup"""
    global stop_threads, acc_enabled, mag_enabled, gyro_enabled, press_enabled

    logging.debug('Telling threads to stop.')
    stop_threads = True
    logging.debug('Stopping i2c devices')
    acc_enabled = stop_acc(active_i2cbus)
    mag_enabled = stop_mag(active_i2cbus)
    gyro_enabled = stop_gyro(active_i2cbus)
    press_enabled = stop_press(active_i2cbus)
    logging.debug('Removing pid file')
    try:
        os.unlink(PID_FILE)
    except OSError:
        pass
    try:
        os.unlink(FALLBACK_PID_FILE)
    except OSError:
        pass
    
def valid_ipv4(address):
    result = True
    if address != '':
        try:
            socket.inet_pton(socket.AF_INET, address)
        except AttributeError:
            try:
                socket.inet_aton(address)
            except socket.error:
                result = False
            if address.count('.') != 3: result = False
        except socket.error:
            result = False

    if not result:
        raise argparse.ArgumentTypeError('%s is not a valid IPv4 address' % address)
    
    return address

def valid_ipv4m(address):
    valid_ipv4(address)
    valid=False
    octets = [int(s) for s in address.split('.')]
    if octets[0] in range(224, 238):
        if octets[0] == 224 and octets[1] == 0 and octets[2] ==0:
            # reserved addresses
            pass
        else:
            # globally scoped multicast addresses
            valid = True
    elif octets[0] == 239:
        # locally scoped multicast addresses
        valid=True

    if not valid:
        raise argparse.ArgumentTypeError('%s is not a valid IPv4 multicast address' % address)
    
    return address

def hex_string_to_int(string):
    return int(string, 16)

        
def heading_to_cardinal(heading, cardinals=CARDINALS_N_NE_ETC):
    """given a heading return direction of nearest cardinal point"""
    lookup = []

    if cardinals == CARDINALS_N_S_E_W:
        lookup = ['N', 'E', 'S', 'W']
    elif cardinals == CARDINALS_N_NE_ETC:
        lookup = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    elif cardinals == CARDINALS_N_NNE_ETC:
        lookup = ['N', 'NNE', 'NE', 'ENE',
                  'E', 'ESE', 'SE', 'SSE',
                  'S', 'SSW', 'SW', 'WSW',
                  'W', 'WNW', 'NW', 'NNW']
    segment_width = 360.0 / cardinals
    heading -= (segment_width / 2.0)
    heading_segment = heading / segment_width
    heading_segment = int(heading_segment + 0.5)
    if heading_segment < 0:
        heading_segment = 0
    try:
        return lookup[heading_segment]
    except IndexError:
        return ''
    
def check_threads():
    """check tracker threads are still running and restart if necessary"""
    global thread_list
    if not stop_threads:
        if not cmd_args.nolisten and thread_list['listener'] is not None:
            if not thread_list['listener'].is_alive():
                logging.info('Thread %s has died. Restarting' % thread_list['listener'].name)
                t = threading.Thread(target=_s_listener,
                                     args=(cmd_args.port, cmd_args.bind))
                t.start()
                thread_list['listener'] = t
        if cmd_args.broadcast and thread_list['broadcaster'] is not None:
            if not thread_list['broadcaster'].is_alive():
                logging.info('Thread %s has died. Restarting' % thread_list['broadcaster'].name)
                t = threading.Thread(name='_bcast_servert', target=_bcast_server,
                                     args=(BROADCAST_ADDRESS, cmd_args.port,BROADCAST_INTERVAL))
                t.start()
                thread_list['broadcaster'] = t
        if cmd_args.multicast and thread_list['multicaster'] is not None:
            if not thread_list['multicaster'].is_alive():
                logging.info('Thread %s has died. Restarting' % thread_list['multicaster'].name)
                t = threading.Thread(name='_mcast_servert', target=_mcast_server,
                                     args=(cmd_args.multicast, cmd_args.port,
                                           BROADCAST_INTERVAL))
                t.start()
                thread_list['broadcaster'] = t
        if acc_enabled and thread_list['acc'] is not None:
            if not thread_list['acc'].is_alive():
                logging.info('Thread %s has died. Restarting' % thread_list['acc'].name)
                t = threading.Thread(target=track_acc, args=(active_i2cbus, ))
                t.start()
                thread_list['acc'] = t
        if mag_enabled and thread_list['mag'] is not None:
            if not thread_list['mag'].is_alive():
                logging.info('Thread %s has died. Restarting' % thread_list['mag'].name)
                t = threading.Thread(target=track_mag, args=(active_i2cbus, ))
                t.start()
                thread_list['mag'] = t
        if gyro_enabled and thread_list['gyro'] is not None:
            if not thread_list['gyro'].is_alive():
                logging.info('Thread %s has died. Restarting' % thread_list['gyro'].name)
                t = threading.Thread(target=track_gyro, args=(active_i2cbus, ))
                t.start()
                thread_list['gyro'] = t
        if press_enabled:
            if  not thread_list['press'].is_alive():
                logging.info('Thread %s has died. Restarting' % thread_list['press'].name)
                t = threading.Thread(target=track_press, args=(active_i2cbus, ))
                t.start()
                thread_list['press'] = t
            if  not thread_list['temp'].is_alive():
                logging.info('Thread %s has died. Restarting' % thread_list['temp'].name)
                t = threading.Thread(target=track_temp, args=(active_i2cbus, ))
                t.start()
                thread_list['press'] = t


# broadcast server
def _bcast_server(addr, port, interval):
    logging.info('Starting broadcast server on port %s with update interval %s'
                 % (port, interval))
    bs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bs.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while not stop_threads:
        time.sleep(interval)
        message = _s_proc_cmd('?')[0]
        bs.sendto(message, (addr, port))
    bs.close()

# multicast server
def _mcast_server(addr, port, interval):
    logging.info('Starting multicast server on %s:%s with update interval %s'
                 % (addr, port, interval))
    ms = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ms.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 20)
    while not stop_threads:
        time.sleep(interval)
        message = _s_proc_cmd('?')[0]
        ms.sendto(message, (addr, port))
    ms.close()


# socket server
def _s_listener(port=DEFAULT_PORT, bind_to=LISTEN_HOST):
    logging.info('Listening on %s:%s' % (bind_to, port))
    ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ss.bind((bind_to, port))
    ss.listen(5)
    ss.setblocking(0) # non blocking
    while not stop_threads:
        r, w, e = select.select([ss, ],[],[],0)
        for s in r:
            if s == ss:
                (cs, addr) = ss.accept()
                ct = threading.Thread(target=_s_worker, args=(cs, addr, ))
                ct.start()
        time.sleep(0.1)
    logging.debug('Closing down listener')
    ss.close()

def _s_proc_cmd(command, up_int=0.0):
    """Interpret command and return result"""
    result = ''
    new_int = None

    if command.strip() == '': return ('', None)

    # force incoming command to lowercase
    command = command.lower()
    
    if command.startswith('u='):
        # set update interval command
        failed = False
        try:
            new_int = float(command[2:])
        except:
            failed = True
        if new_int < 0: failed = True
        if failed:
            result = 'huh?\n'
            new_int = None
    elif command == '?':
        # dontcha just love recursion?
        result += _s_proc_cmd('?o')[0]
        result += _s_proc_cmd('?h')[0]
        result += _s_proc_cmd('?t')[0]
        result += _s_proc_cmd('?p')[0]
        result += _s_proc_cmd('?i')[0]
        result += _s_proc_cmd('?u')[0]
        result = result.replace('OK\n','')
        result = result.replace('huh?\n','')
    elif command == '?o':
        # orientation angles
        if acc_enabled:
            result = 'o:%r\n' % acc_deg
        else:
            result = 'huh?\n'
    elif command == '?h':
        # compass heading
        if mag_enabled:
            result = 'h:'
            result += str((heading_to_cardinal(heading_comp), heading_comp))
            result += '\n'
        else:
            result = 'huh?\n'
    elif command == '?t':
        # temp, from pressure sensor
        if press_enabled:
            result = 't:'
            result += '%2.2f\n' % temp_p
        else:
            result = 'huh?\n'
    elif command == '?p':
        # pressure
        if press_enabled:
            result = 'p:'
            result += '%5.2f\n' % pressure
        else:
            result = 'huh?\n'
    elif command == '?u':
        result = 'u:%f\n' % up_int
    elif command == '?i':
        result = 'i:%i\n' % imu_inverted
    else:
        result = 'huh?\n'
        new_int = None
        
    if result != 'huh?\n': result += 'OK\n'
    if not result.endswith('\n'): result += '\n'
    return (result, new_int)
    
def _s_worker(cs, addr):
    try:
        client_info = socket.gethostbyaddr(addr[0])[0]
    except:
        client_info = ''
    client_info += '(%s)' % addr[0]
    
    logging.info('Accepted connection from: %s' % client_info)

    #rename thread
    me = threading.current_thread()
    me.name = '_s_worker for ' + client_info

    update_interval = 0.0
    
    out_buf = 'hello\n'
    out_buf += CMD_VERSION # command set version
    # i[nverted] & u[pdate interval] are always available
    out_buf += 'available:iu'
    if acc_enabled:
        out_buf += 'o' # o[rentation]
    if mag_enabled:
        out_buf += 'h' # h[eading]
    if press_enabled:
        out_buf += 'tp' # t[emp] & p[ressure]
    out_buf += '\n'
    out_buf += _s_proc_cmd('?')[0]
    cs.sendall(out_buf)
    old_time = time.time()
    while not stop_threads:
        new_time = time.time()
        lapsed_time = new_time - old_time
        # buffers
        in_buf = ''
        new_in = ''
        # check for closed socket
        r, w, e = select.select([cs, ],[],[],0)
        # don't want to block an socker.recv
        if len(r) == 1:
            # so only call it if the socket is ready for it
            new_in = cs.recv(4096)
            logging.debug('Recieved: %s' % new_in)
            if len(new_in) == 0:
                logging.info('%s closed its connection.' % client_info)
                break
            in_buf += new_in
            for l in in_buf.splitlines(True):
                if l.endswith(('\n', '\r\n', '\r')):
                    # only last line can fail this test
                    # remove line from buffer
                    in_buf = in_buf[len(l):]
                    # strip white space at ends
                    l = l.strip()
                    out_buf, new_int = _s_proc_cmd(l, update_interval)
                    if new_int is not None: update_interval = new_int
                    cs.sendall(out_buf)
        if (update_interval != 0) and (lapsed_time >= update_interval):
            old_time = new_time
            cs.sendall(_s_proc_cmd('?')[0])
        time.sleep(0.01) # stop thread from hogging the cpu
    cs.close()

# daemonize
# based on the python recipe at
#   http://code.activestate.com/recipes/278731-creating-a-daemon-the-python-way

def _daemonize_me():
    umask = 0
    workingdir = '/'
    maxfd = 1024
    if (hasattr(os, 'devnull')):
        devnull = os.devnull
    else:
        devnull = '/dev/null'

    try:
        pid = os.fork()
    except OSError as e:
        raise Exception, '%s [%d]' % (e.strerror, e.errno)

    if pid == 0:
        # we are the first child
        os.setsid()
        try:
            pid = os.fork()
        except OSError as e:
            raise Exception, '%s [%d]' % (e.strerror, e.errno)
        if pid == 0:
            # second child
            os.chdir(workingdir)
            os.umask(umask)
        else:
            os._exit(0)
    else:
        os._exit(0)
    # close all open file descriptors
    try:
        mfd = os.sysconf('SC_OPEN_MAX')
    except (AttributeError, ValueError):
        mfd=maxfd
    for fd in range(0, mfd):
        try:
            os.close(fd)
        except OSError:
            pass # ignore as it wasn't open anyway

    # redirect stdio
    os.open(devnull, os.O_RDWR) # stdin
    os.dup2(0, 1) # stdout
    os.dup2(0, 2) # stderr
    
# debug code
def _log_acc():
    if acc_enabled:
        logging.debug('Raw accelerometer data: %s' % acc_raw)
        logging.debug('Accelerometer angles: %s' % acc_deg)

def _log_mag():
    if mag_enabled:
        logging.debug('Raw magnetometer data: %s' % mag_raw)
        logging.debug('Tilt compensated: %s' % mag_comp)
        logging.debug('Heading: %s(%s)' % (heading_to_cardinal(heading_raw), heading_raw))
        logging.debug('Heading (tilt compensated): %s(%s)' % (heading_to_cardinal(heading_comp), heading_comp))

def _log_gyro():
    if gyro_enabled:
        logging.debug('Raw gyro data: %s' % gyro_raw)
        logging.debug('Gyro angles: %s' % gyro_angles)
        logging.debug('Gyro DPS: %s' % gyro_dps)
        
def _log_combined():
    if acc_enabled and gyro_enabled:
        logging.debug('Combined acc & gyro angles: %s' % combined_angles)

def _log_temp():
    logging.debug('Temprature (pressure sensor): %s' % temp_p)

def _log_press():
    logging.debug('Current pressure: %s' % pressure)
    logging.debug('Altitude: %s' % p_calc_alt())
    logging.debug('Sea level pressure: %s' % p_calc_slp())

def _log_all():
    logging.debug('-' * 40)
    _log_acc()
    _log_gyro()
    _log_combined()
    _log_mag()
    _log_temp()
    _log_press()
    logging.debug('-' * 40)


# and now...

if __name__ == '__main__':
    # cmd line args
    parser = argparse.ArgumentParser(
        epilog='If neither -l nor -b are present all available interfaces are bound.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-N', '--nolisten', help='Do not listen for incoming connections',
                        action='store_true')
    group.add_argument('-l', '--local', help='bind to localhost only.',
                        action='store_true')
    group.add_argument('-b', '--bind', help='address of interface to bind to.',
                       type=valid_ipv4, default=LISTEN_HOST, metavar='W.X.Y.Z')
    parser.add_argument('-B', '--broadcast', help='Use netwrok broadcasts.',
                        action='store_true')
    parser.add_argument('-M', '--multicast', help='Use IPv4 multicasting.',
                        type=valid_ipv4m, metavar='address')
    parser.add_argument('-p', '--port', help='port to use.',
                        type=int, default=DEFAULT_PORT)
    parser.add_argument('-i', '--i2c', type=int, choices=[0,1],
                        help='i2c bus to use.(default: %s)' % I2CBUS, default=I2CBUS)
    parser.add_argument('-g', '--gyro', help='address of gyroscope (default: %s)' % hex(IMU_GYRO),
                        choices=['0x6a', '0x6b'], default=IMU_GYRO,
                        type=hex_string_to_int)
    parser.add_argument('-m', '--motion', help='address of magnetometer and accelerometer (default: %s)' % hex(IMU_ACCMAG),
                        choices=['0x1d', '0x1e'], default=IMU_ACCMAG,
                        type=hex_string_to_int)
    parser.add_argument('-n', '--nodaemon', help='do not daemonize. Run in foreground instead.',
                        action='store_true')
    parser.add_argument('--debug', help='enabled debug output',
                        action='store_true')
    cmd_args = parser.parse_args()

    #dev only
##    print cmd_args.multicast
##    sys.exit(os.EX_OK)
    
    # is there an existing process running?
    process_exists = [None, None]
    if os.path.exists(PID_FILE):
        check_pid = int(open(PID_FILE).read())
        process_exists[0] = True
        try:
            os.kill(check_pid, 0)
        except OSError:
            process_exists[0] = False
    else:
        process_exists[0] = False
    if os.path.exists(FALLBACK_PID_FILE):
        check_pid = int(open(FALLBACK_PID_FILE).read())
        process_exists[1] = True
        try:
            os.kill(check_pid, 0)
        except OSError:
            process_exists[1] = False
    else:
        process_exists[1] = False
        
    if any(process_exists):
        print('Process already running. Exiting.')
        sys.exit(os.EX_TEMPFAIL)

    if not cmd_args.nodaemon:
        _daemonize_me()
        # write new pid file (only if daemonized)
        my_pid = os.getpid()
        try:
            open(PID_FILE, 'w').write(str(my_pid) + '\n')
        except IOError:
            try:
                open(FALLBACK_PID_FILE, 'w').write(str(my_pid) + '\n')
            except:
                pass
        
    # exit handler
    atexit.register(_cleanup)
    
    # setup logging
    if cmd_args.debug:
        log_level=logging.DEBUG
    else:
        log_level=logging.INFO
    logging.basicConfig(level=log_level, format='%(levelname)-8s: %(message)s',
                        datefmt='%Y-%m-%d %H:%M')
    try:
        file_logger = logging.FileHandler(filename=LOG_FILE, mode='w')
    except IOError:
        logging.debug('Failed to open log file: %s Trying fallback (%s) instead' % (LOG_FILE, FALLBACK_LOG_FILE))
        try:
            file_logger = logging.FileHandler(filename=FALLBACK_LOG_FILE, mode='w')
        except IOError:
            logging.debug('Failed to open fallback log file (%s). Logging to console only.' % FALLBACK_LOG_FILE)
            file_logger = None
    if file_logger is not None:
        file_formatter = logging.Formatter('%(asctime)s %(name)-12s %(levelname)-8s %(message)s')
        file_logger.setFormatter(file_formatter)
        logging.getLogger('').addHandler(file_logger)

    logging.debug('sys.argv: %s' % str(sys.argv))
    logging.debug('parsed: %s' % cmd_args)
    
    # setup i2c bus access
    try:
        active_i2cbus = smbus.SMBus(cmd_args.i2c)
    except IOError as e:
        logging.critical('Unable to open i2c bus: %s\nAm I running as root?', e.strerror)
        sys.exit(os.EX_IOERR)

    acc_enabled = start_acc(active_i2cbus)
    mag_enabled = start_mag(active_i2cbus)
    gyro_enabled = start_gyro(active_i2cbus)
    press_enabled = start_press(active_i2cbus)

    # exit if nothing enabled
    if acc_enabled or mag_enabled or gyro_enabled or press_enabled:
        pass
    else:
        logging.critical('Unable to access IMU devices. Exiting')
        sys.exit(os.EX_NOINPUT)

    # is the IMU mounted upside down?
    if acc_enabled:
        read_raw_acc(active_i2cbus)
    if mag_enabled:
        read_raw_mag(active_i2cbus)
    if acc_raw['z'] < 0 or mag_raw['z'] < 0:
        # should never be 0 unless we're in orbit ;)
        imu_inverted = True
    logging.debug('Initial z axis accelerometer state(raw): %f' % acc_raw['z'])
    logging.debug('Initial z axis magnetometer state(raw): %f' % mag_raw['z'])
    logging.debug('IMU inverted: %s' % imu_inverted)

    # start threads
    thread_list = {'acc':None, 'mag':None, 'gyro':None, 'press':None,
                       'temp':None, 'listener':None}
    # accelerometer
    if acc_enabled:
        t = threading.Thread(name='track_acc', target=track_acc, args=(active_i2cbus, ))
        t.start()
        thread_list['acc'] = t
    # magnetometer
    if mag_enabled:
        t = threading.Thread(name='track_mag', target=track_mag, args=(active_i2cbus, ))
        t.start()
        thread_list['mag'] = t
    # gryo
    if gyro_enabled:
        t = threading.Thread(name='track_gyro', target=track_gyro, args=(active_i2cbus, ))
        t.start()
        thread_list['gyro'] = t
    # temp and pressure
    if press_enabled:
        t = threading.Thread(name='track_press', target=track_press, args=(active_i2cbus, ))
        t.start()
        thread_list['press'] = t
        t = threading.Thread(name='track_temp', target=track_temp, args=(active_i2cbus, ))
        t.start()
        thread_list['temp'] = t

    # start socket server
    if not cmd_args.nolisten:
        try:
            listener = threading.Thread(name='_s_listener', target=_s_listener,
                                        args=(cmd_args.port, cmd_args.bind))
            listener.start()
            thread_list['listener'] = listener
        except:
            logging.error('Unable to start server thread.')
##            sys.exit(1)
            
    # start broadcast server
    if cmd_args.broadcast:
        try:
            broadcaster = threading.Thread(name='_bcast_servert',
                                           target=_bcast_server,
                                           args=(BROADCAST_ADDRESS, cmd_args.port,
                                                 BROADCAST_INTERVAL))
            broadcaster.start()
            thread_list['broadcaster'] = broadcaster
        except:
            logging.error('Unable to start broadcast server')
    
    # start multicast server
    if cmd_args.multicast is not None:
        try:
            multicaster = threading.Thread(name='_mcast_servert',
                                           target=_mcast_server,
                                           args=(cmd_args.multicast, cmd_args.port,
                                                 BROADCAST_INTERVAL))
            multicaster.start()
            thread_list['multicaster'] = multicaster
        except:
            logging.error('Unable to start multicast server')

    time.sleep(1) # give everything time to start
    
    try:
        while True:
            check_threads()
            time.sleep(0.25)
    except KeyboardInterrupt:
##        _cleanup()
        logging.shutdown()
        sys.exit(os.EX_OK)
    finally:
        _cleanup()
        
