# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
A script to fly 5 Crazyflies in formation. One stays in the center and the
other four fly aound it in a circle. Mainly intended to be used with the
Flow deck.
The starting positions are vital and should be oriented like this

     >

^    +    v

     <

The distance from the center to the perimeter of the circle is around 0.5 m

"""
import math
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
#定义crazyflie飞机
# Change uris according to your setup
# URI0 = 'radio://0/70/2M/E7E7E7E7E7'
# URI1 = 'radio://0/110/2M/E7E7E7E702'
# URI2 = 'radio://0/94/2M/E7E7E7E7E7'
# URI3 = 'radio://0/5/2M/E7E7E7E702'
URI4 = 'radio://0/100/2M/E7E7E7E703'
#定义直径和高度
# d: diameter of circle
# z: altituce
# params0 = {'d': 1.0, 'z': 0.3}
# params1 = {'d': 1.0, 'z': 0.3}
# params2 = {'d': 0.0, 'z': 0.5}
# params3 = {'d': 1.0, 'z': 0.3}
params4 = {'d': 1.0, 'z': 1.0}

#定义飞机群集
uris = {
    # URI0,
    # URI1,
    # URI2,
    # URI3,
    URI4,
}
#定义字典
params = {
    # URI0: [params0],
    # URI1: [params1],
    # URI2: [params2],
    # URI3: [params3],
    URI4: [params4],#字典
}

#定义起飞
def take_off(cf, position):
    take_off_time = 5.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)# 在世界参考坐标系设定点中发送速度。 (self, vx, vy, vz, yawrate)

        time.sleep(sleep_time)

#定义降落
def land(cf, position):
    landing_time = 5.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time

    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

#定义定点（飞机，时间，高度）
def poshold(cf, t, z):
    steps = t * 10

    for r in range(steps):
        cf.commander.send_hover_setpoint(0, 0, 0, z)#高度作为绝对设定点发送(self, vx, vy, yawrate, zdistance)
        time.sleep(0.1)

#定义飞行程序
def run_sequence(scf, params):
    cf = scf.cf

    # Number of setpoints sent per second
    fs = 4
    fsi = 1.0 / fs

    # Compensation for unknown error :-(   未知错误的补偿
    comp = 1.3

    # Base altitude in meters#基准高度，起飞高度
    base = 0.5

    d = params['d']#取直径值
    z = params['z']#取高度值

    take_off(cf, sequence[0])#起飞

    poshold(cf, 2, base)#定点2秒，高度为0.5米

    # ramp = fs * 2#8次
    # for r in range(ramp):#循环8次
    #     cf.commander.send_hover_setpoint(0, 0, 0, base + r * (z - base) / ramp)#(self, vx, vy, yawrate, zdistance)
    #     time.sleep(fsi)
    # poshold(cf, 2, z)

    
# """
#         Control mode where the height is send as an absolute setpoint (intended
#         to be the distance to the surface under the Crazflie).

#         vx and vy are in m/s
#         yawrate is in degrees/s
#         vx and vy are in m/s
#         yawrate is in degrees/s
#         """
# """
# 1 0.25625
# 2 0.3626
# 3 0.46875
# 4 0.575
# 5 0.68125
# 6 0.7875
# 7 0.89375 
# 8 1
# """
#定义绕圆       
    for _ in range(2):
        # The time for one revolution
        circle_time = 8#8秒的绕圆时间

        steps = circle_time * fs #次数8*4=32次
        for _ in range(steps):#循环32次
        #有一个分速度，有一个转动角度，高度保持
            cf.commander.send_hover_setpoint(d * comp * math.pi / circle_time,
                                             0, 360.0 / circle_time, z)#(self, vx, vy, yawrate, zdistance)#高度作为绝对设定点发送
            time.sleep(fsi)

    poshold(cf, 2, z)#定点，飞行完毕后进行定高

    # for r in range(ramp):#循环8次
    #     cf.commander.send_hover_setpoint(0, 0, 0,
    #                                      base + (ramp - r) * (z - base) / ramp)
    #     time.sleep(fsi)

    # poshold(cf, 1, base)

    
    land(cf, sequence[-1])
    #cf.commander.send_stop_setpoint()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel(reset_estimator)
        swarm.parallel(run_sequence, args_dict=params)
