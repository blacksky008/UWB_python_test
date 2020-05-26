# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
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
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger


from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


# URI to the Crazyflie to connect to
uri3 = 'radio://0/100/2M/E7E7E7E703'
uri5 = 'radio://0/100/2M/E7E7E7E705'

# Change the sequence according to your setup
#             x    y    z  YAW
#飞机行走路径
#第一架编号为03飞机行走路径
sequence3 = [
    (1.0, 2.0, 1.0, 0),
    (2.0, 2.0, 1.0, 0),
    (2.0, 2.0, 1.0, 90),
    (2.0, 3.0, 1.0, 90),
    (2.0, 3.0, 1.0, 180),
    (1.0, 3.0, 1.0, 180),
    (1.0, 3.0, 1.0, 270),
    (1.0, 2.0, 1.0, 270),
    (1.0, 2.0, 1.0, 0),
    (1.0, 2.0, 0.5, 0),

]

#第二架编号为05飞机行走路径
sequence5 = [
    (2.0, 2.0, 1.0, 0),
    (2.0, 3.0, 1.0, 0),
    (2.0, 3.0, 1.0, 90),
    (1.0, 3.0, 1.0, 90),
    (1.0, 3.0, 1.0, 180),
    (1.0, 2.0, 1.0, 180),
    (1.0, 2.0, 1.0, 270),
    (2.0, 2.0, 1.0, 270),
    (2.0, 2.0, 1.0, 0),
    (2.0, 2.0, 0.5, 0),
]


#定义对应关系
seq_args = {
    uri3: [sequence3],
    uri5: [sequence5],

}
# List of URIs, comment the one you do not want to fly

uris = {
    uri3,
    uri5,
}


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def take_off(cf, position):
    take_off_time = 3.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

   # print(vz)

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def land(cf, position):
    landing_time = 3.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time

  #  print(vz)

    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

# def speed(cf,position)
#     do_time=1.0
#     sleep_time=0.1
#     steps = int(do_time / sleep_time)
#     vx=position[0]/do_time
#     vy=position[1]/do_time


# def run_sequence(scf, sequence):
#     try:
#         cf = scf.cf

#         # take_off(cf, sequence[0])
#         for position in sequence:
#             # print('Setting position {}'.format(position))
#             end_time = time.time() + position[3]
#             while time.time() < end_time:
#                 cf.commander.send_position_setpoint(position[0],
#                                                     position[1],
#                                                     position[2], 0)
#                 time.sleep(0.1)
#         # land(cf, sequence[-1])
#     except Exception as e:
#         print(e)


def run_sequence(scf, sequence):
    cf = scf.cf
    take_off(cf, sequence[0])
    for position in sequence:
        print('Setting position {}'.format(position))
        for i in range(50):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            time.sleep(0.1)
    land(cf, sequence[-1])
   # cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    #time.sleep(0.1)


# if __name__ == '__main__':
#     cflib.crtp.init_drivers(enable_debug_driver=False)

#     with SyncCrazyflie(uri3, cf=Crazyflie(rw_cache='./cache')) as scf:
#         reset_estimator(scf)
#         # start_position_printing(scf)
#         run_sequence3(scf, sequence3)



# if __name__ == '__main__':
#     cflib.crtp.init_drivers(enable_debug_driver=False)

#     with SyncCrazyflie(uri5, cf=Crazyflie(rw_cache='./cache')) as scf:
#         reset_estimator(scf)
#         # start_position_printing(scf)
#         run_sequence5(scf, sequence5)

if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        # If the copters are started in their correct positions this is
        # probably not needed. The Kalman filter will have time to converge
        # any way since it takes a while to start them all up and connect. We
        # keep the code here to illustrate how to do it.
        # swarm.parallel(reset_estimator)

        # The current values of all parameters are downloaded as a part of the
        # connections sequence. Since we have 10 copters this is clogging up
        # communication and we have to wait for it to finish before we start
        # flying.
      #  print('Waiting for parameters to be downloaded...')
       # swarm.parallel(wait_for_param_download)

        swarm.parallel(run_sequence, args_dict=seq_args)


# if __name__ == '__main__':
#     cflib.crtp.init_drivers(enable_debug_driver=False)

#     with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
#         reset_estimator(scf)
#         # start_position_printing(scf)
#         run_sequence(scf, sequence)