"""
Rename this script to main.py, then upload to the pico board.
"""
import sys
import select
from pico_controller import Robot
from time import sleep_ms


homeplate_robot = Robot(left_motor_pins=(18, 19, 2, 3), right_motor_pins=(20, 21, 4, 5))
while True:
    # read data from serial
    if select.select([sys.stdin], [], [], 0)[0]:
        data_line = sys.stdin.readline()
        vel_msg = data_line.split(',')
        if len(vel_msg) == 2:
            target_lin = float(vel_msg[0])
            target_ang = float(vel_msg[1])
            homeplate_robot.set_velocity(target_lin, target_ang)
            homeplate_robot.led.value(1)
    else:
        homeplate_robot.led.value(0)
    # send data via serial
    sys.stdout.write(f"{homeplate_robot.linear_velocity},{homeplate_robot.angular_velocity}\n")
    sleep_ms(10)
