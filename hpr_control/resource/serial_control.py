"""
Rename this script to main.py, then upload to the pico board.
"""
import sys
import select
from pico_controller import Robot
from time import sleep


mybot = Robot(left_motor_pins=(18, 19, 2, 3), right_motor_pins=(20, 21, 4, 5))
target_lin = 0.
target_ang = 0.
while True:
    if select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline()
        target_lin = float(line.split(',')[0])
        target_ang = float(line.split(',')[1])
    mybot.set_velocity(target_lin=target_lin, target_ang=target_ang)
    sys.stdout.write(f"{mybot.linear_velocity},{mybot.angular_velocity}\n")
    sleep(0.05)

