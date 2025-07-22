# io_stall.py
from controller import Robot
import random
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
    if random.random() < 0.01:  # 1% chance each step
        sleep_duration = random.uniform(0.05, 0.2)
        time.sleep(sleep_duration)
