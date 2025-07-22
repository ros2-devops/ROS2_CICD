# ram_hog.py
from controller import Robot
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

leak = []
count = 0

while robot.step(timestep) != -1:
    # Simulate memory allocation
    if count % 100 == 0:
        leak.append([0] * 1000000)  # ~1MB
    count += 1
