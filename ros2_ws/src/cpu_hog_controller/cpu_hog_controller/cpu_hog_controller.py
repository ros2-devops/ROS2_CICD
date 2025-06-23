#!/usr/bin/env python3

from controller import Robot
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Burn CPU for ~10 seconds (simulate edge load)
start_time = time.time()
while robot.step(timestep) != -1:
    # Tight loop to simulate CPU overload
    for _ in range(10**6):
        _ = 3.1415 ** 3.1415

    if time.time() - start_time > 10:
        break
