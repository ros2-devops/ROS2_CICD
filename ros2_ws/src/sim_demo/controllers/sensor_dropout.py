# sensor_dropout.py
from controller import Robot
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())
duration = 10000  # duration to drop sensors (ms)

gps = robot.getDevice("gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

drop_start = robot.getTime() + random.uniform(5, 15)
drop_end = drop_start + random.uniform(2, 4)

while robot.step(timestep) != -1:
    now = robot.getTime()
    if drop_start <= now <= drop_end:
        gps.disable()
        compass.disable()
    else:
        if gps.getSamplingPeriod() == 0:
            gps.enable(timestep)
        if compass.getSamplingPeriod() == 0:
            compass.enable(timestep)
