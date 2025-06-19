from controller import Robot
import time
import math

robot = Robot()
time_step = int(robot.getBasicTimeStep())

print("cpu_hog_controller started...")

while robot.step(time_step) != -1:
    # Simulate heavy computation to spike CPU usage
    _ = sum(math.sqrt(i) for i in range(100000))
    time.sleep(0.01)  # Simulate a tiny processing delay
