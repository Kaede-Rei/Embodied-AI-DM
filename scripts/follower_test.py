from lerobot_robot_multi_robots.dm_arm import DMFollower
from lerobot_robot_multi_robots.config_dm_arm import DMFollowerConfig
from lerobot_robot_multi_robots.motors.DM_Control_Python.DM_CAN import *

import time

follower_config = DMFollowerConfig(
    port="/dev/ttyACM0",
)
follower = DMFollower(follower_config)

follower.connect()

for key, motor in follower.motors.items():
    follower.control.disable(motor)

try:
    while True:
        print(follower.get_observation())
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping read position...")
    follower.disconnect()
