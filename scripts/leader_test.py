from lerobot_robot_multi_robots.dm_arm import DMLeader
from lerobot_robot_multi_robots.config_dm_arm import DMLeaderConfig
import time

leader_config = DMLeaderConfig(port="/dev/ttyUSB1")

leader = DMLeader(leader_config)
leader.connect()


while True:
    action = leader.get_action()
    print(action)
    time.sleep(0.02)
