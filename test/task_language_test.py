print("正在获取任务语言描述...")

from lerobot.datasets.lerobot_dataset import LeRobotDataset

# 本地路径
ds = LeRobotDataset("/media/kaede-rei/AgroTech/home/huggingface/lerobot/agro/dual_arm_box")

# 打印各个任务的语言描述
print("")
print(ds.meta.tasks)