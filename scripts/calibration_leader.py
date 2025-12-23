from trlc_dk1.leader import DK1Leader, DK1LeaderConfig

leader = DK1Leader(DK1LeaderConfig(port="/dev/ttyUSB0"))
leader.connect()

# 建议先禁用夹爪力矩，避免写寄存器时受力
leader.bus.write("Torque_Enable", "gripper", 0, normalize=False)

# 将所有关节的当前编码值偏移到中点（2048），使当前姿势成为“零”
target_mid = 2048
for motor in leader.bus.motors:
    if motor == "gripper":
        # 夹爪通常不居中到 2048，如需也设零可按需处理
        continue
    pos = leader.bus.read("Present_Position", motor, normalize=False)
    leader.bus.write("Homing_Offset", motor, target_mid - pos, normalize=False)
    print(f"{motor}: Present={pos}, Offset={target_mid - pos}")

# 简单校验
print("Homing_Offset:", leader.bus.sync_read("Homing_Offset", normalize=False))
print("Present_Position:", leader.bus.sync_read("Present_Position", normalize=False))
print("get_action():", leader.get_action())

leader.disconnect()