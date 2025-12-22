import argparse
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from trlc_dk1.follower import DK1Follower, DK1FollowerConfig
from trlc_dk1.leader import DK1Leader, DK1LeaderConfig
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.scripts.lerobot_record import record_loop

def parse_args():
    ap = argparse.ArgumentParser(description="DK1 Dataset Recording")
    ap.add_argument("--follower_port", default="/dev/ttyACM0")
    ap.add_argument("--leader_port", default="/dev/ttyUSB0")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--num_episodes", type=int, default=50)
    ap.add_argument("--episode_time_sec", type=int, default=60)
    ap.add_argument("--reset_time_sec", type=int, default=10)
    ap.add_argument("--repo_id", type=str, required=True, help="Hugging Face dataset repo_id")
    ap.add_argument("--task_description", default="my DK1 task")
    ap.add_argument("--cameras", action="store_true", help="是否启用相机录制（需手动配置 camera_config）")
    ap.add_argument("--push_to_hub", action="store_true", help="是否上传到 Hub")
    return ap.parse_args()

def main():
    args = parse_args()

    # 配置（可根据需要添加相机）
    camera_config = {}  # 示例：{"wrist": {...}, "context": {...}} 如果有相机
    if args.cameras:
        # 在此处添加相机配置
        pass

    follower_config = DK1FollowerConfig(port=args.follower_port, cameras=camera_config if args.cameras else None)
    leader_config = DK1LeaderConfig(port=args.leader_port)

    # 初始化
    follower = DK1Follower(follower_config)
    leader = DK1Leader(leader_config)

    # 数据集特征
    action_features = hw_to_dataset_features(follower.action_features, "action")
    obs_features = hw_to_dataset_features(follower.observation_features, "observation")
    dataset_features = {**action_features, **obs_features}

    # 创建数据集（需先 hf auth login）
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=dataset_features,
        use_videos=args.cameras,
    )

    # 辅助工具
    _, events = init_keyboard_listener()
    init_rerun(session_name="dk1_recording")

    # 连接
    follower.connect()
    leader.connect()

    try:
        episode_idx = dataset.num_episodes
        while episode_idx < args.num_episodes and not events["stop_recording"]:
            log_say(f"Recording episode {episode_idx + 1} of {args.num_episodes}")

            record_loop(
                robot=follower,
                teleop=leader,
                dataset=dataset,
                events=events,
                fps=args.fps,
                episode_time_s=args.episode_time_sec,
                reset_time_s=args.reset_time_sec,
                single_task=args.task_description,
                display_data=True,
            )

            if events["rerecord_episode"]:
                log_say("Re-recording episode")
                events["rerecord_episode"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            episode_idx += 1

        log_say("Recording finished")

    finally:
        follower.disconnect()
        leader.disconnect()

    if args.push_to_hub:
        dataset.push_to_hub()

if __name__ == "__main__":
    main()