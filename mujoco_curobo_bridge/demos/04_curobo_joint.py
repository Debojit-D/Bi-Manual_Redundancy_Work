from curobo.util_file import (
    get_robot_configs_path,
    join_path,
    load_yaml,
)
from curobo.types.robot import RobotConfig

config_file = join_path(
    get_robot_configs_path(),
    "franka.yml"
)

robot_dict = load_yaml(config_file)

robot_cfg = RobotConfig.from_dict(robot_dict)

print("=" * 60)
print("Robot Loaded Successfully")
print("=" * 60)

print(robot_cfg)