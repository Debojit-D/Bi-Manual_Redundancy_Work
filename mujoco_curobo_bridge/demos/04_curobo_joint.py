"""Load and summarize the standard Franka configuration with cuRobo v0.8."""

from curobo.kinematics import Kinematics, KinematicsCfg


def main() -> None:
    config = KinematicsCfg.from_robot_yaml_file("franka.yml")
    robot = Kinematics(config)

    print("=" * 60)
    print("Robot Loaded Successfully")
    print("=" * 60)
    print("cuRobo API: v0.8")
    print("Degrees of freedom:", robot.get_dof())
    print("Joint names:", robot.joint_names)
    print("Tool frames:", robot.tool_frames)
    print("Device:", config.device_cfg.device)


if __name__ == "__main__":
    main()
