import torch

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel

from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig

from curobo.util_file import (
    get_robot_path,
    join_path,
    load_yaml,
)


class CuroboRobot:

    def __init__(self):

        self.tensor_args = TensorDeviceType()

        config = load_yaml(
            join_path(
                get_robot_path(),
                "franka.yml",
            )
        )["robot_cfg"]

        self.robot_cfg = RobotConfig.from_dict(
            config,
            self.tensor_args,
        )

        self.model = CudaRobotModel(
            self.robot_cfg.kinematics
        )

    def fk(self, q):

        q = torch.tensor(
            q,
            **self.tensor_args.as_torch_dict(),
        ).unsqueeze(0)

        return self.model.get_state(q)