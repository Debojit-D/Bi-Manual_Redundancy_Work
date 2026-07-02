import numpy as np


class PandaState:

    ARM_DOF = 7

    @staticmethod
    def get_arm_qpos(mj_qpos: np.ndarray) -> np.ndarray:
        """
        Extract Panda arm joints from MuJoCo qpos.
        """
        return mj_qpos[: PandaState.ARM_DOF].copy()