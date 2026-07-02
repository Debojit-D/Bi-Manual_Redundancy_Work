import mujoco
import mujoco.viewer

from bridge.config import MUJOCO_SCENE


class MujocoRobot:

    def __init__(self):

        self.model = mujoco.MjModel.from_xml_path(
            str(MUJOCO_SCENE)
        )

        self.data = mujoco.MjData(self.model)

    def step(self):

        mujoco.mj_step(self.model, self.data)

    def get_qpos(self):

        return self.data.qpos.copy()

    def set_qpos(self, q):

        self.data.qpos[:] = q

        mujoco.mj_forward(self.model, self.data)

    def sync(self):
        
        self.viewer.sync()

    def forward(self):

        mujoco.mj_forward(
            self.model,
            self.data,
        )

    def launch(self):

        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
        )

        return self.viewer

    def reset_home(self):
        """
        Reset to the model's 'home' keyframe if one exists.
        Falls back to zeros (with a warning) if it doesn't -
        raw zeros are NOT a valid Panda joint configuration
        (joint 4 / elbow has a negative-only range and cannot be 0).
        """
    
        key_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_KEY, "home"
        )
    
        if key_id == -1:
            print(
                "[MujocoRobot] WARNING: no 'home' keyframe found in the "
                "model - qpos stays at zeros, which is NOT a valid Panda "
                "start configuration for cuRobo planning."
            )
            return
    
        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        mujoco.mj_forward(self.model, self.data)