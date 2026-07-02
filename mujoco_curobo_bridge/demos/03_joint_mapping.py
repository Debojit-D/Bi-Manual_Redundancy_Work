import mujoco

MODEL_PATH = "/home/samay/mujoco_menagerie/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

print("=" * 70)
print("JOINT INFORMATION")
print("=" * 70)

for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)

    print(f"\nJoint {i}")
    print(f"Name        : {name}")
    print(f"Type        : {model.jnt_type[i]}")
    print(f"qpos index  : {model.jnt_qposadr[i]}")
    print(f"qvel index  : {model.jnt_dofadr[i]}")

print("\n")
print("=" * 70)
print("INITIAL qpos")
print("=" * 70)
print(data.qpos)