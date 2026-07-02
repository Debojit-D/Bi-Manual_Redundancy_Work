import mujoco

model = mujoco.MjModel.from_xml_path(
    "/home/samay/mujoco_menagerie/franka_emika_panda/scene.xml"
)

print("Number of joints:", model.njnt)
print("Number of bodies:", model.nbody)
print("Number of geoms:", model.ngeom)
print("Number of actuators:", model.nu)