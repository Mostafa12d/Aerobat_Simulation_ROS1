import mujoco as mj
import mujoco_viewer
import time

model = mj.MjModel.from_xml_path('/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/src/ros_flappy_sim/worlds/Flappy_v10_weld.xml')
data = mj.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)
viewer.cam.distance = 2
viewer.cam.azimuth = 180
viewer.cam.elevation = -20

for _ in range(1000):
    mj.mj_step(model, data)
    viewer.render()
    time.sleep(0.001)

viewer.close()