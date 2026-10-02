
import mujoco_py as mj
import numpy as np
import os


XML_PATH = "sphere.xml"


def trajetoria(t):

    theta1 = np.pi/4 * np.sin(2*np.pi*t/5.0)
    theta2 = np.pi/2 * np.sin(2*np.pi*t/10.0)

    return np.array([theta1, theta2])



model = mj.load_model_from_path(XML_PATH)
sim = mj.MjSim(model)
model.opt.timestep = 0.001
viewer = mj.MjViewer(sim)

sim.data.qpos[0] = trajetoria(0)[0]
sim.data.qpos[1] = trajetoria(0)[1]

while True:

    t = sim.data.time
    print(f"time: {str(t)}")

    traj = trajetoria(t)
    sim.data.ctrl[0] = traj[0]
    sim.data.ctrl[1] = traj[1]
    print(f"desired position = {str(traj)} \n")

    for i in range(sim.data.ncon):
        # Note that the contact array has more than `ncon` entries,
        # so be careful to only read the valid entries.

        contact = sim.data.contact[i]
        if sim.model.geom_id2name(contact.geom2) == "robot" or sim.model.geom_id2name(contact.geom1) == "robot":
            print('contact:', i)
            print('distance:', contact.dist)
            print('geom1:', contact.geom1, sim.model.geom_id2name(contact.geom1))
            print('geom2:', contact.geom2, sim.model.geom_id2name(contact.geom2))
            print('contact position:', contact.pos)
        
            # Use internal functions to read out mj_contactForce
            c_array = np.zeros(6, dtype=np.float64)
            mj.functions.mj_contactForce(sim.model, sim.data, i, c_array)

            frame = np.array([[contact.frame[0], contact.frame[1], contact.frame[2]],
                            [contact.frame[3], contact.frame[4], contact.frame[5]],
                            [contact.frame[6], contact.frame[7], contact.frame[8]]])

            force = c_array[0:3]
            torque = c_array[3:6]
            #se o robo é o geom2, rotacionar o frame em Y para obter a força/torque atuando SOBRE o robo
            if (sim.model.geom_id2name(contact.geom2) == "robot"):
                rotation = np.array([[-1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, -1]])
                force = rotation @ force
                torque = rotation @ torque
                frame = rotation@frame

            print('contact force in contact frame:', force)
            print('contact torque in contact frame:', torque)
            print(f'contact frame: {frame} \n')
    sim.step()
    viewer.render()