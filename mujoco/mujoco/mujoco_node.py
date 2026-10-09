import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup    

import mujoco_py as mj
import numpy as np


class MuJoCoSim(Node):

    def __init__(self):

        super().__init__('mujoco_node')
        self.get_logger().info('mujoco_node started')

        self.declare_parameter('xmlPath', '')
        self.declare_parameter('dt', 0.001)

        self.xmlPath = self.get_parameter('xmlPath').get_parameter_value().string_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        self.get_logger().info(self.xmlPath)

        self.model = mj.load_model_from_path(self.xmlPath)
        self.sim = mj.MjSim(self.model)
        self.model.opt.timestep = self.dt
        self.viewer = mj.MjViewer(self.sim)

        self.n_controls = self.model.nu

        self.simulation_step_cb_group = MutuallyExclusiveCallbackGroup()

        self.simulation_timer = self.create_timer(self.dt,
                                                  self.update_sim,
                                                  )
    
        self.gui_timer = self.create_timer(0.01, self.viewer.render)

        self.contact_publisher = self.create_publisher(Float64MultiArray,
                                                       'contact_forces',
                                                       10,

                                                       )


        self.vel_subscription = self.create_subscription( Twist,
                                                          'cmd_vel',
                                                          self.vel_callback,
                                                          10)


        self.contact_forces_cf = np.zeros(6) #forças de contato no frame de contato
        self.contact_frame = np.zeros((3, 3))
        self.frame_transform = np.zeros((6, 6)) #Para transformar um vetor 6X1 de força + torque
        self.contact_forces_rf = np.zeros(6) #forças de contato no frame do robo
        self.contact = Float64MultiArray()

        self.xpos = 0.0
        self.ypos = 0.0


    def update_sim(self):

        self.sim.data.qpos[0] = self.xpos
        self.sim.data.qpos[1] = self.ypos
        self.contact.data = [0.0, 0.0]

        for i in range(self.sim.data.ncon):
            # Note that the contact array has more than `ncon` entries,
            # so be careful to only read the valid entries.

            contact = self.sim.data.contact[i]
            geom1 = self.sim.model.geom_id2name(contact.geom1)
            geom2 = self.sim.model.geom_id2name(contact.geom2)
            if geom2 == "robot" or geom1 == "robot":

                mj.functions.mj_contactForce(self.sim.model, self.sim.data, i, self.contact_forces_cf)

                self.contact_frame = np.array([[contact.frame[0], contact.frame[1], contact.frame[2]],
                                               [contact.frame[3], contact.frame[4], contact.frame[5]],
                                               [contact.frame[6], contact.frame[7], contact.frame[8]]])

                self.frame_transform = np.block([[self.contact_frame, np.zeros((3, 3))],
                                                 [np.zeros((3, 3)), self.contact_frame]])
                self.contact_forces_rf = self.frame_transform.T @ self.contact_forces_cf
                
                self.contact.data = self.contact_forces_rf.tolist()

                self.contact_force = self.contact_forces_rf[:2]


        self.contact_publisher.publish(self.contact)
        self.sim.step()

    def vel_callback(self, msg):

        if self.contact.data[0]*msg.linear.x  <= 0.0 : self.xpos += msg.linear.x * self.dt
        if self.contact.data[1]*msg.angular.z <= 0.0 : self.ypos += msg.angular.z * self.dt


def main():
    rclpy.init()
    node = MuJoCoSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


