import mujoco_py as mj

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

import numpy as np

XML="""
<mujoco model="spherical_robot">
    <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>


    <worldbody>
        <geom name="floor" type="plane" size="0 0 0.05" rgba="0.8 0.8 0.8 1" condim="3"/>


        <body name="robot" pos="0 0 0.11">

            <joint name="slide_x" type="slide" axis="1 0 0" damping="0.5"/>
            <joint name="slide_y" type="slide" axis="0 1 0" damping="0.5"/>
            
            <geom name="robot" type="sphere" size="0.1" mass="1.0" rgba="0.1 0.6 0.9 1" condim="3"/>
        </body>

        <body name="wall_north" pos="0 3 0.25">
            <geom type="box" size="3 0.1 0.25" rgba="0.3 0.3 0.3 1"/>
        </body>
        <body name="wall_south" pos="0 -3 0.25">
            <geom type="box" size="3 0.1 0.25" rgba="0.3 0.3 0.3 1"/>
        </body>
        <body name="wall_east" pos="3 0 0.25">
            <geom type="box" size="0.1 3 0.25" rgba="0.3 0.3 0.3 1"/>
        </body>
        <body name="wall_west" pos="-3 0 0.25">
            <geom type="box" size="0.1 3 0.25" rgba="0.3 0.3 0.3 1"/>
        </body>

        <body name="obstacle_box_1" pos="1.0 1.0 0.2">
            <geom type="box" size="0.3 0.3 0.2" rgba="0.8 0.2 0.2 1" mass="5.0"/>
        </body>
        <body name="obstacle_box_2" pos="-1.2 0.5 0.15">
            <geom type="box" size="0.4 0.2 0.15" rgba="0.2 0.8 0.2 1" mass="5.0"/>
        </body>
        <body name="obstacle_box_3" pos="0.0 -1.5 0.25">
            <geom type="box" size="0.5 0.2 0.25" rgba="0.8 0.8 0.2 1" mass="5.0"/>
        </body>
    </worldbody>

    
    <actuator>
        
        <motor name="actuator_x" joint="slide_x" gear="10.0"/>
        <motor name="actuator_y" joint="slide_y" gear="10.0"/>
    </actuator>
</mujoco>
"""

def trajetoria(t):

    theta1 = 2 * np.sin(2*np.pi*t/5.0)
    theta2 = 2* np.sin(2*np.pi*t/10.0)

    return np.array([theta1, theta2])



class MuJoCoSim(Node):

    def __init__(self):
        super().__init__('mujoco_node')
        self.get_logger().info('mujoco_node started')

        self.dt = 0.001

        self.model = mj.load_model_from_xml(XML)
        self.sim = mj.MjSim(self.model)
        self.model.opt.timestep = self.dt
        self.viewer = mj.MjViewer(self.sim)

        self.n_controls = self.model.nu

        self.sim.data.qpos[0] = trajetoria(0)[0]
        self.sim.data.qpos[1] = trajetoria(0)[1]

        self.simulation_timer = self.create_timer(self.dt, self.update_sim)
        self.viewer_timer = self.create_timer(0.01,  self.viewer.render)

        self.contact_publisher = self.create_publisher(Float64MultiArray, 'contact_forces', 10)


        self.vel_subscription = self.create_subscription( Twist,
                                                          'cmd_vel',
                                                          self.vel_callback,
                                                          10)


        self.contact_broadcaster = TransformBroadcaster(self)

        self.contact_forces_cf = np.zeros(6) #forças de contato no frame de contato
        self.contact_frame = np.zeros((3, 3))
        self.frame_transform = np.zeros((6, 6)) #Para transformar um vetor 6X1 de força + torque
        self.contact_forces_rf = np.zeros(6) #forças de contato no frame do robo
        self.contact = Float64MultiArray()

        self.xpos = 0.0
        self.ypos = 0.0
        self.ang_pos = 0.0

    def update_sim(self):

        t = self.sim.data.time

        self.sim.data.qpos[0] = self.xpos
        self.sim.data.qpos[1] = self.ypos

        for i in range(self.sim.data.ncon):
            # Note that the contact array has more than `ncon` entries,
            # so be careful to only read the valid entries.

            contact = self.sim.data.contact[i]
            if self.sim.model.geom_id2name(contact.geom2) == "robot" or self.sim.model.geom_id2name(contact.geom1) == "robot":
                self.get_logger().info(f'contact: {i}')
                self.get_logger().info(f'distance: {contact.dist}')
                self.get_logger().info(f'geom1: {self.sim.model.geom_id2name(contact.geom1)}')
                self.get_logger().info(f'geom2: {self.sim.model.geom_id2name(contact.geom2)}')
                self.get_logger().info(f'contact position: {contact.pos}')
            
                # Use internal functions to read out mj_contactForce
                
                mj.functions.mj_contactForce(self.sim.model, self.sim.data, i, self.contact_forces_cf)

                self.contact_frame = np.array([[contact.frame[0], contact.frame[1], contact.frame[2]],
                                               [contact.frame[3], contact.frame[4], contact.frame[5]],
                                               [contact.frame[6], contact.frame[7], contact.frame[8]]])

                self.frame_transform = np.block([[self.contact_frame, np.zeros((3, 3))],
                                                 [np.zeros((3, 3)), self.contact_frame]])
                self.contact_forces_rf = np.linalg.inv(self.frame_transform) @ self.contact_forces_cf

                self.get_logger().info(f'contact force in robot\'s frame: {self.contact_forces_cf[0:3]}')
                self.get_logger().info(f'contact torque in robot\'s frame: {self.contact_forces_cf[3:6]} \n')
                
                self.contact.data = self.contact_forces_rf.tolist()
                self.contact_publisher.publish(self.contact)


        self.sim.step()

    def vel_callback(self, msg):
        self.ang_pos += msg.angular.z * self.dt
        self.xpos += msg.linear.x * np.cos(self.ang_pos) * self.dt
        self.ypos += msg.linear.x * np.sin(self.ang_pos) * self.dt
    




def main():
    rclpy.init()
    node = MuJoCoSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


