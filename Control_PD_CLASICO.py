#!/usr/bin/env python3
#################################################################################
#                               30/01/2023
#author: Omar García
#github: https://github.com/OmarGAlcantara/MIL-Nengo-XPlane

# This is a sample script for a PD Control for a MIL simulation with XPlane 11 by using XPlaneConnect and XPlaneROS for the communication
# Simulation should be executed in the following order:
# 1. Opening a simulation in XPlane 11 with the intelAeroRTF model
# 2. Launching the XplaneROS wrapper by typing in the terminal: roslaunch xplane_ros default.launch
# 3. typing in a terminal in the file location : python3 Control_PD_CLASICO.py

#This controller effectively performs a quaqcopter's tracking task of an ascensional ramp and a circle using a classical PD Control strategy
#################################################################################


import sys
import time
import signal
import rospy
import Control_utlis as utlis  # Functions for UDP Communication
import plotters as pltr
import numpy as np
from nav_msgs.msg import Odometry
import socket
import xplane_ros.msg as xplane_msgs
import rosplane_msgs.msg as rosplane_msgs


QUADcon = None
UDP_PORT = 49005
# Open a Socket on UDP Port 49000
UDP_IP = "127.0.0.1"
sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP, UDP_PORT))


class QUADController():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        self.saved_data = []

        self.yaw_init = None
        self.is_yaw_set = False

        self.time_init = None
        self.is_time_set = False

        self.q = 0
        self.p = 0
        self.r = 0
        self.x = 0
        self.y = 0
        self.altura = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.pitch_des = 0
        self.roll_des = 0

        self.reference_time = time.time()
        self.actual_time = 0
        self.altura_des = 0
        self.x_d = 0
        self.y_d = 0
        self.velz_des = 0

        self.sumEr_p = 0
        self.Er0_p = 0
        self.sumEr_q = 0
        self.Er0_q = 0
        self.sumEr_r = 0
        self.Er0_r = 0
        self.sumEr_h = 0
        self.sumEr_h_norm = 0

        self.Int_z = 0
        self.ez = 0

        rospy.Subscriber("/xplane/flightmodel/odom", Odometry, self.odomcallback)
        rospy.Subscriber("/fixedwing/xplane/state", rosplane_msgs.State, self.callbackpqr)
        rospy.Subscriber("/xplane/flightmodel/global_state", xplane_msgs.GlobalState, self.callback)


    def reference(self):
        current_time = time.time() - self.reference_time

        if current_time < 5:
            return -0.5, 0
        elif current_time < 35:
            print('Tracking Ascension Ramp')
            ramp_time = current_time - 5
            ramp_value = -0.5 + ((-30 - (-0.5)) / 30) * ramp_time
            return ramp_value, (-30 - (-0.5)) / 30
        else:
            return -30, 0

    def referencex(self, x):
        current_timex = time.time() - self.reference_time

        if current_timex < 5:
            return 0
        elif current_timex < 35:
            ramp_timex = current_timex - 5
            ramp_valuex = 0.66 * ramp_timex
            return ramp_valuex
        elif current_timex < 50:
            return 19.5
        elif current_timex < 120:
            r = 19.5
            circleTime = 0.2 * (time.time() - self.reference_time)  # Adjust the scaling factor (0.1) as needed
            circle = r * np.cos(circleTime - np.pi * 1.2)
            return circle

    def referencey(self, x):
        current_timex = time.time() - self.reference_time

        if current_timex < 5:
            return 0
        elif current_timex < 35:
            ramp_timex = current_timex - 5
            ramp_valuex = 0.66 * ramp_timex
            return ramp_valuex
        elif current_timex < 50:
            return 19.5
        elif current_timex < 120:
            print('Tracking Circular reference')
            r = 19.5
            circleTimey = 0.2 * (time.time() - self.reference_time)  # Adjust the scaling factor (0.1) as needed
            circley = r * np.sin(circleTimey - np.pi * 1.2) + 19.5
            return circley

    def odomcallback(self, data):

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        self.vx = data.twist.twist.linear.x
        self.vy = data.twist.twist.linear.y
        self.vz = data.twist.twist.linear.z
        self.secs = data.header.stamp.secs

    def callbackpqr(self, data):
        self.p = data.p
        self.q = data.q
        self.r = data.r
        self.altura = data.position[2]

    def callback(self, data):

        self.phi = data.roll
        self.theta = data.pitch
        self.psi = data.heading

        if not self.is_yaw_set:  # Flag for saving initial yaw
            self.yaw_init = data.heading
            self.is_yaw_set = True

        if not self.is_time_set:  # Flag for saving initial time
            self.time_init = time.time()
            self.is_time_set = True

        self.k_ph = 0.02 * 8
        self.k_dh = 0.06 * 4.0
        self.kh_i = 0

        self.kpx = 0.03 * 2.6
        self.kdx = 0.08 * 1.9
        self.kpy = -0.03 * 4.5 * 1
        self.kdy = -0.11 * 1.2 * 1.9

        self.k_phi = 2.9  # * 0.1
        self.k_theta = 2.9  # 5# * 0.1
        self.k_psi = -1.4

        self.kp_p = 0.045 * 4
        self.kp_d = 0.035 * 2
        self.kp_i = 0

        self.kq_p = 0.06 * 4
        self.kq_d = 0.035 * 3
        self.kq_i = 0

        self.kr_p = 0.25
        self.kr_d = 0.25
        self.kr_i = 0.0

        self.Int_Error_z = 0.0
        self.dt = 0.05

        ###########    Calculating the control law    ##################

        # Calculating Phi desired and Theta desired

        self.x_d = self.referencex(self.x)
        self.y_d = self.referencey(self.y)

        self.error_x = self.x - self.x_d
        self.error_y = self.y - self.y_d
        self.error_velx = self.vx - 0
        self.error_vely = self.vy - 0

        self.phi_d = self.kpy * self.error_y + self.kdy * self.error_vely
        self.theta_d = self.kpx * self.error_x + self.kdx * self.error_velx

        # Calculating Phi_dot_ desired and Theta_dot_ desired and psi_dot_desired

        self.phi_rad = self.phi * (3.14 / 180)
        self.theta_rad = self.theta * (3.14 / 180)
        self.psi_rad = self.psi * (3.14 / 180)
        self.psi_rad_d = 6

        self.error_roll = -self.phi_rad + self.phi_d
        self.error_pitch = -self.theta_rad + self.theta_d

        # Wrap Psi Angle

        if self.psi_rad_d < np.pi / 2 and self.psi_rad > 3 * np.pi / 2:
            self.error_yaw = -(self.psi_rad_d + (2 * np.pi - self.psi_rad))
            if abs(self.error_yaw) > np.pi:
                self.error_yaw = -self.error_yaw
        elif self.psi_rad < np.pi / 2 and self.psi_rad_d > 3 * np.pi / 2:
            self.error_yaw = -(-(2 * np.pi - self.psi_rad_d) - self.psi_rad)
            if abs(self.error_yaw) > np.pi:
                self.error_yaw = -self.error_yaw
        else:
            self.error_yaw = self.psi_rad - self.psi_rad_d

        self.phi_dot_d = self.k_phi * self.error_roll
        self.theta_dot_d = self.k_theta * self.error_pitch
        self.psi_dot_d = self.k_psi * self.error_yaw

        # Tranforming into body axes

        self.eta_dot = np.array([self.psi_dot_d, self.theta_dot_d, self.phi_dot_d])

        ROT = np.array([[-np.sin(self.theta_rad), 0, 1],
                        [np.cos(self.theta_rad) * np.sin(self.phi_rad), np.cos(self.phi_rad), 0],
                        [np.cos(self.theta_rad) * np.cos(self.phi_rad), -np.sin(self.phi_rad), 0]])

        Omega_d = np.dot(ROT, self.eta_dot)

        self.error_p = -self.p + Omega_d[0]
        self.error_q = -self.q + Omega_d[1]
        self.error_r = -(self.r - Omega_d[2])

        # Derivating and Integrating the error

        P_p = self.kp_p * self.error_p  # (eq 28) [1]
        Int_p = (self.error_p + self.sumEr_p) * self.dt
        I_p = self.kp_i * Int_p  # (eq 28) [1]
        Der_p = ((self.error_p - self.Er0_p) / self.dt)
        D_p = self.kp_d * Der_p  # (eq 28) [1]
        self.Er0_p = self.error_p  # update Er0
        self.sumEr_p = self.error_p + self.sumEr_p  # update sumEr
        self.PID_p = P_p + I_p + D_p

        P_q = self.kq_p * self.error_q  # (eq 28) [1]
        Int_q = (self.error_q + self.sumEr_q) * self.dt
        I_q = self.kq_i * Int_q  # (eq 28) [1]
        Der_q = ((self.error_q - self.Er0_q) / self.dt)
        D_q = self.kq_d * Der_q  # (eq 28) [1]
        self.Er0_q = self.error_q  # update Er0
        self.sumEr_q = self.error_q + self.sumEr_q  # update sumEr
        self.PID_q = P_q + I_q + D_q

        P_r = self.kr_p * self.error_r  # (eq 28) [1]
        Int_r = (self.error_r + self.sumEr_r) * self.dt
        I_r = self.kr_i * Int_r  # (eq 28) [1]
        Der_r = ((self.error_r - self.Er0_r) / self.dt)
        D_r = self.kr_d * Der_r  # (eq 28) [1]
        self.Er0_r = self.error_r  # update Er0
        self.sumEr_r = self.error_r + self.sumEr_r  # update sumEr
        self.PID_r = (P_r + I_r + D_r)

        # Altitude Control
        _, self.velz_des = self.reference()
        self.altura_des, _ = self.reference()
        self.error_z = self.altura - self.altura_des
        self.error_velz = self.vz - self.velz_des

        #Integrating the error
        self.dt2 = 0.016
        self.Int = (self.error_z + self.ez) * self.dt2 / 2  # Calculate Integral
        self.Int_z = self.Int_z + self.Int  # Acumulate Integral
        self.ez = self.error_z  # Variable Update
        I_h = self.kh_i * self.Int_z

        P_h = self.error_z * self.k_ph
        D_h = self.error_velz * self.k_dh

        self.PDAltura = P_h + D_h + I_h

        # Mixer ( See pdf attached for motor configuration )

        self.Throttle1 = self.PDAltura - self.PID_p + self.PID_q + self.PID_r
        self.Throttle2 = self.PDAltura - self.PID_p - self.PID_q - self.PID_r
        self.Throttle3 = self.PDAltura + self.PID_p - self.PID_q + self.PID_r
        self.Throttle4 = self.PDAltura + self.PID_p + self.PID_q - self.PID_r

        with utlis.XPlaneConnect() as client:

            self.data = [ \
                [25, self.Throttle1, self.Throttle2, self.Throttle3, self.Throttle4, -998, -998, -998, -998], \
                [8, -998, -998, -998, -998, -998, -998, -998, -998], \
                ]

            client.sendDATA(self.data)

        # Saving data for plotting

        self.actual_time = time.time() - self.time_init

        self.saved_data.append([self.actual_time])
        self.saved_data[-1].append(self.error_x)
        self.saved_data[-1].append(self.error_y)
        self.saved_data[-1].append(self.error_z)
        self.saved_data[-1].append(self.error_velx)
        self.saved_data[-1].append(self.error_vely)
        self.saved_data[-1].append(self.error_velz)
        self.saved_data[-1].append(self.Int_z)
        self.saved_data[-1].append(self.phi_rad)
        self.saved_data[-1].append(self.theta_rad)
        self.saved_data[-1].append(self.psi_rad)
        self.saved_data[-1].append(self.p)
        self.saved_data[-1].append(self.q)
        self.saved_data[-1].append(self.r)
        self.saved_data[-1].append(self.psi_rad_d)
        self.saved_data[-1].append(-self.altura)
        self.saved_data[-1].append(-self.altura_des)
        self.saved_data[-1].append(self.phi_dot_d)
        self.saved_data[-1].append(self.theta_dot_d)
        self.saved_data[-1].append(self.psi_dot_d)
        self.saved_data[-1].append(self.error_roll)
        self.saved_data[-1].append(self.error_pitch)
        self.saved_data[-1].append(self.error_yaw)
        self.saved_data[-1].append(Omega_d[0])
        self.saved_data[-1].append(Omega_d[1])
        self.saved_data[-1].append(Omega_d[2])
        self.saved_data[-1].append(self.p)
        self.saved_data[-1].append(self.PID_r)
        self.saved_data[-1].append(self.error_p)
        self.saved_data[-1].append(self.error_q)
        self.saved_data[-1].append(self.error_r)
        self.saved_data[-1].append(Der_p)
        self.saved_data[-1].append(Der_q)
        self.saved_data[-1].append(Der_r)
        self.saved_data[-1].append(self.PDAltura)
        self.saved_data[-1].append(self.PID_p)
        self.saved_data[-1].append(self.PID_q)
        self.saved_data[-1].append(self.PID_r)
        self.saved_data[-1].append(self.Throttle1)
        self.saved_data[-1].append(self.Throttle2)
        self.saved_data[-1].append(self.Throttle3)
        self.saved_data[-1].append(self.Throttle4)
        self.saved_data[-1].append(self.phi_d)
        self.saved_data[-1].append(self.theta_d)
        self.saved_data[-1].append(self.x)
        self.saved_data[-1].append(self.y)
        self.saved_data[-1].append(self.x_d)
        self.saved_data[-1].append(self.y_d)
        self.saved_data[-1].append(self.q)
        self.saved_data[-1].append(self.vx)
        self.saved_data[-1].append(self.vy)


def handle_interrupt(signal, frame):
    # This function is used to graph
    t = QUADcon.saved_data

    actual_time = [row[0] for row in t]
    error_x = [row[1] for row in t]
    error_y = [row[2] for row in t]
    error_z = [row[3] for row in t]
    error_velx = [row[4] for row in t]
    error_vely = [row[5] for row in t]
    error_velz = [row[6] for row in t]
    Int_Error_z = [row[7] for row in t]
    phi_rad = [row[8] for row in t]
    theta_rad = [row[9] for row in t]
    psi_rad = [row[10] for row in t]
    p = [row[11] for row in t]
    q = [row[12] for row in t]
    r = [row[13] for row in t]
    psi_rad_d = [row[14] for row in t]
    z = [row[15] for row in t]
    z_des = [row[16] for row in t]
    phi_dot_D = [row[17] for row in t]
    theta_dot_D = [row[18] for row in t]
    psi_dot_D = [row[19] for row in t]
    e_phi = [row[20] for row in t]
    e_theta = [row[21] for row in t]
    e_psi = [row[22] for row in t]
    pd = [row[23] for row in t]
    qd = [row[24] for row in t]
    rd = [row[25] for row in t]
    p_filt = [row[26] for row in t]
    PIDR = [row[27] for row in t]
    error_p = [row[28] for row in t]
    error_q = [row[29] for row in t]
    error_r = [row[30] for row in t]
    der_p = [row[31] for row in t]
    der_q = [row[32] for row in t]
    der_r = [row[33] for row in t]
    PDAlt = [row[34] for row in t]
    PDp = [row[35] for row in t]
    PDq = [row[36] for row in t]
    PDr = [row[37] for row in t]
    T1 = [row[38] for row in t]
    T2 = [row[39] for row in t]
    T3 = [row[40] for row in t]
    T4 = [row[41] for row in t]
    phi_d = [row[42] for row in t]
    theta_d = [row[43] for row in t]
    x = [row[44] for row in t]
    y = [row[45] for row in t]
    x_d = [row[46] for row in t]
    y_d = [row[47] for row in t]
    q_filt = [row[48] for row in t]
    v_x = [row[49] for row in t]
    v_y = [row[50] for row in t]

    pltr.plot_3(actual_time, error_x[:len(actual_time)], error_y[:len(actual_time)], error_z[:len(actual_time)],
                'Error x', 'Meters (m)', 'Error y', 'Meters(m)', 'Error z', 'Meters (m)', 'Classical PD Position Errors')
    pltr.plot_3d_trajectory(actual_time,x[:len(actual_time)],y[:len(actual_time)],z[:len(actual_time)],x_d[:len(actual_time)],y_d[:len(actual_time)],z_des[:len(actual_time)],'x','[m]','y', '[m]','z','[m]','Classical PD 3D Trajectory')

    pltr.plot_2in1_3(actual_time, x[:len(actual_time)], x_d[:len(actual_time)], y[:len(actual_time)],
                     y_d[:len(actual_time)], z[:len(actual_time)], z_des[:len(actual_time)], 'x', 'm', 'xd', 'm', 'y',
                     'm', 'y_d', 'm', 'z', 'm', 'z_d', 'm', 'Classical PD Position vs Desired')
    pltr.plot_xz_plane(actual_time,x, y, x_d, y_d, 'x', 'm', 'y', 'm', 'Classical PD XY position')

    sys.exit(0)


def main():
    global QUADcon
    QUADcon = QUADController()
    signal.signal(signal.SIGINT, handle_interrupt)
    rospy.spin()

if __name__ == '__main__':
    main()
