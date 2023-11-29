#!/usr/bin/env python3

#################################################################################

#                               08/06/2023

#author: omarg

#En este codigo el control PD del cuadricoptero se ejecuta a traves de un rosrun y ya se puede #interrumpir con un crtl c.

#Se construyo basado en el ejemplo simple del publicador de ROS

# Este script tiene un control de orientacion en roll y pitch y yaw y control de altura.

#################################################################################


import sys
import time
import datetime
import signal
import rospy
import Control_utlis as utlis      #Funciones para envio de datos por UDP
import matplotlib.pyplot as plt

from std_msgs.msg import String, Bool, UInt16, Float64
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


import struct 
import socket

import xplane_ros.msg as xplane_msgs
import rosplane_msgs.msg as rosplane_msgs



filep = open('data_p.txt', 'w')
fileq = open('data_q.txt', 'w')
filer = open('data_r.txt', 'w')

filero = open('data_roll.txt', 'w')
filepi = open('data_pitch.txt', 'w')
filey = open('data_heading.txt', 'w')

fileer = open('data_e_roll.txt', 'w')
fileep = open('data_e_pitch.txt', 'w')
fileey = open('data_e_heading.txt', 'w')

filez = open('data_z.txt', 'w')
filevz = open('data_vz.txt', 'w')
fileez = open('data_e_z.txt', 'w')

filePDh = open('data_PD_h.txt', 'w')
filePDroll = open('data_PD_roll.txt', 'w')
filePDpitch = open('data_PD_pitch.txt', 'w')
filePDyaw = open('data_PD_yaw.txt', 'w')

fileM1 = open('data_M1.txt', 'w')
fileM2 = open('data_M2.txt', 'w')
fileM3 = open('data_M3.txt', 'w')
fileM4 = open('data_M4.txt', 'w')



QUADcon = None



UDP_PORT = 49005


# Open a Socket on UDP Port 49000
UDP_IP = "127.0.0.1"

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

    
class QUADController():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        self.saved_data = []

        self.yaw_init = None
        self.is_yaw_set = False

        self.time_init = None
        self.is_time_set = False

        self.q_des = 0
        self.p_des = 0
        self.r_des = 0
        self.q = 0
        self.p = 0
        self.r = 0
        self.altura = 0
        self.vz = 0

        self.pitch_des = 0
        self.roll_des = 0

        self.altura_des = -10
        self.velz_des = 0

        rospy.Subscriber("/xplane/flightmodel/odom", Odometry, self.odomcallback)
        rospy.Subscriber("/fixedwing/xplane/state", rosplane_msgs.State, self.callbackpqr)
        rospy.Subscriber("/xplane/flightmodel/global_state", xplane_msgs.GlobalState, self.callback)
    

	# La posicion puclicada en el Odometry y en el State es la misma con el mismo marco de referencia NED
	# la velocidad angular publicada en Odom es la misma que la p q r de State pero en Odom esta en grad/s y en state en
    # rad/s ambos comparten el mismo sistema de referencias.

    def plot_decoded_3(self, t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3):
        fig, axs = plt.subplots(1, 3, figsize=(15, 10))

        # Plot Input (first column, first row)
        axs[0].plot(t, data_1)
        axs[0].set_title(name_1)
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel(units_1)

        # Plot Graph 1 (second column, first row)
        axs[1].plot(t, data_2)
        axs[1].set_title(name_2)
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel(units_2)

        # Plot Graph 2 (third column, first row)
        axs[2].plot(t, data_3)
        axs[2].set_title(name_3)
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel(units_3)

        # plt.ylim(-0.01, 0.01)
        axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
        plt.tight_layout()
        plt.show()


           
    def odomcallback(self, data):
        self.vz = data.twist.twist.linear.z
        self.secs = data.header.stamp.secs

        #self.saved_data.append([self.vz])
        
    
    def callbackpqr(self, data):
        self.p = data.p
        self.q = data.q
        self.r = data.r
        self.altura = data.position[2]

        #self.saved_data.append(self.p)

    
    def callback(self, data):

        self.phi = data.roll 
        self.theta = data.pitch
        self.psi = data.heading

        # self.saved_data.append()

        if not self.is_yaw_set:               #Bandera para guardar el angulo de yaw inicial
            self.yaw_init = data.heading
            self.is_yaw_set = True  

        if not self.is_time_set:               #Bandera para guardar el angulo de yaw inicial
            self.time_init = time.time()
            self.is_time_set = True


        #data, addr = sock.recvfrom(1024)
        #self.values = DecodePacket(data)
    
        self.k_p = 0.03
        self.k_d = 0.07
        self.k_rp = 0.07
        self.k_rd = 0.66
        self.k_ph = 0.06
        self.k_dh = 0.19
        

        self.Throttle = 0.2

        self.pitch2 = self.theta*(3.14/180)
        self.roll2 = self.phi*(3.14/180)
        self.yaw2 = self.psi*(3.14/180)
        self.yaw_des = 4#self.yaw_init*(3.14/180)
    
        self.error_pitch = self.pitch2 - self.pitch_des
        self.error_roll = self.roll2 - self.roll_des 
        self.error_yaw = self.yaw2 - self.yaw_des
    
        self.error_q = self.q - self.q_des
        self.error_p = self.p - self.p_des 
        self.error_r = self.r - self.r_des

        self.error_altura = self.altura - self.altura_des
        self.error_velz = self.vz - self.velz_des

        self.actual_time = time.time() - self.time_init

        #print(self.altura)
        #print(self.actual_time)

        # Calculating the Control Law
    
        self.PDRoll =  self.error_roll*self.k_p + self.error_p*self.k_d
        self.PDPitch =  self.error_pitch*self.k_p*1.05 + self.error_q*self.k_d*1.9
        self.PDYaw = self.error_yaw*self.k_rp + self.error_r*self.k_rd

        self.PDAltura = self.error_altura*self.k_ph + self.error_velz*self.k_dh

        self.Throttle1 = self.PDAltura + self.PDRoll - self.PDPitch - self.PDYaw #+ self.Throttle
        self.Throttle2 = self.PDAltura + self.PDRoll + self.PDPitch + self.PDYaw #+ self.Throttle
        self.Throttle3 = self.PDAltura - self.PDRoll + self.PDPitch - self.PDYaw #+ self.Throttle
        self.Throttle4 = self.PDAltura - self.PDRoll - self.PDPitch + self.PDYaw #+ self.Throttle

        # For Recording only

        filep.write(f"{self.actual_time}\t{float(self.p)}\n")
        fileq.write(f"{self.actual_time}\t{float(self.q)}\n")
        filer.write(f"{self.actual_time}\t{float(self.r)}\n")

        filero.write(f"{self.actual_time}\t{float(self.roll2)}\n")
        filepi.write(f"{self.actual_time}\t{float(self.pitch2)}\n")
        filey.write(f"{self.actual_time}\t{float(self.yaw2)}\n")

        fileer.write(f"{self.actual_time}\t{float(self.error_roll)}\n")
        fileep.write(f"{self.actual_time}\t{float(self.error_pitch)}\n")
        fileey.write(f"{self.actual_time}\t{float(self.error_yaw)}\n")

        filez.write(f"{self.actual_time}\t{float(self.altura)}\n")
        filevz.write(f"{self.actual_time}\t{float(self.vz)}\n")
        fileez.write(f"{self.actual_time}\t{float(self.error_altura)}\n")

        filePDh.write(f"{self.actual_time}\t{float(self.PDAltura)}\n")
        filePDroll.write(f"{self.actual_time}\t{float(self.PDRoll)}\n")
        filePDpitch.write(f"{self.actual_time}\t{float(self.PDPitch)}\n")
        filePDyaw.write(f"{self.actual_time}\t{float(self.PDYaw)}\n")

        fileM1.write(f"{self.actual_time}\t{float(self.Throttle1)}\n")
        fileM2.write(f"{self.actual_time}\t{float(self.Throttle2)}\n")
        fileM3.write(f"{self.actual_time}\t{float(self.Throttle3)}\n")
        fileM4.write(f"{self.actual_time}\t{float(self.Throttle4)}\n")

        ####
    
    
        with utlis.XPlaneConnect() as client:
    
            self.data = [\
                    [25,self.Throttle1, self.Throttle2, self.Throttle3, self.Throttle4, -998, -998, -998, -998],\
                    [8, -998,  -998, -998,  -998, -998, -998, -998, -998],\
    	       ]
    
            client.sendDATA(self.data)

        # Saving data for plotting

        self.saved_data.append([self.actual_time])

        self.saved_data[-1].append(self.roll2)
        self.saved_data[-1].append(self.pitch2)
        self.saved_data[-1].append(self.yaw2)

        self.saved_data[-1].append(self.p)
        self.saved_data[-1].append(self.q)
        self.saved_data[-1].append(self.r)

        self.saved_data[-1].append(self.error_roll)
        self.saved_data[-1].append(self.error_pitch)
        self.saved_data[-1].append(self.error_yaw)

        self.saved_data[-1].append(self.error_p)
        self.saved_data[-1].append(self.error_q)
        self.saved_data[-1].append(self.error_r)

        self.saved_data[-1].append(self.PDAltura)
        self.saved_data[-1].append(self.PDRoll)
        self.saved_data[-1].append(self.PDPitch)
        self.saved_data[-1].append(self.PDYaw)

        self.saved_data[-1].append(self.Throttle1)
        self.saved_data[-1].append(self.Throttle2)
        self.saved_data[-1].append(self.Throttle3)
        self.saved_data[-1].append(self.Throttle4)




def handle_interrupt(signal, frame):
    # Time values
    t = QUADcon.saved_data

    time = [row[0] for row in t]
    roll = [row[1] for row in t]
    pitch = [row[2] for row in t]
    yaw = [row[3] for row in t]

    p = [row[4] for row in t]
    q = [row[5] for row in t]
    r = [row[6] for row in t]




    #print((t))

    # Call plot_decoded_3 to plot the saved data
    QUADcon.plot_decoded_3(time, roll, pitch, yaw,
                           'roll', 'rad', 'pitch', 'rad', 'yaw', 'rad')

    QUADcon.plot_decoded_3(time, p, q, r,
                           'p', 'rad/s', 'q', 'rad/s', 'r', 'rad/s')

    # Exit the program
    sys.exit(0)
    
def main():
    global QUADcon
    QUADcon = QUADController()
    signal.signal(signal.SIGINT, handle_interrupt)
    rospy.spin()
    #QUADcon.plot_decoded_3(self.actual_time, self.Throttle1, self.Throttle2, self.Throttle3)

if __name__ == '__main__':
    main()
