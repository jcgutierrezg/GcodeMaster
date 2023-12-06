import os 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import turtle
import tkinter as tk
from tkinter import filedialog
from pathlib import Path
import sys
import time
import threading
import time
from threading import Thread

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
    
from geometry_msgs.msg import Pose


from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import DisplayTrajectory

import numpy as np # Scientific computing library for Python
import math

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  
  rollR = math.radians(roll)
  pitchR = math.radians(pitch)
  yawR = math.radians(yaw)
  
  q = [0] * 4
  
  q[1] = np.sin(rollR/2) * np.cos(pitchR/2) * np.cos(yawR/2) - np.cos(rollR/2) * np.sin(pitchR/2) * np.sin(yawR/2)
  q[2] = np.cos(rollR/2) * np.sin(pitchR/2) * np.cos(yawR/2) + np.sin(rollR/2) * np.cos(pitchR/2) * np.sin(yawR/2)
  q[3] = np.cos(rollR/2) * np.cos(pitchR/2) * np.sin(yawR/2) - np.sin(rollR/2) * np.sin(pitchR/2) * np.cos(yawR/2)
  q[0] = np.cos(rollR/2) * np.cos(pitchR/2) * np.cos(yawR/2) + np.sin(rollR/2) * np.sin(pitchR/2) * np.sin(yawR/2)
 
  return q

########################################################################################################################

class Gcode_interface(Node):
    global height, width

    def update_ori(self):

        quat = get_quaternion_from_euler(self.roll, self.pitch, self.yaw)

        self.ori_w = quat[0]
        self.ori_x = quat[1]
        self.ori_y = quat[2]
        self.ori_z = quat[3]

    def calculate_radius(self, cx, cy, x, y):
        return math.sqrt((cx - x)**2 + (cy - y)**2)

    def draw_point(self, x, y):

        radius = 2
        color="black"

        self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=color, outline=color)

    def calculate_arc_angles(self, x1, y1, x2, y2, cx, cy, radius, clockwise):
        # Calculate angles in radians
        start_angle = math.atan2(y1 - cy, x1 - cx)
        end_angle = math.atan2(y2 - cy, x2 - cx)

        # Ensure angles are positive
        #start_angle = (start_angle + 2 * math.pi) % (2 * math.pi)
        #end_angle = (end_angle + 2 * math.pi) % (2 * math.pi)
        #print(f"Start Angle: {math.degrees(start_angle)} degrees, End Angle: {math.degrees(end_angle)} degrees")

        # If clockwise, swap start and end angles
        if clockwise:
            start_angle, end_angle = end_angle, start_angle

        # Convert angles to degrees
        start_angle_deg = math.degrees(start_angle)
        end_angle_deg = math.degrees(end_angle)

        return start_angle_deg, end_angle_deg

    def parse_arc_command(self, command):

        parts = command.split()
        x, y, i, j = None, None, None, None
        for part in parts:
            if part.startswith("X"):
                x = float(part[1:])
            elif part.startswith("Y"):
                y = -float(part[1:])
            elif part.startswith("I"):
                i = float(part[1:])
            elif part.startswith("J"):
                j = -float(part[1:])
        return x, y, i, j

    def draw_arc(self, x, y, i, j, clockwise=True):
        # Calculate the bounding box for the arc

        cx = self.cursorX + i
        cy = self.cursorY + j

        x0 = self.cursorX
        y0 = self.cursorY

        x1 = x
        y1 = y

        #print(f"cx: {cx}, cy: {cy}, x0: {x0}, y0: {y0}, x1: {x1}, y1: {y1}")

        radius = self.calculate_radius(cx, cy, x0, y0)
        #radius_from_goal = self.calculate_radius(cx, cy, x1, y1)

        #print(f"Radius from current position: {radius_from_current}")
        #print(f"Radius from goal position: {radius_from_goal}")

        start_angle, end_angle = self.calculate_arc_angles(x0, y0, x1, y1, cx, cy, radius, clockwise)
        #print(f"Start Angle: {start_angle} degrees, End Angle: {end_angle} degrees")

        start_angle = -start_angle
        end_angle = -end_angle

        x0 = cx - radius
        y0 = cy - radius

        x1 = cx + radius
        y1 = cy + radius

        extent_angle = end_angle - start_angle

        if(abs(extent_angle)>180.0):

            extent_angle = end_angle + start_angle

        #print(extent_angle)

        # Draw the arc
        self.draw_point(x, y)
        self.canvas.create_arc(x0, y0, x1, y1, start=start_angle, extent=extent_angle, outline="red", width=2, style=tk.ARC)

        #x1, y1, x2, y2 = 50, 50, 150, 100
        #fill_color = "blue"
        #self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill_color)

    def draw_gcode(self, line):

        #x, y = 0, 0  # Initial position
        if line.startswith("G1"):
            parts = line.split()
            for part in parts:
                if part.startswith("X"):
                    x = float(part[1:])
                elif part.startswith("Y"):
                    y = -float(part[1:])
            # Draw a line from the current position to (x, y)
            if self.drawing:
                self.canvas.create_line(self.cursorX, self.cursorY, x, y, width=2, fill="blue")
            else:
                self.canvas.create_line(self.cursorX, self.cursorY, x, y, width=2, fill="green", dash=(4, 2))

            self.draw_point(x, y)
            #print(f"drawn line from X{self.cursorX} to X{x} and Y{self.cursorY} to Y{y}.")
            self.cursorX = x
            self.cursorY = y

        elif (line.startswith("G2") and not line.startswith("G21") and not line.startswith("G20")):
            x, y, i, j = self.parse_arc_command(line)
            self.draw_arc(x, y, i, j, clockwise=True)
            self.cursorX = x
            self.cursorY = y

        elif line.startswith("G3"):
            x, y, i, j = self.parse_arc_command(line)
            self.draw_arc(x, y, i, j, clockwise=False)
            self.cursorX = x
            self.cursorY = y

        elif line.startswith("M5"):
            self.drawing = False

        elif line.startswith("M3"):
            self.drawing = True


    def define_plane(self, p1, p2, p3):

        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]])
        v2 = np.array([p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]])

        n = np.cross(v1, v2)
        n /= np.linalg.norm(n)

        # Define the plane equation: Ax + By + Cz = D
        A, B, C = n
        D = np.dot(n, p1)

        self.A_plane = A
        self.B_plane = B
        self.C_plane = C
        self.D_plane = D


    def boton1(self):

        print ("Boton1")
        gcode_filename = filedialog.askopenfilename(initialdir='~/Tesis_IMEC/gcodes/')
        print(gcode_filename)
        self.gcodetext = os.path.basename(gcode_filename)

        text_label = tk.Label(self.root, text=self.gcodetext, font=("Arial", 16), fg="white", bg=self.backgndColor)
        text_label.place(x=50, y=50)

        filemsg = String()
        filemsg.data = gcode_filename
        #self.canvas.postscript(file=ruta, colormode='color')
        self.filename.publish(filemsg)

        #thread = threading.Thread(target=rclpy.spin(self))
        #thread.start()
        pass
        
    
    def boton2(self):
        print ("Auto")

        pass
        
    def boton3(self):
        print ("Plan/Exec")

        strmsg = String()
        strmsg.data = 'planexec'
        self.plan_exec.publish(strmsg)

        boolmsg = Bool()
        boolmsg.data = True
        self.nexttraj.publish(boolmsg)
        #self.nextgcode.publish(boolmsg)

        self.executeDone = False

        self.button3['state'] = 'disabled'
        self.button4['state'] = 'disabled'
        self.button5['state'] = 'disabled'
        self.button5a['state'] = 'disabled'

        self.planFlag = False
        
        pass

    def boton4(self):
        print ("Plan")
        strmsg = String()
        strmsg.data = 'plan'
        self.plan_exec.publish(strmsg)

        #self.button4['bg'] = self.greenColor
        #self.button5['bg'] = self.greenColor

        self.planFlag = True
        pass

    def boton5(self):
        print ("Execute")
        strmsg = String()
        strmsg.data = 'execute'
        self.plan_exec.publish(strmsg)

        boolmsg = Bool()
        boolmsg.data = True
        self.nexttraj.publish(boolmsg)
        #self.nextgcode.publish(boolmsg)

        self.executeDone = False

        self.button3['state'] = 'disabled'
        self.button4['state'] = 'disabled'
        self.button5['state'] = 'disabled'
        self.button5a['state'] = 'disabled'

        self.planFlag = False

        pass

    def boton5a(self):
        print("Next Line")
        boolmsg = Bool()
        boolmsg.data = True
        #self.nexttraj.publish(boolmsg)
        self.nextgcode.publish(boolmsg)
        self.button5a['bg'] = self.greenColor
        pass

    def boton6(self):
        print ("1mm")
        self.pos_resolution = 1.0/1000
        self.ori_resolution = 1.0
        self.vel_resolution = 0.001
        self.manual_resolution = 0.1

        self.button7['bg'] = self.buttonColor
        self.button6['bg'] = self.blueColor
        self.button8['bg'] = self.buttonColor
        pass

    def boton7(self):
        print ("10mm")
        self.pos_resolution = 10.0/1000
        self.ori_resolution = 10.0
        self.vel_resolution = 0.01
        self.manual_resolution = 1.0

        self.button7['bg'] = self.blueColor
        self.button6['bg'] = self.buttonColor
        self.button8['bg'] = self.buttonColor
        pass

    def boton8(self):
        print ("100mm")
        self.pos_resolution = 100.0/1000
        self.ori_resolution = 45.0
        self.vel_resolution = 0.1
        self.manual_resolution = 10.0

        self.button7['bg'] = self.buttonColor
        self.button6['bg'] = self.buttonColor
        self.button8['bg'] = self.blueColor
        pass

    def boton9(self):
        print ("Home")
        self.roll = self.homeroll
        self.pitch = self.homepitch
        self.yaw = self.homeyaw
        self.update_ori()

        self.update_values()
        pass

    def boton10(self):
        print ("Pitch-")
        self.pitch = self.pitch - self.ori_resolution
        self.update_ori()

        self.update_values()
        pass

    def boton11(self):
        print ("Pitch+")
        self.pitch = self.pitch + self.ori_resolution
        self.update_ori()

        self.update_values()
        pass

    def boton12(self):
        print ("Yaw+")
        self.yaw = self.yaw + self.ori_resolution
        self.update_ori()

        self.update_values()
        pass

    def boton13(self):
        print ("Yaw-")
        self.yaw = self.yaw - self.ori_resolution
        self.update_ori()

        self.update_values()
        pass

    def boton14(self):
        print ("Roll-")
        self.roll = self.roll - self.ori_resolution
        self.update_ori()

        self.update_values()
        pass

    def boton15(self):
        print ("Roll+")
        self.roll = self.roll + self.ori_resolution
        self.update_ori()

        self.update_values()
        pass

    def boton16(self):
        print ("Home")
        self.pos_x = self.homepos_x
        self.pos_y = self.homepos_y
        self.pos_z = self.homepos_z

        self.update_values()
        pass

    def boton17(self):
        print ("Z+")
        self.pos_z = self.pos_z + self.pos_resolution

        self.update_values()
        pass

    def boton18(self):
        print ("Z-")
        self.pos_z = self.pos_z - self.pos_resolution

        self.update_values()
        pass

    def boton19(self):
        print ("X-")
        self.pos_x = self.pos_x - self.pos_resolution

        self.update_values()
        pass

    def boton20(self):
        print ("X+")
        self.pos_x = self.pos_x + self.pos_resolution

        self.update_values()
        pass

    def boton21(self):
        print ("Y+")
        self.pos_y = self.pos_y + self.pos_resolution

        self.update_values()
        pass

    def boton22(self):
        print ("Y-")
        self.pos_y = self.pos_y - self.pos_resolution

        self.update_values()
        pass

    def boton23(self):
        print ("Rest")
        self.pos_x = self.restpos_x
        self.pos_y = self.restpos_y
        self.pos_z = self.restpos_z

        self.ori_w = self.restori_w
        self.ori_x = self.restori_x
        self.ori_y = self.restori_y
        self.ori_z = self.restori_z

        self.update_values()
        pass

    def boton24(self):
        print ("Publish pose")
        posemsg = Pose()
            
        posemsg.position.x = self.pos_x
        posemsg.position.y = self.pos_y
        posemsg.position.z = self.pos_z
        posemsg.orientation.w = self.ori_w
        posemsg.orientation.x = self.ori_x
        posemsg.orientation.y = self.ori_y
        posemsg.orientation.z = self.ori_z
            
        self.manual_pose.publish(posemsg)
        pass

    def boton25(self):
        print ("Travel")
        strmsg = String()
        strmsg.data = "travel"
        self.move_type.publish(strmsg)
        pass

    def boton26(self):
        print ("Line")
        strmsg = String()
        strmsg.data = "line"
        self.move_type.publish(strmsg)
        pass

    def boton27(self):
        print ("ARM EXECUTE")

        anglemsg = Float32MultiArray()

        anglemsg.data.append(self.motor1_angle)
        anglemsg.data.append(self.motor2_angle)
        anglemsg.data.append(self.motor3_angle)
        anglemsg.data.append(self.motor4_angle)
        anglemsg.data.append(self.motor5_angle)
        anglemsg.data.append(self.motor6_angle)

        anglemsg.data.append(self.gripper1_angle)
        anglemsg.data.append(self.gripper2_angle)

        self.manual_angles.publish(anglemsg)

        if(self.motor1_angle == 0.0 and self.motor2_angle == 0.0 and self.motor3_angle == 0.0 and self.motor4_angle == 0.0 and self.motor5_angle == 0.0 and self.motor6_angle == 0.0):

            boolmsg = Bool()
            boolmsg.data = True
            self.finalExecute.publish(boolmsg)

        else:

            self.motor1_angle = 0.0
            self.motor2_angle = 0.0
            self.motor3_angle = 0.0
            self.motor4_angle = 0.0
            self.motor5_angle = 0.0
            self.motor6_angle = 0.0

            self.update_values()

        pass

    def boton28(self):
        print ("V +")
        self.velocity += self.vel_resolution
        self.accel += self.vel_resolution
        self.update_values()
        pass

    def boton29(self):
        print ("V -")
        if((self.velocity - self.vel_resolution) > 0.0):

            self.velocity -= self.vel_resolution
            self.accel -= self.vel_resolution
            self.update_values()
        pass

    def boton30(self):
        print ("Update V")
        velmsg = Float32MultiArray()
        velmsg.data = [self.velocity, self.accel]
        self.velocityconfig.publish(velmsg)
        pass

    def boton31(self):
        print ("Next Calibration")

        strmsg = String()
        strmsg.data = "line"
        self.move_type.publish(strmsg)

        if(self.calibrate_counter == 0):

            self.pos_x = self.calibratePoint1[0]
            self.pos_y = self.calibratePoint1[1]
            self.pos_z = self.calibratePoint1[2]

        elif(self.calibrate_counter == 1):

            self.pos_x = self.calibratePoint2[0]
            self.pos_y = self.calibratePoint2[1]
            self.pos_z = self.calibratePoint2[2]

        elif(self.calibrate_counter == 2):

            self.pos_x = self.calibratePoint3[0]
            self.pos_y = self.calibratePoint3[1]
            self.pos_z = self.calibratePoint3[2]

        elif(self.calibrate_counter == 3):

            self.pos_x = self.homepos_x
            self.pos_y = self.homepos_y - 0.05
            self.pos_z = self.homepos_z

            self.define_plane(self.calibratePoint1, self.calibratePoint2, self.calibratePoint3)

            paramsg = Float32MultiArray()

            paramsg.data = [self.A_plane, self.B_plane, self.C_plane, self.D_plane]

            self.plane_param.publish(paramsg)

            self.calibrate_counter = 0
            self.button1['state'] = 'normal'


        posemsg = Pose()
            
        posemsg.position.x = self.pos_x
        posemsg.position.y = self.pos_y
        posemsg.position.z = self.pos_z
        posemsg.orientation.w = self.ori_w
        posemsg.orientation.x = self.ori_x
        posemsg.orientation.y = self.ori_y
        posemsg.orientation.z = self.ori_z
            
        self.manual_pose.publish(posemsg)

        self.update_values()

        pass

    def boton32(self):
        print ("Save point")
        
        if(self.calibrate_counter == 0):

            self.calibratePoint1[1] = self.pos_y

        elif(self.calibrate_counter == 1):

            self.calibratePoint2[1] = self.pos_y

        elif(self.calibrate_counter == 2):

            self.calibratePoint3[1] = self.pos_y

        self.calibrate_counter += 1

        pass

    def boton33(self):
        print ("M1+")
        self.motor1_angle += self.manual_resolution

        self.update_values()
        pass

    def boton34(self):
        print ("M1-")
        self.motor1_angle -= self.manual_resolution

        self.update_values()
        pass

    def boton35(self):
        print ("M2+")
        self.motor2_angle += self.manual_resolution

        self.update_values()
        pass

    def boton36(self):
        print ("M2-")
        self.motor2_angle -= self.manual_resolution

        self.update_values()
        pass

    def boton37(self):
        print ("M3+")
        self.motor3_angle += self.manual_resolution

        self.update_values()
        pass

    def boton38(self):
        print ("M3-")
        self.motor3_angle -= self.manual_resolution

        self.update_values()
        pass

    def boton39(self):
        print ("M4+")
        self.motor4_angle += self.manual_resolution

        self.update_values()
        pass

    def boton40(self):
        print ("M4-")
        self.motor4_angle -= self.manual_resolution

        self.update_values()
        pass

    def boton41(self):
        print ("M5+")
        self.motor5_angle += self.manual_resolution

        self.update_values()
        pass

    def boton42(self):
        print ("M5-")
        self.motor5_angle -= self.manual_resolution

        self.update_values()
        pass

    def boton43(self):
        print ("M6+")
        self.motor6_angle += self.manual_resolution

        self.update_values()
        pass

    def boton44(self):
        print ("M6-")
        self.motor6_angle -= self.manual_resolution

        self.update_values()
        pass

    def boton45(self):
        print ("G1+")
        self.gripper1_angle += self.manual_resolution

        self.update_values()
        pass

    def boton46(self):
        print ("G1-")
        self.gripper1_angle -= self.manual_resolution

        self.update_values()
        pass

    def boton47(self):
        print ("G2+")
        self.gripper2_angle += self.manual_resolution

        self.update_values()
        pass

    def boton48(self):
        print ("G2-")
        self.gripper2_angle -= self.manual_resolution

        self.update_values()
        pass

    def boton_debug(self):
        print ("Debug")

        x0 = 50
        y0 = 50
        x1 = 200
        y1 = 300

        self.draw_point(x0, y0)
        self.draw_point(x1, y1)

        arc = self.canvas.create_arc(x0, y0, x1, y1, start=00, extent=120, style=tk.ARC, width=3)

        pass

    def ros_spin(self):

        # Create ROS 2 executor
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)

        # Spin the ROS 2 executor
        executor.spin()

    def update_values(self):

        self.xpostext = 'X: '+str(round(self.pos_x, 3)) + 'm'
        self.ypostext = 'Y: '+str(round(self.pos_y, 3)) + 'm'
        self.zpostext = 'Z: '+str(round(self.pos_z, 3)) + 'm'
        self.rolltext = 'R: '+str(round(self.roll, 3)) + '°'
        self.pitchtext = 'P: '+str(round(self.pitch, 3)) + '°'
        self.yawtext = 'Y: '+str(round(self.yaw, 3)) + '°'

        self.veltext = 'Velocity: '+str(round(self.velocity, 5))

        self.motor1_text = 'M1: ' + str(round(self.motor1_angle, 3)) + '°'
        self.motor2_text = 'M2: ' + str(round(self.motor2_angle, 3)) + '°'
        self.motor3_text = 'M3: ' + str(round(self.motor3_angle, 3)) + '°'
        self.motor4_text = 'M4: ' + str(round(self.motor4_angle, 3)) + '°'
        self.motor5_text = 'M5: ' + str(round(self.motor5_angle, 3)) + '°'
        self.motor6_text = 'M6: ' + str(round(self.motor6_angle, 3)) + '°'

        self.gripper1_text = 'G1: ' + str(round(self.gripper1_angle, 3)) + '°'
        self.gripper2_text = 'G2: ' + str(round(self.gripper2_angle, 3)) + '°'

        self.xvar.set(self.xpostext)
        self.yvar.set(self.ypostext)
        self.zvar.set(self.zpostext)
        self.rollvar.set(self.rolltext)
        self.pitchvar.set(self.pitchtext)
        self.yawvar.set(self.yawtext)

        self.velvar.set(self.veltext)

        self.m1var.set(self.motor1_text)
        self.m2var.set(self.motor2_text)
        self.m3var.set(self.motor3_text)
        self.m4var.set(self.motor4_text)
        self.m5var.set(self.motor5_text)
        self.m6var.set(self.motor6_text)

        self.g1var.set(self.gripper1_text)
        self.g2var.set(self.gripper2_text)


        self.xpos_label.config(text=self.xvar.get())
        self.ypos_label.config(text=self.yvar.get())
        self.zpos_label.config(text=self.zvar.get())
        self.roll_label.config(text=self.rollvar.get())
        self.pitch_label.config(text=self.pitchvar.get())
        self.yaw_label.config(text=self.yawvar.get())

        self.velocity_label.config(text=self.velvar.get())

        self.m1_label.config(text=self.m1var.get())
        self.m2_label.config(text=self.m2var.get())
        self.m3_label.config(text=self.m3var.get())
        self.m4_label.config(text=self.m4var.get())
        self.m5_label.config(text=self.m5var.get())
        self.m6_label.config(text=self.m6var.get())

        self.g1_label.config(text=self.g1var.get())
        self.g2_label.config(text=self.g2var.get())



    def __init__(self):
        super().__init__('gui')

        self.backgndColor = "#151716"
        self.buttonColor = "#cfc213"
        self.butTextColor = "black"

        self.redColor = "#e36868"
        self.greenColor = "#7de368"
        self.blueColor = "#6873e3"

        self.restpos_x = -0.017998
        self.restpos_y = -0.0765
        self.restpos_z = 0.26261

        self.restori_x = 0.28761
        self.restori_y = 0.287609
        self.restori_z = -0.645965
        self.restori_w = 0.64598


        self.homepos_x = 0.25
        self.homepos_y = -0.35
        self.homepos_z = 0.5

        self.homeroll = 0.0
        self.homepitch = 0.0
        self.homeyaw = -90.0


        self.pos_x = 0.25
        self.pos_y = -0.35
        self.pos_z = 0.5

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = -90.0

        quat = get_quaternion_from_euler(self.roll, self.pitch, self.yaw)

        self.ori_w = quat[0]
        self.ori_x = quat[1]
        self.ori_y = quat[2]
        self.ori_z = quat[3]

        self.pos_resolution = 10.0/1000
        self.ori_resolution = 10.0
        self.vel_resolution = 0.01
        self.manual_resolution = 1.0

        self.cursorX = 0.0
        self.cursorY = 0.0

        self.drawing = False

        self.desired_position = []
        self.executeDone = False

        self.autoMode = False
        self.planFlag = False

        self.gcodetext = ' '

        self.velocity = 0.1
        self.accel = 0.1

        self.calibrate_counter = 0

        self.calibratePoint1 = [self.homepos_x, self.homepos_y - 0.05, self.homepos_z]
        self.calibratePoint2 = [self.homepos_x - 0.5, self.homepos_y - 0.05, self.homepos_z]
        self.calibratePoint3 = [self.homepos_x - 0.5, self.homepos_y - 0.05, self.homepos_z - 0.5]

        self.A_plane = 0.0
        self.B_plane = 0.0
        self.C_plane = 0.0
        self.D_plane = 0.0

        self.xpostext = 'X: ' + str(round(self.pos_x, 3)) + 'm'
        self.ypostext = 'Y: ' + str(round(self.pos_y, 3)) + 'm'
        self.zpostext = 'Z: ' + str(round(self.pos_z, 3)) + 'm'
        self.rolltext = 'R: ' + str(round(self.roll, 3)) + '°'
        self.pitchtext = 'P: ' + str(round(self.pitch, 3)) + '°'
        self.yawtext = 'Y: ' + str(round(self.yaw, 3)) + '°'

        self.motor1_angle = 0.0
        self.motor2_angle = 0.0
        self.motor3_angle = 0.0
        self.motor4_angle = 0.0
        self.motor5_angle = 0.0
        self.motor6_angle = 0.0

        self.gripper1_angle = 90.0
        self.gripper2_angle = 90.0

        self.motor1_text = 'M1: ' + str(round(self.motor1_angle, 3)) + '°'
        self.motor2_text = 'M2: ' + str(round(self.motor2_angle, 3)) + '°'
        self.motor3_text = 'M3: ' + str(round(self.motor3_angle, 3)) + '°'
        self.motor4_text = 'M4: ' + str(round(self.motor4_angle, 3)) + '°'
        self.motor5_text = 'M5: ' + str(round(self.motor5_angle, 3)) + '°'
        self.motor6_text = 'M6: ' + str(round(self.motor6_angle, 3)) + '°'

        self.gripper1_text = 'G1: ' + str(round(self.gripper1_angle, 3)) + '°'
        self.gripper2_text = 'G2: ' + str(round(self.gripper2_angle, 3)) + '°'

        self.veltext = 'Velocity: '+str(round(self.velocity, 5))

        self.gcodestring = self.create_subscription(String, 'robocol/arm/line_data' ,self.gcodestring_callback, 10)

        self.planning_scene = self.create_subscription(PlanningScene, '/monitored_planning_scene', self.planning_scene_callback, 10)

        self.planned_path = self.create_subscription(DisplayTrajectory, '/display_planned_path', self.display_planned_path_callback, 10)

        self.planning_info = self.create_subscription(Bool, '/robocol/arm/planning_info', self.planning_info_callback, 10)

        self.nextgcode = self.create_publisher(Bool, 'robocol/arm/next_gcode', 10)
        self.nexttraj = self.create_publisher(Bool, 'robocol/arm/next_position', 10)
        self.plan_exec = self.create_publisher(String, 'robocol/arm/plan_exec', 10)
        self.manual_pose = self.create_publisher(Pose, 'robocol/arm/pose_data', 10)
        self.move_type = self.create_publisher(String, 'robocol/arm/movetype', 10)
        self.filename = self.create_publisher(String, 'robocol/arm/filename', 10)
        self.finalExecute = self.create_publisher(Bool, 'robocol/arm/final_Execute', 10)
        self.velocityconfig = self.create_publisher(Float32MultiArray, 'robocol/arm/velocity_accel', 10)
        self.plane_param = self.create_publisher(Float32MultiArray, 'robocol/arm/plane_param', 10)
        self.manual_angles = self.create_publisher(Float32MultiArray, 'robocol/arm/manual_angles', 10)

        #self.mapa_base =  255*np.ones((500,500),dtype=np.uint8)
        self.root = tk.Tk()
        
        # Adjust size 
        self.root.geometry("800x700")
        self.root.title("Gcode Interface")
        self.root.configure(bg=self.backgndColor)

        self.var = tk.StringVar()
        self.var.set("GCODE ACTUAL")

        self.velvar = tk.StringVar()
        self.velvar.set(self.veltext)
        #bg = tk.PhotoImage(file = "mapa_final.png")
        
        # Show image using label
        #label1 = Label( root, image = bg)
        #label1.place(x = 0, y = 0)

        self.button1 = tk.Button(
            self.root,
            text="Seleccionar Gcode",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=15,
            height=1,
            command=self.boton1,
            cursor="hand2",
        )
        self.button1.place(x=60, y=10)

        self.button2 = tk.Button(
            self.root,
            text="Auto",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=8,
            height=2,
            command=self.boton2,
            cursor="hand2",
        )
        self.button2.place(x=30, y=100)

        self.button3 = tk.Button(
            self.root,
            text="Plan/Exec",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=8,
            height=2,
            command=self.boton3,
            cursor="hand2",
        )
        self.button3.place(x=170, y=100)

        self.gcode_label = tk.Label(self.root, text=self.var.get(), font=("Arial", 12), fg="white", bg=self.backgndColor)
        self.gcode_label.place(x=30, y=160)

        self.button4 = tk.Button(
            self.root,
            text="Plan",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton4,
            cursor="hand2",
        )
        self.button4.place(x=110, y=200)

        self.button5 = tk.Button(
            self.root,
            text="Execute",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton5,
            cursor="hand2",
        )
        self.button5.place(x=200, y=200)

        self.button5a = tk.Button(
            self.root,
            text="Next Line",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton5a,
            cursor="hand2",
        )
        self.button5a.place(x=20, y=200)

        text_label = tk.Label(self.root, text='Manual Pose', font=("Arial", 16, "bold"), fg="white", bg=self.backgndColor)
        text_label.place(x=30, y=270)

        self.xvar = tk.StringVar()
        self.xvar.set(self.xpostext)
        self.xpos_label = tk.Label(self.root, text=self.xvar.get(), font=("Arial", 13, "bold"), fg=self.redColor, bg=self.backgndColor)
        self.xpos_label.place(x=20, y=300)

        self.yvar = tk.StringVar()
        self.yvar.set(self.ypostext)
        self.ypos_label = tk.Label(self.root, text=self.yvar.get(), font=("Arial", 13, "bold"), fg=self.greenColor, bg=self.backgndColor)
        self.ypos_label.place(x=20, y=330)

        self.zvar = tk.StringVar()
        self.zvar.set(self.zpostext)
        self.zpos_label = tk.Label(self.root, text=self.zvar.get(), font=("Arial", 13, "bold"), fg=self.blueColor, bg=self.backgndColor)
        self.zpos_label.place(x=20, y=360)

        self.rollvar = tk.StringVar()
        self.rollvar.set(self.rolltext)
        self.roll_label = tk.Label(self.root, text=self.rollvar.get(), font=("Arial", 13, "bold"), fg=self.redColor, bg=self.backgndColor)
        self.roll_label.place(x=120, y=300)

        self.pitchvar = tk.StringVar()
        self.pitchvar.set(self.pitchtext)
        self.pitch_label = tk.Label(self.root, text=self.pitchvar.get(), font=("Arial", 13, "bold"), fg=self.greenColor, bg=self.backgndColor)
        self.pitch_label.place(x=120, y=330)

        self.yawvar = tk.StringVar()
        self.yawvar.set(self.yawtext)
        self.yaw_label = tk.Label(self.root, text=self.yawvar.get(), font=("Arial", 13, "bold"), fg=self.blueColor, bg=self.backgndColor)
        self.yaw_label.place(x=120, y=360)

        res_label = tk.Label(self.root, text='Resolution', font=("Arial", 13, "bold"), fg="white", bg=self.backgndColor)
        res_label.place(x=100, y=600)

        self.button6 = tk.Button(
            self.root,
            text="1mm/1°",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton6,
            cursor="hand2",
        )
        self.button6.place(x=10, y=630)

        self.button7 = tk.Button(
            self.root,
            text="10mm/10°",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.blueColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton7,
            cursor="hand2",
        )
        self.button7.place(x=110, y=630)

        self.button8 = tk.Button(
            self.root,
            text="100mm/45°",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton8,
            cursor="hand2",
        )
        self.button8.place(x=210, y=630)

        #ori_label = tk.Label(self.root, text='Orientation', font=("Arial", 14), fg="white", bg=self.backgndColor)
        #ori_label.place(x=350, y=500)

        ori_place_frame = tk.Frame(self.root)
        ori_place_frame.configure(bg=self.backgndColor)
        ori_place_frame.place(x=370, y=510)

        self.button9 = tk.Button(
            ori_place_frame,
            text="Home",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton9,
            cursor="hand2",
        )
        self.button9.grid(row=2, column=2, padx=5, pady=5)

        self.button10 = tk.Button(
            ori_place_frame,
            text="Pitch-",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.greenColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton10,
            cursor="hand2",
        )
        self.button10.grid(row=1, column=2, pady=5)

        self.button11 = tk.Button(
            ori_place_frame,
            text="Pitch+",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.greenColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton11,
            cursor="hand2",
        )
        self.button11.grid(row=3, column=2, pady=5)

        self.button12 = tk.Button(
            ori_place_frame,
            text="Yaw+",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.blueColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton12,
            cursor="hand2",
        )
        self.button12.grid(row=2, column=1, padx=5)

        self.button13 = tk.Button(
            ori_place_frame,
            text="Yaw-",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.blueColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton13,
            cursor="hand2",
        )
        self.button13.grid(row=2, column=3, padx=5)

        self.button14 = tk.Button(
            ori_place_frame,
            text="Roll+",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.redColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton14,
            cursor="hand2",
        )
        self.button14.grid(row=1, column=3, padx=5)

        self.button15 = tk.Button(
            ori_place_frame,
            text="Roll-",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.redColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton15,
            cursor="hand2",
        )
        self.button15.grid(row=3, column=3, padx=5)

        #pos_label = tk.Label(self.root, text='Position', font=("Arial", 14), fg="white", bg=self.backgndColor)
        #pos_label.place(x=600, y=500)

        pos_place_frame = tk.Frame(self.root)
        pos_place_frame.configure(bg=self.backgndColor)
        pos_place_frame.place(x=600, y=510)

        self.button16 = tk.Button(
            pos_place_frame,
            text="Home",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton16,
            cursor="hand2",
        )
        self.button16.grid(row=2, column=2, padx=5, pady=5)

        self.button17 = tk.Button(
            pos_place_frame,
            text="Z+",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.blueColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton17,
            cursor="hand2",
        )
        self.button17.grid(row=1, column=2, pady=5)

        self.button18 = tk.Button(
            pos_place_frame,
            text="Z-",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.blueColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton18,
            cursor="hand2",
        )
        self.button18.grid(row=3, column=2, pady=5)

        self.button19 = tk.Button(
            pos_place_frame,
            text="X-",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.redColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton19,
            cursor="hand2",
        )
        self.button19.grid(row=2, column=1, padx=5)

        self.button20 = tk.Button(
            pos_place_frame,
            text="X+",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.redColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton20,
            cursor="hand2",
        )
        self.button20.grid(row=2, column=3, padx=5)

        self.button21 = tk.Button(
            pos_place_frame,
            text="Y+",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.greenColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton21,
            cursor="hand2",
        )
        self.button21.grid(row=1, column=1, padx=5)

        self.button22 = tk.Button(
            pos_place_frame,
            text="Y-",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.greenColor,
            borderwidth=2,
            relief="raised",
            width=3,
            height=2,
            command=self.boton22,
            cursor="hand2",
        )
        self.button22.grid(row=3, column=1, padx=5)

        self.button23 = tk.Button(
            self.root,
            text="Rest",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=8,
            height=2,
            command=self.boton23,
            cursor="hand2",
        )
        self.button23.place(x=30, y=400)

        self.button24 = tk.Button(
            self.root,
            text="Publish Pose",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=8,
            height=2,
            command=self.boton24,
            cursor="hand2",
        )
        self.button24.place(x=160, y=400)

        self.button25 = tk.Button(
            self.root,
            text="Travel",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=8,
            height=2,
            command=self.boton25,
            cursor="hand2",
        )
        self.button25.place(x=30, y=470)

        self.button26 = tk.Button(
            self.root,
            text="Line",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=8,
            height=2,
            command=self.boton26,
            cursor="hand2",
        )
        self.button26.place(x=160, y=470)

        self.button27 = tk.Button(
            self.root,
            text="ARM EXECUTE",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg="white",
            borderwidth=2,
            relief="raised",
            width=12,
            height=2,
            command=self.boton27,
            cursor="hand2",
        )
        self.button27.place(x=20, y=540)

        self.button28 = tk.Button(
            self.root,
            text="V +",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=2,
            height=1,
            command=self.boton28,
            cursor="hand2",
        )
        self.button28.place(x=190, y=530)

        self.button29 = tk.Button(
            self.root,
            text="V -",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=2,
            height=1,
            command=self.boton29,
            cursor="hand2",
        )
        self.button29.place(x=190, y=560)

        self.velocity_label = tk.Label(self.root, text=self.velvar.get(), font=("Arial", 12), fg="white", bg=self.backgndColor)
        self.velocity_label.place(x=270, y=530)

        self.button30 = tk.Button(
            self.root,
            text="Update V",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton30,
            cursor="hand2",
        )
        self.button30.place(x=270, y=570)

        self.button31 = tk.Button(
            self.root,
            text="Next Corner",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton31,
            cursor="hand2",
        )
        self.button31.place(x=200, y=270)

        self.button32 = tk.Button(
            self.root,
            text="Save Point",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton32,
            cursor="hand2",
        )
        self.button32.place(x=200, y=330)

        manual_place_frame = tk.Frame(self.root)
        manual_place_frame.configure(bg=self.backgndColor)
        manual_place_frame.place(x=300, y=400)

        self.button33 = tk.Button(
            manual_place_frame,
            text="^",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton33,
            cursor="hand2",
        )
        self.button33.grid(row=1, column=1, padx=0, pady=5)

        self.m1var = tk.StringVar()
        self.m1var.set(self.motor1_text)
        self.m1_label = tk.Label(manual_place_frame, text=self.m1var.get(), font=("Arial", 10, "bold"), fg="white", bg=self.backgndColor)
        self.m1_label.grid(row=2, column=1, padx=0, pady=0)

        self.button34 = tk.Button(
            manual_place_frame,
            text="v",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton34,
            cursor="hand2",
        )
        self.button34.grid(row=3, column=1, padx=0, pady=0)

        self.button35 = tk.Button(
            manual_place_frame,
            text="^",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton35,
            cursor="hand2",
        )
        self.button35.grid(row=1, column=2, padx=0, pady=5)

        self.m2var = tk.StringVar()
        self.m2var.set(self.motor2_text)
        self.m2_label = tk.Label(manual_place_frame, text=self.m2var.get(), font=("Arial", 10, "bold"), fg="white", bg=self.backgndColor)
        self.m2_label.grid(row=2, column=2, padx=0, pady=0)

        self.button36 = tk.Button(
            manual_place_frame,
            text="v",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton36,
            cursor="hand2",
        )
        self.button36.grid(row=3, column=2, padx=0, pady=0)

        self.button37 = tk.Button(
            manual_place_frame,
            text="^",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton37,
            cursor="hand2",
        )
        self.button37.grid(row=1, column=3, padx=0, pady=5)

        self.m3var = tk.StringVar()
        self.m3var.set(self.motor3_text)
        self.m3_label = tk.Label(manual_place_frame, text=self.m3var.get(), font=("Arial", 10, "bold"), fg="white", bg=self.backgndColor)
        self.m3_label.grid(row=2, column=3, padx=0, pady=0)

        self.button38 = tk.Button(
            manual_place_frame,
            text="v",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton38,
            cursor="hand2",
        )
        self.button38.grid(row=3, column=3, padx=0, pady=0)

        self.button39 = tk.Button(
            manual_place_frame,
            text="^",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton39,
            cursor="hand2",
        )
        self.button39.grid(row=1, column=4, padx=0, pady=5)

        self.m4var = tk.StringVar()
        self.m4var.set(self.motor4_text)
        self.m4_label = tk.Label(manual_place_frame, text=self.m4var.get(), font=("Arial", 10, "bold"), fg="white", bg=self.backgndColor)
        self.m4_label.grid(row=2, column=4, padx=0, pady=0)

        self.button40 = tk.Button(
            manual_place_frame,
            text="v",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton40,
            cursor="hand2",
        )
        self.button40.grid(row=3, column=4, padx=0, pady=0)

        self.button41 = tk.Button(
            manual_place_frame,
            text="^",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton41,
            cursor="hand2",
        )
        self.button41.grid(row=1, column=5, padx=0, pady=5)

        self.m5var = tk.StringVar()
        self.m5var.set(self.motor5_text)
        self.m5_label = tk.Label(manual_place_frame, text=self.m5var.get(), font=("Arial", 10, "bold"), fg="white", bg=self.backgndColor)
        self.m5_label.grid(row=2, column=5, padx=0, pady=0)

        self.button42 = tk.Button(
            manual_place_frame,
            text="v",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton42,
            cursor="hand2",
        )
        self.button42.grid(row=3, column=5, padx=0, pady=0)

        self.button43 = tk.Button(
            manual_place_frame,
            text="^",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton43,
            cursor="hand2",
        )
        self.button43.grid(row=1, column=6, padx=0, pady=5)

        self.m6var = tk.StringVar()
        self.m6var.set(self.motor6_text)
        self.m6_label = tk.Label(manual_place_frame, text=self.m6var.get(), font=("Arial", 10, "bold"), fg="white", bg=self.backgndColor)
        self.m6_label.grid(row=2, column=6, padx=0, pady=0)

        self.button44 = tk.Button(
            manual_place_frame,
            text="v",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton44,
            cursor="hand2",
        )
        self.button44.grid(row=3, column=6, padx=0, pady=0)

        self.button45 = tk.Button(
            manual_place_frame,
            text="^",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton45,
            cursor="hand2",
        )
        self.button45.grid(row=1, column=7, padx=0, pady=5)

        self.g1var = tk.StringVar()
        self.g1var.set(self.gripper1_text)
        self.g1_label = tk.Label(manual_place_frame, text=self.g1var.get(), font=("Arial", 10, "bold"), fg="white", bg=self.backgndColor)
        self.g1_label.grid(row=2, column=7, padx=0, pady=0)

        self.button46 = tk.Button(
            manual_place_frame,
            text="v",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton46,
            cursor="hand2",
        )
        self.button46.grid(row=3, column=7, padx=0, pady=0)

        self.button47 = tk.Button(
            manual_place_frame,
            text="^",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton47,
            cursor="hand2",
        )
        self.button47.grid(row=1, column=8, padx=0, pady=5)

        self.g2var = tk.StringVar()
        self.g2var.set(self.gripper2_text)
        self.g2_label = tk.Label(manual_place_frame, text=self.g2var.get(), font=("Arial", 10, "bold"), fg="white", bg=self.backgndColor)
        self.g2_label.grid(row=2, column=8, padx=0, pady=0)

        self.button48 = tk.Button(
            manual_place_frame,
            text="v",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=1,
            height=1,
            command=self.boton48,
            cursor="hand2",
        )
        self.button48.grid(row=3, column=8, padx=0, pady=0)



        button_debug = tk.Button(
            self.root,
            text="Debug",
            font=("Arial", 12),
            fg=self.butTextColor,
            bg=self.buttonColor,
            borderwidth=2,
            relief="raised",
            width=6,
            height=2,
            command=self.boton_debug,
            cursor="hand2",
        )
        #button_debug.place(x=150, y=540)


        ros_thread = threading.Thread(target=self.ros_spin)
        ros_thread.start()


        self.canvas = tk.Canvas(self.root, width=500, height=400, bg="white")
        self.canvas.place(x=300, y=0)

        #x1, y1, x2, y2 = 50, 50, 150, 100
        #fill_color = "blue"
        #self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill_color)

        #print ("minimal_subscriber_publisher")

        self.button1['state'] = 'disabled'

        self.root.mainloop()

    def gcodestring_callback(self, msg):

        print(msg.data)

        self.var.set(msg.data)

        self.gcode_label.config(text=self.var.get())

        self.draw_gcode(msg.data)

    def planning_scene_callback(self, msg):
        #print("planning_scene")
        #print(msg.data)

        #self.desired_position = [1.5, 0.8, -0.3]
        #print(msg.robot_state.joint_state.position)

        if not self.planFlag:

            if msg.robot_state.is_diff:
                # Check for changes in joint_state indicating execution completion
                if msg.robot_state.joint_state.position == self.desired_position:
                    print('MoveIt! execution has completed.')
                    self.executeDone = True

                    self.button3['state'] = 'normal'
                    self.button4['state'] = 'normal'
                    #self.button5['state'] = 'normal'
                    self.button5a['state'] = 'normal'

                    self.button4['bg'] = self.buttonColor
                    self.button5['bg'] = self.buttonColor
                    self.button5a['bg'] = self.buttonColor

    def display_planned_path_callback(self, msg):
        # Extract joint states from the received message
        self.desired_position = msg.trajectory[-1].joint_trajectory.points[-1].positions  # Assumes the last point in the path has joint positions

        # Process the last joint states as needed
        #print(f"Last joint states: {self.desired_position}")

    def planning_info_callback(self, msg):
        
        if msg.data:

            self.button4['bg'] = self.greenColor
            self.button5['bg'] = self.greenColor
            self.button5['state'] = 'normal'

        else:

            self.button4['bg'] = self.redColor
            self.button5['bg'] = self.redColor
            self.button5['state'] = 'disabled'



##############################################################################################################################
    
def main(args=None):


    rclpy.init(args=args)
    gcode_interface = Gcode_interface()

    #rclpy.spin(gcode_interface)
    
    gcode_interface.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()