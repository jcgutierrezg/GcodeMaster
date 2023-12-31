import re
import time
import rclpy
import os
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

import numpy as np # Scientific computing library for Python
import math


#10mm_square2.gcode
#MHammerRz_0004.gcode

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


class Gcode_reader(Node):

    def __init__(self):
        super().__init__('gcode_reader')

        self.positioning = 'abs'
        self.positioningCount = 0
        self.units = 'mm'
        self.unitsCount = 0

        self.xoffset = 0.5/2
        self.yoffset = -0.35
        self.zoffset = 0.5

        self.prevX = self.xoffset
        self.prevY = self.yoffset
        self.prevZ = self.zoffset

        self.moveFlag = False

        #filename = os.path.expanduser('~/Tesis_IMEC/gcodes/10mm_JUANaw.gcode')

        self.pose_data = self.create_publisher(Pose,'robocol/arm/pose_data',10)
        self.string_data = self.create_publisher(String,'robocol/arm/movetype',10)
        self.line_data = self.create_publisher(String,'robocol/arm/line_data',10)

        self.nextline = self.create_subscription(
            Bool,
            'robocol/arm/next_gcode',
            self.listener_callback,
            10)

        self.filestr = self.create_subscription(
            String,
            'robocol/arm/filename',
            self.filestr_callback,
            10)

        self.nextline  # prevent unused variable warning
        self.filestr  # prevent unused variable warning

        self.get_logger().info('Waiting for gcode file selection...')

        

    def filestr_callback(self, msg):

        filename = os.path.expanduser(msg.data)

        # Input archive with multiple lines
        with open(filename, 'r') as file:
            archive = file.read()

        # Regular expression pattern to match the values
        pattern = r'([A-Z])(-?\d+(\.\d+)?)?'

        # Initialize a list to store dictionaries
        self.result_list = []

        # Iterate through lines in the archive
        for line in archive.strip().split('\n'):
            # Find all matches in the line
            matches = re.findall(pattern, line)

            # Create a dictionary to store the values
            result = {}

            # Iterate through matches and populate the dictionary
            for match in matches:
                key = match[0]
                value = match[1] if match[1] else None
                result[key] = value

            # Append the dictionary to the result list
            self.result_list.append(result)

        self.gcode_length = len(self.result_list)
        self.line_counter = 0

        Roll = 0
        Pitch = 0
        Yaw = -90

        quat = get_quaternion_from_euler(Roll, Pitch, Yaw)

        strmsg = String()
        strmsg.data = "travel"
        self.string_data.publish(strmsg)
        self.get_logger().info('Published string: %s' % strmsg.data)

        posemsg = Pose()

        posemsg.position.x = self.xoffset
        posemsg.position.y = self.yoffset
        posemsg.position.z = self.zoffset
        posemsg.orientation.w = quat[0]
        posemsg.orientation.x = quat[1]
        posemsg.orientation.y = quat[2]
        posemsg.orientation.z = quat[3]
            
        self.pose_data.publish(posemsg)

        self.get_logger().info('Published Pose: %s, %s, %s' % (posemsg.position.x, posemsg.position.y, posemsg.position.z))


    def listener_callback(self, msg):

        self.moveFlag = False

        if(self.line_counter< self.gcode_length):

            while(not self.moveFlag):


                print("Executing command: ")

                result = self.result_list[self.line_counter]

                # Using a list comprehension to create a list of formatted strings
                formatted_data = [f"{key}{value}" for key, value in result.items()]

                # Join the list of formatted strings into a single string
                stringresult = ' '.join(formatted_data)

                #print(mystring)

                linemsg = String()

                linemsg.data = stringresult

                self.line_data.publish(linemsg)

                self.get_logger().info(stringresult)

                if(result.get('G') == '4'):

                    dwellTime = int(result.get('P'))/1000
                    time.sleep(dwellTime)
                    dwellTime = 0.0
                    self.moveFlag = False

                elif(result.get('G') == '21' and self.unitsCount == 0):
                    self.units = 'mm'
                    self.unitsCount = self.unitsCount+1
                    self.moveFlag = False

                elif(result.get('G') == '20' and self.unitsCount == 0):
                    self.units = 'inch'
                    self.unitsCount = self.unitsCount+1
                    self.moveFlag = False

                elif((result.get('G') == '21' or result.get('G') == '20') and self.positioningCount > 0):
                    print("Units inconsistent, redo gcode file.")
                    self.moveFlag = False


                elif(result.get('G') == '90' and self.positioningCount == 0):
                    self.positioning = 'abs'
                    self.positioningCount = self.positioningCount+1
                    self.moveFlag = False

                elif(result.get('G') == '91' and self.positioningCount == 0):
                    self.positioning = 'rel'
                    self.positioningCount = self.positioningCount+1
                    self.moveFlag = False

                elif((result.get('G') == '90' or result.get('G') == '91') and self.positioningCount > 0):
                    print("Positioning inconsistent, redo gcode file.")
                    self.moveFlag = False


                elif(result.get('G') == '1'):

                    if(result.get('F') != None):

                        speed = float(result.get('F'))/1000.0/60

                    if(result.get('X') != None):
                        PosX = self.xoffset-float(result.get('X'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True
                    
                    if(result.get('Y') != None):
                        PosZ = self.zoffset+float(result.get('Y'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True

                    moveType = 'line'

                elif(result.get('G') == '2'): #CW

                    if(result.get('F') != None):

                        speed = float(result.get('F'))/1000.0/60

                    if(result.get('X') != None):
                        PosX = self.xoffset-float(result.get('X'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True
                    
                    if(result.get('Y') != None):
                        PosZ = self.zoffset+float(result.get('Y'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True

                    if(result.get('I') != None):
                        centerX = self.prevX-float(result.get('I'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True
                    
                    if(result.get('J') != None):
                        centerZ = self.prevZ+float(result.get('J'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True

                    moveType = 'arc'

                elif(result.get('G') == '3'): #CCW

                    if(result.get('F') != None):

                        speed = float(result.get('F'))/1000.0/60

                    if(result.get('X') != None):
                        PosX = self.xoffset-float(result.get('X'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True
                    
                    if(result.get('Y') != None):
                        PosZ = self.zoffset+float(result.get('Y'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True

                    if(result.get('I') != None):
                        centerX = self.prevX-float(result.get('I'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True
                    
                    if(result.get('J') != None):
                        centerZ = self.prevZ+float(result.get('J'))/1000.0
                        yActual = self.prevY

                        self.moveFlag = True

                    moveType = 'arc'


                if(result.get('M') == '5'):

                    laser = 0.0
                    yActual = self.yoffset

                    PosX = self.prevX
                    PosZ = self.prevZ

                    laserPower = 0.0
                    self.moveFlag = True

                    moveType = 'line'

                elif(result.get('M') == '3'):

                    laser = 1.0

                    PosX = self.prevX
                    PosZ = self.prevZ

                    yActual = self.yoffset-0.05
                    laserPower = float(result.get('S'))
                    self.moveFlag = True

                    moveType = 'line'

                if(self.moveFlag):
                    #array_data = [X, Y, SPEED, ON/OFF, POWER]

                    Roll = 0
                    Pitch = 0
                    Yaw = -90

                    quat = get_quaternion_from_euler(Roll, Pitch, Yaw)

                    #self.array_data = [Xpos, Ypos, speed, laser, laserPower]

                    strmsg = String()
                    strmsg.data = moveType
                    self.string_data.publish(strmsg)
                    self.get_logger().info('Published string: %s' % strmsg.data)

                    posemsg = Pose()
            
                    posemsg.position.x = PosX
                    posemsg.position.y = yActual
                    posemsg.position.z = PosZ
                    posemsg.orientation.w = quat[0]
                    posemsg.orientation.x = quat[1]
                    posemsg.orientation.y = quat[2]
                    posemsg.orientation.z = quat[3]
                        
                    self.pose_data.publish(posemsg)

                    self.get_logger().info('Published Pose X: %s, %s, %s' % (posemsg.position.x, posemsg.position.y, posemsg.position.z))

                    self.prevX = PosX
                    self.prevY = yActual
                    self.prevZ = PosZ

                    if(moveType == 'arc'):

                        centermsg = Pose()
            
                        centermsg.position.x = centerX
                        centermsg.position.y = yActual
                        centermsg.position.z = centerZ
                        centermsg.orientation.w = quat[0]
                        centermsg.orientation.x = quat[1]
                        centermsg.orientation.y = quat[2]
                        centermsg.orientation.z = quat[3]
                            
                        self.pose_data.publish(centermsg)

                        self.get_logger().info('Published Pose X: %s, %s, %s' % (centermsg.position.x, centermsg.position.y, centermsg.position.z))

                    # prevOriX = Roll
                    # prevOriY = Pitch
                    # prevOriZ = Yaw

                    # prevPosX = PosX
                    # prevPosY = PosY
                    # prevPosZ = PosZ




                self.line_counter = self.line_counter+1


        else:

            print("Gcode execution complete. Shutting down node.")

            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    gcode_reader = Gcode_reader()

    rclpy.spin(gcode_reader)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gcode_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




