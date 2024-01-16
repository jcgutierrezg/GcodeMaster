import re
import time
import rclpy
import os
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

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


class Gcode_reader(Node):

    def __init__(self):
        super().__init__('gcode_reader')

        self.cylinderMode = False

        self.positioning = 'abs'
        self.positioningCount = 0
        self.units = 'mm'
        self.unitsCount = 0

        if(self.cylinderMode):

            self.xoffset = 0.05
            self.yoffset = -0.35
            self.zoffset = 0.5

        else:
            self.xoffset = 0.5/2
            self.yoffset = -0.35
            self.zoffset = 0.5

        self.cylinder_radius = 0.2
        #self.cylinderCoords = [self.xoffset-self.cylinder_radius, -0.5, self.zoffset-self.cylinder_radius]
        self.cylinderCoords = [0.0, -0.5, 0.0]

        self.prevX = self.xoffset
        self.prevY = self.yoffset
        self.prevZ = self.zoffset

        self.moveFlag = False

        self.startMoveFlag = False

        self.A_plane = 0.0
        self.B_plane = 1.0
        self.C_plane = 0.0
        self.D_plane = 0.0

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

        self.plane_param = self.create_subscription(
            Float32MultiArray,
            'robocol/arm/plane_param',
            self.calibrate_callback,
            10)

        self.nextline  # prevent unused variable warning
        self.filestr  # prevent unused variable warning

        self.get_logger().info('Waiting for gcode file selection...')

    def normalize_vector(self, vector):
        magnitude = np.linalg.norm(vector)
        return vector / magnitude if magnitude != 0 else vector

    def multiply_quaternions(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    def calculate_orientation(self, normalized_normal_vector):
        # Apply a -90° rotation around the Z-axis
        #z_offset_quaternion = np.array([np.cos(-np.pi / 4), 0, 0, np.sin(-np.pi / 4)])
        offset_quaternion = get_quaternion_from_euler(0.0, 0.0, -90.0)

        # Calculate the quaternion from the normalized normal vector
        angle = np.arccos(normalized_normal_vector[2])  # Assuming z is up
        axis = np.cross(np.array([0, 0, 1]), normalized_normal_vector)
        axis_normalized = self.normalize_vector(axis)
        
        # Quaternion components
        scalar_part = np.cos(angle / 2)
        vector_part = np.sin(angle / 2) * axis_normalized
        
        # Combine the offset and calculated quaternion
        #quaternion = self.multiply_quaternions(offset_quaternion, np.concatenate(([scalar_part], vector_part)))
        quaternion = self.multiply_quaternions(offset_quaternion, np.concatenate(([scalar_part], vector_part)))

        return quaternion

    def project_onto_plane(self, x, y, z):

        A = self.A_plane
        B = self.B_plane
        C = self.C_plane
        D = self.D_plane

        y_projected = (D - A*x - C*z) / B

        return y_projected

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

        if(self.line_counter < self.gcode_length):

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

                    if not self.cylinderMode:

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

                    else:

                        if not self.startMoveFlag:

                            if(result.get('F') != None):

                                speed = float(result.get('F'))/1000.0/60

                            if(result.get('X') != None and result.get('Y') != None):

                                PosX = self.xoffset-float(result.get('X'))/1000.0
                            
                                PosZ = self.zoffset+float(result.get('Y'))/1000.0
                                #yActual = self.prevY

                                yActual = math.sqrt((self.cylinder_radius)**2 - (PosX-self.cylinderCoords[0])**2) + self.cylinderCoords[1]

                                self.get_logger().info('X: %f' % PosX)
                                self.get_logger().info('Y: %f' % yActual)
                                self.get_logger().info('Z: %f' % PosZ)

                                self.moveFlag = True


                            moveType = 'travel'

                            self.startMoveFlag = True


                        else:


                            if(result.get('F') != None):

                                speed = float(result.get('F'))/1000.0/60

                            if(result.get('X') != None and result.get('Y') != None):

                                PosX = self.xoffset-float(result.get('X'))/1000.0
                            
                                PosZ = self.zoffset+float(result.get('Y'))/1000.0
                                #yActual = self.prevY

                                yActual = math.sqrt((self.cylinder_radius)**2 - (PosX-self.cylinderCoords[0])**2) + self.cylinderCoords[1]

                                self.get_logger().info('X: %f' % PosX)
                                self.get_logger().info('Y: %f' % yActual)
                                self.get_logger().info('Z: %f' % PosZ)

                                self.moveFlag = True


                            moveType = 'arc'

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


                if(result.get('M') == '5' and not self.cylinderMode):

                    laser = 0.0
                    yActual = self.yoffset

                    PosX = self.prevX
                    PosZ = self.prevZ

                    laserPower = 0.0
                    self.moveFlag = True

                    moveType = 'line'

                elif(result.get('M') == '3' and not self.cylinderMode):

                    laser = 1.0

                    PosX = self.prevX
                    PosZ = self.prevZ

                    yActual = self.yoffset-0.05
                    laserPower = float(result.get('S'))
                    self.moveFlag = True

                    moveType = 'line'

                if(self.moveFlag):
                    #array_data = [X, Y, SPEED, ON/OFF, POWER]

                    if (moveType == 'travel' or moveType == 'line' or (not self.cylinderMode and moveType == 'arc')):

                        Roll = 0
                        Pitch = 0
                        Yaw = -90

                        quat = get_quaternion_from_euler(Roll, Pitch, Yaw)

                        #self.array_data = [Xpos, Ypos, speed, laser, laserPower]

                    elif(self.cylinderMode and moveType == "arc"):

                        normal_vector = np.array([PosX - self.cylinderCoords[0], yActual - self.cylinderCoords[1], 0.0])

                        normalized_normal_vector = self.normalize_vector(normal_vector)

                        quat = self.calculate_orientation(normalized_normal_vector)

                        self.get_logger().info('Published Quat WXYZ: %s, %s, %s, %s' % (quat[0], quat[1], quat[2], quat[3]))

                    strmsg = String()
                    strmsg.data = moveType
                    self.string_data.publish(strmsg)
                    self.get_logger().info('Published string: %s' % strmsg.data)

                    if(not self.cylinderMode):

                        y_new = self.project_onto_plane(PosX, yActual, PosZ)

                    else:

                        y_new = yActual

                    posemsg = Pose()
            
                    posemsg.position.x = PosX
                    posemsg.position.y = y_new
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

                        if not self.cylinderMode:

                            centermsg = Pose()
                
                            centermsg.position.x = centerX
                            centermsg.position.y = yActual
                            centermsg.position.z = centerZ
                            centermsg.orientation.w = quat[0]
                            centermsg.orientation.x = quat[1]
                            centermsg.orientation.y = quat[2]
                            centermsg.orientation.z = quat[3]
                                
                            self.pose_data.publish(centermsg)

                            self.get_logger().info('Published Center X: %s, %s, %s' % (centermsg.position.x, centermsg.position.y, centermsg.position.z))

                        else: 


                            centermsg = Pose()
                
                            centermsg.position.x = self.cylinderCoords[0]
                            centermsg.position.y = self.cylinderCoords[1]
                            centermsg.position.z = PosZ
                            centermsg.orientation.w = quat[0]
                            centermsg.orientation.x = quat[1]
                            centermsg.orientation.y = quat[2]
                            centermsg.orientation.z = quat[3]
                                
                            self.pose_data.publish(centermsg)

                            self.get_logger().info('Published Center X: %s, %s, %s' % (centermsg.position.x, centermsg.position.y, centermsg.position.z))

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

    def calibrate_callback(self, msg):

        self.A_plane = msg.data[0]
        self.B_plane = msg.data[1]
        self.C_plane = msg.data[2]
        self.D_plane = msg.data[3]




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




