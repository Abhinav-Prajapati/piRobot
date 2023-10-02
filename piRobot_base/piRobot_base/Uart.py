import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
from threading import Lock

class Uart(Node):
    def __init__(self):
        super().__init__('uart_node')

        self.wheelRadius = 0.2
        self.wheelSeparation = 0.35
        self.wheelAngle = 120.0 * (math.pi / 180.0)

        self.loop_rate=0.020 # miliseconds
        self.MotorPWM:[int,int,int]=[0,0,0] 


        self.mutex = Lock()
        self.ser = serial.Serial('/dev/ttyACM0', 115200) 
        self.uart_timer=self.create_timer(self.loop_rate,self.uart_timer_callback)
        self.subscription = self.create_subscription(Twist,'piRobot/cmd_vel',self.twist_callback,10)
        # self.encoder=self.create_publisher(Motor_encoder,'/drive/raw_encoder',10)

        
    def twist_callback(self, msg):
        xVel = msg.linear.x
        yVel = msg.linear.y
        zAngular = msg.angular.z

        self.MotorPWM[0] = (-xVel - (self.wheelSeparation / 2.0) * zAngular) / self.wheelRadius
        self.MotorPWM[1] = (-xVel * math.cos(self.wheelAngle) + yVel * math.sin(self.wheelAngle) - (self.wheelSeparation / 2.0) * zAngular) / self.wheelRadius
        self.MotorPWM[2] = (-xVel * math.cos(self.wheelAngle) - yVel * math.sin(self.wheelAngle) - (self.wheelSeparation / 2.0) * zAngular) / self.wheelRadius
        
        self.get_logger().info('Motor PWM values: Motor1={}, Motor2={}, Motor3={}'.format(self.MotorPWM[0], self.MotorPWM[1], self.MotorPWM[2]))

    def uart_timer_callback(self):
        
        self.mutex.acquire()

        try:

            dataToSend = f"{self.MotorPWM[0]*255},{self.MotorPWM[1]},{self.MotorPWM[2]},\n".encode('utf-8')
            self.ser.write('m'.encode('utf-8'))
            self.ser.write(dataToSend)

            # response = self.ser.readline().strip()
            # self.get_logger().info(f"sent {dataToSend} rec {response}")
            # response=0

            self.ser.write('e'.encode('utf-8'))
            response = self.ser.readline().strip()  # read(3) will read 3 bytes
            # print(f"revived {response}")

            # todo: publish encoder data here 
            # encoder_data:[int,int,int]=[int(num) for num in response.split()] 

        finally:
            self.mutex.release()

def main(args=None):
    rclpy.init(args=args)
    node = Uart()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()