#!/usr/bin/python3

# Author: christoph.roesmann@tu-dortmund.de
# Modifier: Donghee Han, hdh7485@kaist.ac.kr, Pascal Pieper
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

def cmd_callback(data):
  v = data.linear.x
  #interpreting angular Z as angle!
  steering = -data.angular.z    #translate between mathematical and locical turning direction
  #interpreting linear Z as steering velocity.
  steering_v = 0.2 + abs(data.linear.z)

  if message_type == 'ackermann_drive':
    msg = AckermannDrive()
    msg.steering_angle = steering
    msg.steering_velocity = steering_v
    msg.speed = v
    pub.publish(msg)

  else:    
    msg = AckermannDriveStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.drive.steering_angle = steering
    msg.drive.steering_angle_velocity = steering_v
    msg.drive.speed = v
    
    pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    global ackermann_cmd_topic
    global frame_id
    global pub
    global message_type
    global node

    node = rclpy.create_node('cmd_vel_to_ackermann_drive')

    twist_cmd_topic = node.declare_parameter('twist_cmd_topic', '/cmd_vel').value 
    ackermann_cmd_topic = node.declare_parameter('ackermann_cmd_topic', '/ackermann_cmd').value
    frame_id = node.declare_parameter('frame_id', 'odom').value
     # ackermann_drive or ackermann_drive_stamped
    message_type = node.declare_parameter('message_type', 'ackermann_drive_stamped').value

    qos = QoSProfile(depth=1) 
    sub = node.create_subscription(Twist, twist_cmd_topic, cmd_callback, qos_profile=qos)
    if message_type == 'ackermann_drive':
      pub = node.create_publisher(AckermannDrive, ackermann_cmd_topic, qos_profile=qos)
    else:
      pub = node.create_publisher(AckermannDriveStamped, ackermann_cmd_topic, qos_profile=qos)

    node.get_logger().info("Node 'cmd_vel_to_ackermann_drive' started.\n" +
                  "Listening to "+twist_cmd_topic+", publishing to "+ackermann_cmd_topic+". " +
                  "Frame id: " + frame_id)
    rclpy.spin(node)

if __name__ == '__main__':
    main(sys.argv)