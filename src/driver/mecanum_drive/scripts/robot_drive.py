#!/usr/bin/python
import serial
import rospy
from mecanum_drive import util
from std_msgs.msg import String
from geometry_msgs.msg import Twist


WHEEL_RADIUS = 0.03     # in meter
DISTANCE_LEFT_TO_RIGHT_WHEEL = 0.135    # in meter
DISTANCE_FRONT_TO_REAR_WHEEL = 0.115    # in meter
WHEEL_SEPARATION_WIDTH  = DISTANCE_LEFT_TO_RIGHT_WHEEL / 2
WHEEL_SEPARATION_LENGTH = DISTANCE_FRONT_TO_REAR_WHEEL / 2

serial_buffer = ""

serial_port = serial.Serial (
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE )

# Create publisher
pub_robotrx = rospy.Publisher("robot_rx", String, queue_size=1)

def UART_Send_STOP():
    UART_Send_LRFB(0, 0, 0, 0)

    # packet = bytearray()
    # packet.append(0x81)
    # packet.append(0x02)
    # packet.append(0x00)
    # packet.append(0x02)
    # packet.append(0x00)
    # packet.append(0x02)
    # packet.append(0x00)
    # packet.append(0x02)
    # packet.append(0x00)
    # packet.append(0x80)
    # serial_port.write(packet)

def UART_Send_LRFB(lf, lb, rf, rb):

    packet = bytearray()
    packet.append(0x81)

    msb = (lf+256) >> 7
    lsb = (lf+256) & 0X007F
    packet.append(msb)
    packet.append(lsb)

    msb = (lb+256) >> 7
    lsb = (lb+256) & 0X007F
    packet.append(msb)
    packet.append(lsb)

    msb = (rf+256) >> 7
    lsb = (rf+256) & 0X007F
    packet.append(msb)
    packet.append(lsb)

    msb = (rb+256) >> 7
    lsb = (rb+256) & 0X007F
    packet.append(msb)
    packet.append(lsb)

    packet.append(0x80)

    serial_port.write(packet)


def UART_Send_Twist(linear_x, linear_y, angular_z):
    wheel_front_left  = (1/WHEEL_RADIUS) * (linear_x - linear_y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angular_z)
    wheel_front_right = (1/WHEEL_RADIUS) * (linear_x + linear_y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angular_z)
    wheel_rear_left   = (1/WHEEL_RADIUS) * (linear_x + linear_y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angular_z)
    wheel_rear_right  = (1/WHEEL_RADIUS) * (linear_x - linear_y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angular_z)

    UART_Send_LRFB(int(wheel_front_left), int(wheel_rear_left), int(wheel_front_right), int(wheel_rear_right))


# Define Timer callback
def TIMER_callback(event):
    global serial_buffer

    if serial_port.inWaiting() > 0:
        data = serial_port.read()
        # if we get a carriage return, add a line feed too
        # \r is a carriage return; \n is a line feed
        # This is to help the tty program on the other end
        # Windows is \r\n for carriage return, line feed
        # Macintosh and Linux use \n

        if data == "\n".encode() or data == "\r".encode():
            if len(serial_buffer) > 0:
                pub_robotrx.publish(serial_buffer)
                rospy.loginfo(serial_buffer)
            serial_buffer = ""
        else:
            serial_buffer += data


def twist_callback(data):
    print('Data from /twist_raw received')


def xbox_callback(msg):

    if msg.data[0] == 'D':
        forward = int(msg.data[2:4], 16)
        angle = int(msg.data[4:6], 16)

        UART_Send_Twist(3.5 * ( forward - 128 ) / 128, 0, 50.0 * (128-angle)/128)

        rospy.loginfo("XBOX Drive: %f:%f", 3.5 * ( forward - 128 ) / 128, 50.0 * (128-angle)/128)

    if msg.data[0] == 'C':
        rospy.loginfo("XBOX Control: %s", msg.data)

def main():

    # initialize a node by the name 'listener'.
    # you may choose to name it however you like,
    # since you don't have to use it ahead
    rospy.init_node('mecanum_drive')

    rospy.Subscriber("xbox", String, xbox_callback)
    rospy.Subscriber("twist_raw", Twist, twist_callback)

    # Read parameter
    pub_period = rospy.get_param("~pub_period",0.05)

    # Create timer
    rospy.Timer(rospy.Duration.from_sec(pub_period), TIMER_callback)

    # spin() simply keeps python from
    # exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass
