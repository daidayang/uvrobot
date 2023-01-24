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

serial_port = serial.Serial (
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE )


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

    if serial_port.inWaiting() > 0:
        data = serial_port.read()
        print(data)
        serial_port.write(data)
        # if we get a carriage return, add a line feed too
        # \r is a carriage return; \n is a line feed
        # This is to help the tty program on the other end 
        # Windows is \r\n for carriage return, line feed
        # Macintosh and Linux use \n
        if data == "\r".encode():
            # For Windows boxen on the other end
            serial_port.write("\n".encode())


def twist_callback(data):
    print('Data from /twist_raw received')


def xbox_callback(data):
      
    # print the actual message in its raw format
    rospy.loginfo("Here's what was subscribed: %s", data.data)
      
    # otherwise simply print a convenient message on the terminal
    print('Data from /topic_name received')

  
def main():
      
    # initialize a node by the name 'listener'.
    # you may choose to name it however you like,
    # since you don't have to use it ahead
    rospy.init_node('mecanum_drive', anonymous=True)

    rospy.Subscriber("xbox", String, xbox_callback)
    rospy.Subscriber("twist_raw", Twist, twist_callback)

    # Create publisher
    publisher = rospy.Publisher("~topic",String,queue_size=1)

    # apdriverx_pub_ = nh_.advertise<std_msgs::String>("ApDrvRx", 1);
    # apdrivetx_pub_ = nh_.advertise<std_msgs::String>("ApDrvTx", 1);

    # Read parameter
    pub_period = rospy.get_param("~pub_period",1.0)

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
