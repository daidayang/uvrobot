#!/usr/bin/python3
import time
import serial

WHEEL_RADIUS = 0.03     # in meter
DISTANCE_LEFT_TO_RIGHT_WHEEL = 0.135    # in meter
DISTANCE_FRONT_TO_REAR_WHEEL = 0.115    # in meter
WHEEL_SEPARATION_WIDTH  = DISTANCE_LEFT_TO_RIGHT_WHEEL / 2
WHEEL_SEPARATION_LENGTH = DISTANCE_FRONT_TO_REAR_WHEEL / 2    

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


print("UART Demonstration Program")

# https://www.robotsforroboticists.com/drive-kinematics/

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
# Wait a second to let the port initialize
time.sleep(1)

try:
    # Send a simple header
    serial_port.write("UART motor test\r\n".encode())
    speed = 100
    delta = 1

    while True:
        time.sleep(0.01)

        # target value range -255 0 +255
        # actual value range 1 256 511
        speed += delta
        if speed >= 255:
            delta = -1
        if speed <= -255:
            delta = 1

        UART_Send_Twist(2.5, 0, 0)
        # UART_Send_Twist(0, 0, 25)

        # UART_Send_LRFB(speed, speed, speed, speed)
        # msb = (speed+256) >> 7
        # lsb = (speed+256) & 0X007F

        # packet = bytearray()
        # packet.append(0x81)
        # packet.append(msb)
        # packet.append(lsb)
        # packet.append(msb)
        # packet.append(lsb)
        # packet.append(msb)
        # packet.append(lsb)
        # packet.append(msb)
        # packet.append(lsb)
        # packet.append(0x80)

        # serial_port.write(packet)
        print( speed )

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


except KeyboardInterrupt:
    print("Exiting Program")
    UART_Send_STOP()
    print("Sent zero speed cmd")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    print("Close serial port")
    serial_port.close()
    pass
