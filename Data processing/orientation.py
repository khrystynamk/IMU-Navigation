import math

import serial
import threading
import time
from Madjwick import Madgwick
import matplotlib.pyplot as plt

def receiver(number, serial):
    roll = []
    pitch = []
    yaw = []
    x_raw = []
    y_raw = []
    z_raw = []
    index = 0
    try:
        while True:
            response = serial.readline()
            current = response.decode('utf-8', errors='ignore')[:-2:] # Decode bytes to string
            print(current)
            m = Madgwick()
            a_x, a_y, a_z, m_x, m_y, m_z, g_x, g_y, g_z = current.split(";")

            yaw_raw = math.atan2(math.sqrt(float(g_y) * float(g_y) + float(g_z) * float(g_z)), float(g_x)) * 57.3
            roll_raw = math.atan2(float(a_y), float(a_z)) * 57.3
            pitch_raw = math.asin((float(a_x)) / math.sqrt(float(a_x) * float(a_x) + float(a_y) * float(a_y) + float(a_z) * float(a_z))) * 57.3

            x_raw.append(roll_raw)
            y_raw.append(pitch_raw)
            z_raw.append(yaw_raw)

            m.update(float(g_x), float(g_y), float(g_z), float(a_x), float(a_y), float(a_z), float(m_x), float(m_y), float(m_z))
            m.computeAngles()
            roll.append(m.roll * 57.3)
            pitch.append(m.pitch * 57.3)
            yaw.append(m.yaw * 57.3)
            index += 1
            time.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
            if index == number:
                raise KeyboardInterrupt

    except KeyboardInterrupt:

        # Create figure and subplots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
        fig.suptitle('Orientation: roll, pitch and yaw', fontsize=16)

        # Plot data on subplots
        ax1.plot(roll,  label='roll', color='royalblue')
        ax1.plot(x_raw, label='roll raw', color='salmon')
        # ax1.set_ylim(-2, 2)
        ax1.set_title('roll')
        ax1.legend()

        ax2.plot(pitch, label='pitch', color='royalblue')
        ax2.plot(y_raw, label='pitch raw', color='salmon')
        ax2.set_title('pitch')
        # ax2.set_ylim(-2, 2)
        ax2.legend()

        ax3.plot(yaw, label='yaw', color='royalblue')
        ax3.plot(z_raw, label='yaw raw', color='salmon')
        # ax3.set_ylim(-2, 2)
        ax3.set_title('Yaw')
        ax3.legend()

        # Adjust layout and display the plot
        plt.tight_layout()
        plt.savefig("orientation.png")


# Serial port settings
port = "/dev/tty.usbserial-0001"
filename = "data.txt"
number = 300
baudrate = 115200
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE
bytesize = serial.EIGHTBITS
timeout = 1  # Timeout value in seconds
xonxoff = False  # Disable software flow control (XON/XOFF)
rtscts = False   # Disable hardware flow control (RTS/CTS)

ser = serial.Serial(port, baudrate=baudrate, parity=parity, stopbits=stopbits,
                    bytesize=bytesize, timeout=timeout, xonxoff=xonxoff, rtscts=rtscts)

print(f"Serial port configured: {ser.name}")

t = threading.Thread(target=receiver, args=(number,ser,))
t.start()
