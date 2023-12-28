import math

import serial
import threading
import time
import matplotlib.pyplot as plt
from Madjwick import Madgwick

dt = 0.012
g = 983
kalman = 0.01

def transform(a, b):
    a1, a2, a3, a4 = a
    b1, b2, b3, b4 = b

    c1 = a1*b1 - a2*b2 - a3*b3 - a4*b4
    c2 = a1*b2 + a2*b1 + a3*b4 - a4*b3
    c3 = a1*b3 - a2*b4 + a3*b1 + a4*b2
    c4 = a1*b4 + a2*b3 - a3*b2 + a4*b1

    return c1, c2, c3, c4

def receiver(number, serial):
    index = 0

    accel_x_native = []
    accel_y_native = []
    accel_z_native = []

    accel_x_opt = [0]
    accel_y_opt = [0]
    accel_z_opt = [0]

    accel_x = []
    accel_y = []
    accel_z = []
    try:
        while True:
            response = serial.readline()
            current = response.decode('utf-8', errors='ignore')[:-2:]  # [:-2:] to delete '\n'
            print(current)
            m = Madgwick()
            a_x, a_y, a_z, m_x, m_y, m_z, g_x, g_y, g_z = current.split(";")
            m.update(float(g_x), float(g_y), float(g_z), float(a_x), float(a_y), float(a_z), float(m_x), float(m_y), float(m_z))

            index += 1
            q_seq = [0.0, 0.0, 0.0, 0.0]
            m.get_q(q_seq)
            _, ae_x, ae_y, ae_z = transform(transform([q_seq[0], q_seq[1], q_seq[2], q_seq[3]], [0, float(a_x), float(a_y), float(a_z)]), [q_seq[0], -q_seq[1], -q_seq[2], -q_seq[3]])

            accel_x_opt.append((1 - kalman) * accel_x_opt[-1] + kalman * ae_x)
            accel_y_opt.append((1 - kalman) * accel_y_opt[-1] + kalman * ae_y)
            accel_z_opt.append((1 - kalman) * accel_z_opt[-1] + kalman * ae_z)
            ae_x, ae_y, ae_z = float(a_x), float(a_y), float(a_z)
            # ae_z -= g

            accel_x.append(ae_x)
            accel_y.append(ae_y)
            accel_z.append(ae_z)

            accel_x_native.append(a_x)
            accel_y_native.append(a_y)
            accel_z_native.append(a_z)
            time.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
            if index == number:
                raise KeyboardInterrupt

    except KeyboardInterrupt:
        veloc_x = [0]
        veloc_y = [0]
        veloc_z = [0]

        for i in range(number):
            veloc_x.append(veloc_x[-1] + accel_x_opt[i] * dt /1000)
            veloc_y.append(veloc_y[-1] + accel_y_opt[i] * dt /1000)
            veloc_z.append(veloc_z[-1] + accel_z_opt[i] * dt /1000)

        veloc_x.pop(0)
        veloc_y.pop(0)
        veloc_z.pop(0)

        x = [0]
        y = [0]
        z = [0]

        for i in range(number):
            x.append(x[-1] + veloc_x[i] * dt + (accel_x[i] * dt ** 2) / 2)
            y.append(y[-1] + veloc_y[i] * dt + (accel_y[i] * dt ** 2) / 2)
            z.append(z[-1] + veloc_z[i] * dt + (accel_z[i] * dt ** 2) / 2)

        x.pop(0)
        y.pop(0)
        z.pop(0)

        # Create figure and subplots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
        fig.suptitle('Displacement', fontsize=16)

        # Plot data on subplots
        ax1.plot(x, label='x', color='orchid')
        ax1.set_title('along x')
        ax1.legend()

        ax2.plot(y, label='y', color='orchid')
        ax2.set_title('along y')
        ax2.legend()

        ax3.plot(z, label='z', color='orchid')
        ax3.set_title('along z')
        ax3.legend()

        # Adjust layout and display the plot
        plt.tight_layout()
        plt.savefig("displacement.png")


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
