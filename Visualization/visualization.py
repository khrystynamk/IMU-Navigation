"""
Visualization of the board in a 3D space.
"""
import time
import serial
import threading
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

from MadgwickFilter.madgwick import Madgwick
from figure import *
from tkinter import Tk, Button, INSERT, Label, Entry

GRAVITATIONAL_ACCELERATION = 9.81
ApplicationGL = False
m = Madgwick()
display = (0, 0)
serial_object = serial.Serial()


class PortSettings:
    Name = "/dev/tty.usbserial-0001"
    Speed = 115200
    Timeout = 1


class IMU:
    Roll = 0
    Pitch = 0
    Yaw = 0


my_port = PortSettings()
my_imu = IMU()


# The function is called when the "Ok" button is pressed.
# It updates port name, baud rate (attributes Name and Speed of the my_port instance)
# and destroys the configuration window.
def run_app():
    global ApplicationGL
    ApplicationGL = True
    my_port.Name = port_entry.get()
    my_port.Speed = baud_entry.get()
    conf_window.destroy()


# Creating a window
conf_window = Tk()
conf_window.title("Configure Serial Port")
conf_window.configure(bg="#4c5b91")
conf_window.geometry('500x250')
conf_window.resizable(width=False, height=False)

# Setting position of the window
positionLeft = int(conf_window.winfo_screenwidth() / 2 - 500 / 2)
positionTop = int(conf_window.winfo_screenheight() / 2 - 250 / 2)
# The argument passed to geometry should be of the form =widthxheight+x+y.
# So it specifies the width, height, and position of the window.
conf_window.geometry(f"+{positionLeft}+{positionTop}")

# Port name label
port_label = Label(text="Port:", font=("Gotu", 21), justify="right", bg="#4c5b91", fg="#FFFFFF")
port_label.place(x=130, y=60, anchor="center")
# Port name input
port_entry = Entry(width=22, bg="#37364D", fg="#FFFFFF", justify="center", font=("STHeiti", 16))
port_entry.insert(INSERT, my_port.Name)
port_entry.place(x=285, y=60, anchor="center")

# Baud rate label
baud_label = Label(text="Speed:", font=("Gotu", 21), justify="right", bg="#4c5b91", fg="#FFFFFF")
baud_label.place(x=130, y=110, anchor="center")
# Baud rate input
baud_entry = Entry(width=22, bg="#37364D", fg="#FFFFFF", justify="center", font=("STHeiti", 16))
baud_entry.insert(INSERT, str(my_port.Speed))
baud_entry.place(x=285, y=110, anchor="center")

# Ok button
ok_button = Button(text="Ok", width=8, command=run_app, fg="#000000",
                   font=("Gotu", 18), cursor="pirate", bd=0)
ok_button.place(x=250, y=170, anchor="center")


def init_pygame():
    global display
    pygame.init()
    display = (1300, 700)
    # Using double buffering (it helps to prevent flickering in the display) and OpenGl rendering
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption('IMU visualizer   (Press Esc to exit)')


def init_gl():
    # Specify the red, green, blue, and alpha (RGBA) values used when the color buffers are cleared.
    glClearColor((1.0 / 255 * 46), (1.0 / 255 * 45), (1.0 / 255 * 64), 1)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    #       fovy = 100
    # Specifies the field of view angle, in degrees, in the y direction.
    #       aspect = display[0] / display[1]
    # Specifies the aspect ratio that determines the field of view in the x direction.
    # The aspect ratio is the ratio of x (width) to y (height).
    #       zNear = 0.1
    # Specifies the distance from the viewer to the near clipping plane (always positive).
    #       zFar = 50.0
    # Specifies the distance from the viewer to the far clipping plane (always positive).
    gluPerspective(100, display[0] / display[1], 0.1, 50.0)
    # Move an object by the given x-, y-, z-values.
    glTranslatef(0.0, 0.0, -5)


def draw_text(text_str):
    font = pygame.font.SysFont("Courier New", 25, True)
    # antialiasing = True: enables antialiasing for smoother text rendering.
    text_surface = font.render(text_str, True, (255, 255, 0), (46, 45, 64, 255))
    # Transfer image to byte buffer.
    # "RGBA" specifies the pixel format.
    text_data = pygame.image.tostring(text_surface, "RGBA", True)
    # Draw pixels directly onto the framebuffer.
    glDrawPixels(text_surface.get_width(), text_surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, text_data)


def draw_board():
    glBegin(GL_QUADS)
    for i, surface in enumerate(surfaces):
        for vertex in surface:
            # Specify a color for 6 planes.
            glColor3fv((colors[i]))
            # Specify a vertex in 3D space.
            glVertex3fv(vertices[vertex])
    glEnd()


def draw_gl():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # Replace the current matrix with the identity matrix.
    glLoadIdentity()
    gluPerspective(100, (display[0] / display[1]), 0.1, 50.0)
    # Move the scene 5 units along the negative z-axis to position the objects within the view.
    glTranslatef(0.0, 0.0, -5)

    # Apply rotations to the current matrix.
    glRotatef(round(my_imu.Pitch, 1), 1, 0, 0)
    glRotatef(round(my_imu.Yaw, 1), 0, 1, 0)
    glRotatef(round(my_imu.Roll, 1), 0, 0, 1)

    draw_text(f"    Roll: {round(my_imu.Roll, 1)}°"
              f"    Pitch: {round(my_imu.Pitch, 1)}°"
              f"    Yaw: {round(my_imu.Yaw, 1)}°")

    # Rendering a 3D board with OpenGl.
    draw_board()
    # Updates the display to show the rendered frame.
    pygame.display.flip()


def serial_connection():
    global serial_object
    serial_object = serial.Serial(my_port.Name, baudrate=my_port.Speed, timeout=my_port.Timeout,
                                  parity="N", stopbits=1, bytesize=8, xonxoff=False, rtscts=False)


def read_data():
    while True:
        response = serial_object.readline()
        # Decode bytes to string
        current = response.decode('utf-8', errors='ignore')

        global m
        a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z = current.split(",")
        m.update(float(g_x), float(g_y), float(g_z),
                 float(a_x) * GRAVITATIONAL_ACCELERATION,
                 float(a_y) * GRAVITATIONAL_ACCELERATION,
                 float(a_z) * GRAVITATIONAL_ACCELERATION,
                 float(m_x), float(m_y), float(m_z))
        m.compute_angles()

        my_imu.Roll = m.roll * 180
        my_imu.Pitch = m.pitch * 180
        my_imu.Yaw = m.yaw * 180


def main():
    conf_window.mainloop()
    if ApplicationGL:
        init_pygame()
        init_gl()

        try:
            serial_connection()
            my_thread1 = threading.Thread(target=read_data)
            my_thread1.daemon = True
            my_thread1.start()
            while True:
                event = pygame.event.poll()
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    pygame.quit()
                    break

                draw_gl()
                pygame.time.wait(10)

        except Exception as e:
            print(e)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            draw_text("Sorry, something is wrong :c")
            pygame.display.flip()
            time.sleep(5)


if __name__ == '__main__':
    main()
