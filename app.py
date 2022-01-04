import numpy as np
import tkinter as tk
from tkinter import ttk
from pyfirmata import Arduino, SERVO
import serial.tools.list_ports


def get_ports():
    ports = serial.tools.list_ports.comports()
    return ports


def find_arduino(ports_found):
    com_port = 'None'
    num_connection = len(ports_found)
    for i in range(0, num_connection):
        port = found_ports[i]
        str_port = str(port)
        if 'CH340' in str_port:
            split_port = str_port.split(' ')
            com_port = (split_port[4])
    return com_port.translate(str.maketrans({"(": "", ")": ""}))


found_ports = get_ports()
connect_port = find_arduino(found_ports)


class MainApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("5DoF Robot Control")
        self.geometry("640x540")
        self.config(background="#FFFFFF")
        self.iconbitmap(r'ico_arm.ico')
        s = ttk.Style()
        s.configure('Style', foreground='white')

        title_f_label = ttk.Label(self, text="Forward Kinematics", font=("Arial", 12))
        title_f_label.grid(column=0, row=0, columnspan=4, sticky=tk.W, padx=10, pady=15)

        title_i_label = ttk.Label(self, text="Inverse Kinematics", font=("Arial", 12))
        title_i_label.grid(column=0, row=8, columnspan=4, sticky=tk.W, padx=10, pady=15)

        if connect_port != 'None':
            board = Arduino(connect_port)

        def servo1(angle):
            board.digital[3].mode = SERVO
            board.digital[3].write(slide_1.get())
            print(angle)

        def servo2(angle):
            board.digital[5].mode = SERVO
            board.digital[5].write(slide_2.get())
            print(angle)

        def servo3(angle):
            board.digital[6].mode = SERVO
            board.digital[6].write(slide_3.get())
            print(angle)

        def servo4(angle):
            board.digital[9].mode = SERVO
            board.digital[9].write(slide_4.get())
            print(angle)

        def servo5(angle):
            board.digital[10].mode = SERVO
            board.digital[10].write(slide_5.get())
            print(angle)

        def grip():
            board.digital[11].mode = SERVO
            board.digital[11].write(0)

        def release():
            board.digital[11].mode = SERVO
            board.digital[11].write(90)

        a1, a2 = 14, 120
        d1, d4, d6 = 100, 118, 120

        def forward_kinematics():
            q1 = np.round(slide_1.get())
            q2 = np.round(slide_2.get())
            q3 = np.round(slide_3.get())
            q4 = np.round(slide_4.get())
            q5 = np.round(slide_5.get())

            s1, s2, s3, s4, s5 = np.sin(q1), np.sin(q2), np.sin(q3), np.sin(q4), np.sin(q5)
            c1, c2, c3, c4, c5 = np.cos(q1), np.cos(q2), np.cos(q3), np.cos(q4), np.cos(q5)
            s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

            px = -d6 * c1 * s23 * c4 * s5 - d6 * s1 * s4 * s5 + \
                d6 * c1 * c23 * c5 + d4 * c1 * c23 + a2 * c1 * c2 + a1 * c1
            py = -d6 * s1 * s23 * c4 * s5 + d6 * c1 * s4 * s5 + \
                d6 * s1 * c23 * c5 + d4 * s1 * c23 + a2 * s1 * c2 + a1 * s1
            pz = -d6 * c23 * c4 * s5 - d6 * s23 * c5 - d4 * s23 - a2 * s2 + d1

            v1 = ttk.Label(text=np.round(px), font=("Arial", 10))
            v1.grid(row=6, column=2, pady=4)

            v2 = ttk.Label(text=np.round(py), font=("Arial", 10))
            v2.grid(row=6, column=4, pady=4)

            v3 = ttk.Label(text=np.round(pz), font=("Arial", 10))
            v3.grid(row=6, column=6, pady=4)

            for i in [v1, v2, v3]:
                i.config(background="#FFFFFF")

            print(f"X coordinates: {px} [mm]\nY coordinates: {py}[mm]\nZ coordinates: {pz}[mm]")

        def position_inverse_kinematics():
            px = float(px_entry.get())
            py = float(py_entry.get())
            pz = float(pz_entry.get())

            t1 = np.arctan2(py, px)
            r_1 = np.sqrt((px - a1 * np.cos(t1)) ** 2 + (py - a1 * np.sin(t1)) ** 2)
            r_2 = pz - d1
            r_3 = np.sqrt(r_1 ** 2 + r_2 ** 2)
            phi_1 = np.arccos((d4 ** 2 - a2 ** 2 - r_3 ** 2) / (-2 * a2 * r_3))
            phi_2 = np.arctan2(r_2, r_1)
            phi_3 = np.arccos((r_3 ** 2 - a2 ** 2 - d4 ** 2) / (-2 * a2 * d4))
            t2 = phi_2 - phi_1
            t3 = np.pi - phi_3

            print(f"First three joint angles: {np.degrees(t1)}, {np.degrees(t2)}, {np.degrees(t3)} [deg].")

            board.digital[3].mode = SERVO
            board.digital[3].write(np.degrees(t1))

            board.digital[5].mode = SERVO
            board.digital[5].write(-1 * np.degrees(t2))

            board.digital[6].mode = SERVO
            board.digital[6].write(np.degrees(t3))

        def set_orientation(event):
            px = x_slide.get()
            py = 0
            pz = 20

            if x1.get() == 1:
                k = down
            else:
                raise TypeError

            nx = k[0, 0]
            ny = k[1, 0]
            nz = k[2, 0]
            ox = k[0, 1]
            oy = k[1, 1]
            oz = k[2, 1]
            ax = k[0, 2]
            ay = k[1, 2]
            az = k[2, 2]

            j1 = np.arctan2(-ay * d6 + py, -ax * d6 + px)

            b1 = az * d6 - pz + d1
            b2 = np.sqrt((-ax * d6 + px) ** 2 + (-ay * d6 + py) ** 2) - a1
            gamma = np.arctan2(b2, b1)
            j2 = np.arcsin((b1 ** 2 + b2 ** 2 + a2 ** 2 - d4 ** 2) / (2 * a2 * np.sqrt(b1 ** 2 + b2 ** 2))) - gamma

            j3 = np.arctan2(b1 - a2 * np.sin(j2), b2 - a2 * np.cos(j2)) - j2

            s1, s2, s3 = np.sin(j1), np.sin(j2), np.sin(j3)
            c1, c2, c3 = np.cos(j1), np.cos(j2), np.cos(j3)
            s23, c23 = np.sin(j2 + j3), np.cos(j2 + j3)

            j4 = np.arctan2(-ox * c1 * s23 - oy * s1 * s23 - oz * c23, ox * s1 - oy * c1)
            j5 = np.arctan2(nx * c1 * c23 + ny * s1 * c23 - nz * s23, -ax * c1 * c23 + ay * s1 * s23 - az * s23)

            print(np.degrees(j1), np.degrees(j2), np.degrees(j3), np.degrees(j4), np.degrees(j5))

            board.digital[3].mode = SERVO
            board.digital[3].write(np.degrees(j1))

            board.digital[5].mode = SERVO
            board.digital[5].write(-1 * np.degrees(j2))

            board.digital[6].mode = SERVO
            board.digital[6].write(np.degrees(j3))

            board.digital[9].mode = SERVO
            board.digital[9].write(90 + np.degrees(j4))

            board.digital[10].mode = SERVO
            board.digital[10].write(np.degrees(j5))

        def servo_roll(angle):
            board.digital[9].mode = SERVO
            board.digital[9].write(roll_slide.get())
            print(angle)

        def servo_pitch(angle):
            board.digital[10].mode = SERVO
            board.digital[10].write(pitch_slide.get())
            print(angle)

        calculate_button = ttk.Button(text="Calculate", command=forward_kinematics)
        calculate_button.grid(row=6, column=0, padx=4, pady=2)

        mm_label = ttk.Label(text="[mm]", font=("Arial", 10))
        mm_label.grid(row=6, column=8, padx=10)

        px_label = ttk.Label(text="Px:", font=("Arial", 10))
        px_label.grid(row=6, column=1, padx=10, pady=10)

        py_label = ttk.Label(text="Py:", font=("Arial", 10))
        py_label.grid(row=6, column=3, padx=10, pady=10)

        pz_label = ttk.Label(text="Pz:", font=("Arial", 10))
        pz_label.grid(row=6, column=5, padx=10, pady=10)

        base_label = ttk.Label(text="Base Joint", font=("Arial", 10))
        base_label.grid(row=1, column=0, padx=10, pady=2)
        theta_1 = tk.DoubleVar()
        slide_1 = ttk.Scale(from_=0, to=180, value=0, variable=theta_1,
                            command=servo1, orient='horizontal', length=300)
        slide_1.grid(row=1, column=1, columnspan=6, pady=2)
        spin_1 = ttk.Spinbox(textvariable=theta_1, wrap=True, width=8, increment=1)
        spin_1.grid(row=1, column=7)
        deg_label1 = ttk.Label(text="[deg]", font=("Arial", 10))
        deg_label1.grid(row=1, column=8, padx=10)

        shoulder_label = ttk.Label(text="Shoulder Joint", font=("Arial", 10))
        shoulder_label.grid(row=2, column=0, padx=10, pady=2)
        theta_2 = tk.DoubleVar()
        slide_2 = ttk.Scale(from_=0, to=180, value=0, variable=theta_2,
                            command=servo2, orient='horizontal', length=300)
        slide_2.grid(row=2, column=1, columnspan=6, pady=2)
        spin_2 = ttk.Spinbox(textvariable=theta_2, wrap=True, width=8, increment=1)
        spin_2.grid(row=2, column=7)
        deg_label2 = ttk.Label(text="[deg]", font=("Arial", 10))
        deg_label2.grid(row=2, column=8, padx=10)

        elbow_label = ttk.Label(text="Elbow Joint", font=("Arial", 10))
        elbow_label.grid(row=3, column=0, padx=10, pady=2)
        theta_3 = tk.DoubleVar()
        slide_3 = ttk.Scale(from_=0, to=180, value=0, variable=theta_3,
                            command=servo3, orient='horizontal', length=300)
        slide_3.grid(row=3, column=1, columnspan=6, pady=2)
        spin_3 = ttk.Spinbox(textvariable=theta_3, wrap=True, width=8, increment=1)
        spin_3.grid(row=3, column=7)
        deg_label3 = ttk.Label(text="[deg]", font=("Arial", 10))
        deg_label3.grid(row=3, column=8, padx=10)

        roll_label = ttk.Label(text="Wrist Roll", font=("Arial", 10))
        roll_label.grid(row=4, column=0, padx=10, pady=2)
        theta_4 = tk.DoubleVar()
        slide_4 = ttk.Scale(from_=0, to=180, value=90, variable=theta_4,
                            command=servo4, orient='horizontal', length=300)
        slide_4.grid(row=4, column=1, columnspan=6, pady=2)
        spin_4 = ttk.Spinbox(textvariable=theta_4, wrap=True, width=8, increment=1)
        spin_4.grid(row=4, column=7)
        deg_label4 = ttk.Label(text="[deg]", font=("Arial", 10))
        deg_label4.grid(row=4, column=8, padx=10)

        pitch_label = ttk.Label(text="Wrist Pitch", font=("Arial", 10))
        pitch_label.grid(row=5, column=0, padx=10, pady=2)
        theta_5 = tk.DoubleVar()
        slide_5 = ttk.Scale(from_=0, to=180, value=90, variable=theta_5,
                            command=servo5, orient='horizontal', length=300)
        slide_5.grid(row=5, column=1, columnspan=6, pady=2)
        spin_5 = ttk.Spinbox(textvariable=theta_5, wrap=True, width=8, increment=1)
        spin_5.grid(row=5, column=7)
        deg_label5 = ttk.Label(text="[deg]", font=("Arial", 10))
        deg_label5.grid(row=5, column=8, padx=10)

        def home():
            for i in [theta_1, theta_2, theta_3]:
                i.set(0)
            for i in [theta_4, theta_5]:
                i.set(90)

        home_button = ttk.Button(text="Home", command=lambda: [home(),
                                                               servo1(theta_1),
                                                               servo2(theta_2),
                                                               servo3(theta_3),
                                                               servo4(theta_4),
                                                               servo5(theta_5)])
        home_button.grid(row=7, column=0, pady=2)

        saved_positions = []

        def save_positions():
            new_pos = [np.round(slide_1.get()),
                       np.round(slide_2.get()),
                       np.round(slide_3.get()),
                       np.round(slide_4.get()),
                       np.round(slide_5.get())]
            for i in new_pos:
                saved_positions.append(i)

        save_button = ttk.Button(text="Save", command=save_positions)
        save_button.grid(row=7, column=1, columnspan=2, pady=2)

        def run():
            theta_1.set(saved_positions[0])
            theta_2.set(saved_positions[1])
            theta_3.set(saved_positions[2])
            theta_4.set(saved_positions[3])
            theta_5.set(saved_positions[4])

        run_button = ttk.Button(text="Run", command=lambda: [run(),
                                                             servo1(theta_1),
                                                             servo2(theta_2),
                                                             servo3(theta_3),
                                                             servo4(theta_4),
                                                             servo5(theta_5)])
        run_button.grid(row=7, column=5, columnspan=2)

        clear_button = ttk.Button(text="Clear", command=lambda: saved_positions.clear())
        clear_button.grid(row=7, column=3, columnspan=2, pady=2)

        grip_button = ttk.Button(text="Grip", command=grip)
        grip_button.grid(row=7, column=7, padx=8, pady=2)

        release_button = ttk.Button(text="Release", command=release)
        release_button.grid(row=7, column=8, padx=8, pady=2)

        x_position = tk.DoubleVar()
        y_position = tk.DoubleVar()
        z_position = tk.DoubleVar()

        px_entry = ttk.Spinbox(textvariable=x_position, wrap=True, width=8, increment=1)
        px_entry.grid(row=9, column=2)

        py_entry = ttk.Spinbox(textvariable=y_position, wrap=True, width=8, increment=1)
        py_entry.grid(row=9, column=4)

        pz_entry = ttk.Spinbox(textvariable=z_position, wrap=True, width=8, increment=1)
        pz_entry.grid(row=9, column=6)

        run_i_button = ttk.Button(text="Run", command=position_inverse_kinematics)
        run_i_button.grid(row=9, column=0, pady=2)

        px_i_label = ttk.Label(text="Px:", font=("Arial", 10))
        px_i_label.grid(row=9, column=1, padx=10, pady=10)

        py_i_label = ttk.Label(text="Py:", font=("Arial", 10))
        py_i_label.grid(row=9, column=3, padx=10, pady=10)

        pz_i_label = ttk.Label(text="Pz:", font=("Arial", 10))
        pz_i_label.grid(row=9, column=5, padx=10, pady=10)

        roll_i_label = ttk.Label(text="Roll", font=("Arial", 10))
        roll_i_label.grid(row=8, column=7)

        pitch_i_label = ttk.Label(text="Pitch", font=("Arial", 10))
        pitch_i_label.grid(row=8, column=8)

        roll_slide = ttk.Scale(from_=0, to=180, value=0, variable=theta_4,
                               command=servo_roll, orient='vertical', length=200)
        roll_slide.grid(row=9, column=7, rowspan=4)

        pitch_slide = ttk.Scale(from_=0, to=180, value=0, variable=theta_5,
                                command=servo_pitch, orient='vertical', length=200)
        pitch_slide.grid(row=9, column=8, rowspan=4)

        orientation_label = ttk.Label(text="Orientation Lock", font=("Arial", 11))
        orientation_label.grid(row=10, column=0, columnspan=2)

        down = np.array([[1, 0, 0],
                         [0, -1, 0],
                         [0, 0, -1]])

        x1 = tk.IntVar()
        x2 = tk.IntVar()
        x3 = tk.IntVar()

        s = ttk.Style()
        s.configure("TCheckbutton", background='white', font=("Arial", 10))

        check_1 = ttk.Checkbutton(text="Down", variable=x1, style='TCheckbutton')
        check_1.grid(row=10, column=2)

        check_2 = ttk.Checkbutton(text="Forward", variable=x2, style='TCheckbutton')
        check_2.grid(row=10, column=4)

        check_3 = ttk.Checkbutton(text="Up", variable=x3, style='TCheckbutton')
        check_3.grid(row=10, column=6)

        x = tk.DoubleVar()
        x_slide = ttk.Scale(from_=100, to=200, value=0, variable=x,
                            command=set_orientation, orient='horizontal', length=400)
        x_slide.grid(row=11, column=0, columnspan=7)

        my_label = ttk.Label(text="Gheorma Cristian-Tudor", font=("Arial", 11))
        my_label.grid(row=12, column=0, columnspan=3)

        labels = [title_f_label, mm_label, px_label, py_label, pz_label, base_label, shoulder_label, elbow_label,
                  roll_label, pitch_label, deg_label1, deg_label2, deg_label3, deg_label4, deg_label5, title_i_label,
                  px_i_label, py_i_label, pz_i_label, roll_i_label, pitch_i_label, orientation_label, my_label]

        for label in labels:
            label.config(background="#FFFFFF")


if __name__ == '__main__':
    MainApp().mainloop()
