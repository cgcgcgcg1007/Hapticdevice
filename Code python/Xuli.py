import socket
import json
import select
import numpy as np
import time
import tkinter as tk
import math

# ======================
# Math utils
# ======================
def rotation_matrix_around_axis(axis, angle_rad):
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    I = np.eye(3)
    return I + np.sin(angle_rad) * K + (1 - np.cos(angle_rad)) * (K @ K)

def get_axes_from_roll(z_hat, roll_deg, roll_calib_deg=0.0, g_world=np.array([0,-1,0])):
    z_hat = z_hat / np.linalg.norm(z_hat)
    x0 = g_world - np.dot(g_world, z_hat) * z_hat
    n = np.linalg.norm(x0)
    if n < 1e-6:
        tmp = np.array([1,0,0]) if abs(z_hat[0]) < 0.9 else np.array([0,1,0])
        x0 = tmp - np.dot(tmp, z_hat) * z_hat
        x0 /= np.linalg.norm(x0)
    else:
        x0 /= n
    roll_rel = np.radians(roll_deg - roll_calib_deg)
    Rz = rotation_matrix_around_axis(z_hat, roll_rel)
    x_hat = Rz @ x0
    y_hat = np.cross(z_hat, x_hat)
    y_hat /= np.linalg.norm(y_hat)
    return x_hat, y_hat, z_hat

def world_to_wrist_coords_using_roll(elbow, wrist, point_green, roll_deg, roll_calib_deg=0.0, g_world=np.array([0,-1,0])):
    z_hat = (wrist - elbow)
    z_hat = z_hat / (np.linalg.norm(z_hat) + 1e-12)
    x_hat, y_hat, z_hat = get_axes_from_roll(z_hat, roll_deg, roll_calib_deg, g_world)
    R_wrist_world = np.vstack([x_hat, y_hat, z_hat])
    p_world = point_green - wrist
    p_wrist = R_wrist_world @ p_world
    return p_wrist

# ======================
# UDP config
# ======================
UDP_IP = "0.0.0.0"
UDP_PORT_KINECT = 5005
UDP_PORT_ESP = 12345

sock_kinect = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_kinect.bind((UDP_IP, UDP_PORT_KINECT))

sock_esp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_esp.bind((UDP_IP, UDP_PORT_ESP))

print(f"Listening for Kinect on {UDP_PORT_KINECT} and ESP32 on {UDP_PORT_ESP}...")

# Bộ nhớ tạm
kinect_data = None
esp_data = None
esp_addr = None   

ROLL_CALIB_DEG = 0.0
G_WORLD = np.array([0, -1, 0])

# ======================
# Rung logic (giả lập)
# ======================
ON_MAX  = 0.8
ON_MIN  = 0.2
OFF_MAX = 0.2
OFF_MIN = 0.05

lastVibrate = 0
isVibrating = False
curOn, curOff = 0.5, 0.2
currentDir = -1

handover_start = None
handover_done = False

# ======================
# GUI setup
# ======================
root = tk.Tk()
root.title("Kinect Handover Monitor")

status_label = tk.Label(root, text="Waiting...", font=("Arial", 20), width=35, height=3)
status_label.pack(pady=10)

coords_label = tk.Label(root, text="Coords: (0.00, 0.00, 0.00)", font=("Arial", 14))
coords_label.pack(pady=5)

# Vẽ 4 motor
canvas = tk.Canvas(root, width=300, height=300, bg="white")
canvas.pack(pady=10)
motor_ids = {
    "top": canvas.create_oval(120, 20, 180, 80, fill="white"),
    "bottom": canvas.create_oval(120, 220, 180, 280, fill="white"),
    "left": canvas.create_oval(20, 120, 80, 180, fill="white"),
    "right": canvas.create_oval(220, 120, 280, 180, fill="white")
}

def set_motor_visual(top, bottom, left, right):
    canvas.itemconfig(motor_ids["top"],    fill="red" if top else "white")
    canvas.itemconfig(motor_ids["bottom"], fill="red" if bottom else "white")
    canvas.itemconfig(motor_ids["left"],   fill="red" if left else "white")
    canvas.itemconfig(motor_ids["right"],  fill="red" if right else "white")

# ======================
# Hàm update GUI
# ======================
def update_status_ui(status, dist, red_dist, coords=None):
    status_label.config(
        text=f"{status}\n"
             f"d={dist:.2f} m | red_dist={red_dist:.2f} m"
    )
    if "Nguy hiểm" in status:
        status_label.config(bg="red", fg="white")
    elif "giao đồ" in status:
        status_label.config(bg="yellow", fg="black")
    elif "xong" in status:
        status_label.config(bg="blue", fg="white")
    else:
        status_label.config(bg="green", fg="white")
    if coords is not None:
        coords_label.config(text=f"Coords: ({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})")
    root.update_idletasks()

# ======================
# Label buttons
# ======================
def vibrate_label(btn_value):
    # định nghĩa pattern theo nút bấm
    if btn_value == 1:
        pattern = [(250, 250)] * 3
    elif btn_value == 2:
        pattern = [(500, 250)] * 3
    elif btn_value == 3:
        pattern = [(250, 100), (250, 250)] * 3
    else:
        return

    def run_step(i=0):
        if i >= len(pattern):
            set_motor_visual(0,0,0,0)
            return
        on_ms, off_ms = pattern[i]

        # Chỉ rung motor trên
        set_motor_visual(1,0,0,0)

        root.after(on_ms, lambda: (
            set_motor_visual(0,0,0,0),
            root.after(off_ms, lambda: run_step(i+1))
        ))

    run_step(0)

btn_frame = tk.Frame(root)
btn_frame.pack(pady=10)
for i in range(1, 4):
    btn = tk.Button(btn_frame, text=str(i), font=("Arial", 16),
                    width=5, height=2,
                    command=lambda v=i: vibrate_label(v))
    btn.grid(row=0, column=i-1, padx=5)

# ======================
# Process Coords
# ======================
def process_coords(x, y, z):
    global curOn, curOff, currentDir
    r = math.sqrt(x*x + y*y)
    if r < 0.1:
        currentDir = -1
        curOn, curOff = ON_MIN, OFF_MIN
        return
    ratio = max(0.0, min(1.0, (r - 0.13) / 1.0))
    curOn  = ON_MAX  - ratio * (ON_MAX  - ON_MIN)
    curOff = OFF_MAX - ratio * (OFF_MAX - OFF_MIN)
    angle = math.degrees(math.atan2(y, x))
    if angle < 0: angle += 360
    if (angle >= 337.5 or angle < 22.5): currentDir = 0
    elif angle < 67.5: currentDir = 1
    elif angle < 112.5: currentDir = 2
    elif angle < 157.5: currentDir = 3
    elif angle < 202.5: currentDir = 4
    elif angle < 247.5: currentDir = 5
    elif angle < 292.5: currentDir = 6
    else: currentDir = 7

def set_direction_visual(dir):
    if dir == -1: set_motor_visual(1,1,1,1)
    elif dir == 0: set_motor_visual(0,1,0,0)
    elif dir == 1: set_motor_visual(0,1,1,0)
    elif dir == 2: set_motor_visual(0,0,1,0)
    elif dir == 3: set_motor_visual(1,0,1,0)
    elif dir == 4: set_motor_visual(1,0,0,0)
    elif dir == 5: set_motor_visual(1,0,0,1)
    elif dir == 6: set_motor_visual(0,0,0,1)
    elif dir == 7: set_motor_visual(0,1,0,1)

# ======================
# Main loop (tkinter)
# ======================
last_time = 0
PRINT_INTERVAL = 0.05

def tick():
    global kinect_data, esp_data, esp_addr
    global isVibrating, lastVibrate, handover_start, handover_done

    rlist, _, _ = select.select([sock_kinect, sock_esp], [], [], 0)
    for s in rlist:
        data, addr = s.recvfrom(1024)
        msg = data.decode("utf-8").strip()
        if s == sock_kinect:
            try:
                values = list(map(float, msg.split(",")))
                if len(values) == 13:  # có thêm tọa độ marker đỏ
                    dist_red = values[0]
                    wrist = np.array(values[1:4])
                    elbow = np.array(values[4:7])
                    green = np.array(values[7:10])
                    red   = np.array(values[10:13])
                    kinect_data = (elbow, wrist, green, red, dist_red)
            except: pass
        elif s == sock_esp:
            try:
                data_json = json.loads(msg)
                esp_data = {"pitch": data_json.get("pitch",0),
                            "roll": data_json.get("roll",0),
                            "ts": data_json.get("ts",0)}
                esp_addr = addr
            except: pass

    now = time.time()
    if kinect_data and now - last_time >= PRINT_INTERVAL:
        elbow, wrist, green, red, dist_red = kinect_data
        roll = esp_data["roll"] if esp_data else 0
        p_wrist = world_to_wrist_coords_using_roll(
            elbow, wrist, green,
            roll_deg=roll,
            roll_calib_deg=ROLL_CALIB_DEG,
            g_world=G_WORLD
        )
        x,y,z = p_wrist

        # khoảng cách giữa marker đỏ và marker xanh
        red_dist = float(np.linalg.norm(red - green))

        # --- HANDOVER DONE CHECK ---
        r_marker = float(np.sqrt(x*x + y*y))
        if not handover_done:
            if r_marker < 0.1:
                if handover_start is None:
                    handover_start = now
                elif now - handover_start >= 2.0:
                    handover_done = True
                    print("🎉 Handover completed!")
            else:
                handover_start = None

        if handover_done:
            set_motor_visual(0,0,0,0)
            update_status_ui("✅ Giao đồ xong", dist_red, red_dist, p_wrist)
            root.after(10, tick)
            return

        # --- region check ---
        if dist_red <= 0.45:
            status = "⚠ Nguy hiểm!"
        elif dist_red <= 1.5:
            status = "🤝 Vùng giao đồ"
        else:
            status = "✅ An toàn"

        update_status_ui(status, dist_red, red_dist, p_wrist)

        # --- rung logic với hysteresis ---
        if status == "🤝 Vùng giao đồ":
            if not hasattr(tick, "mode"):
                tick.mode = "all"

            if red_dist > 0.35:
                tick.mode = "all"
            elif red_dist < 0.25:
                tick.mode = "dir"

            if tick.mode == "all":
                set_motor_visual(1,1,1,1)
            else:
                process_coords(x,y,z)
                if isVibrating:
                    if now - lastVibrate >= curOn:
                        isVibrating = False
                        lastVibrate = now
                        set_motor_visual(0,0,0,0)
                else:
                    if now - lastVibrate >= curOff:
                        isVibrating = True
                        lastVibrate = now
                        set_direction_visual(currentDir)
        else:
            set_motor_visual(0,0,0,0)

    root.after(10, tick)

# start loop
root.after(10, tick)
root.mainloop()
