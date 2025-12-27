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
    z_hat_norm = np.linalg.norm(z_hat)
    if z_hat_norm < 1e-6:
        z_hat = np.array([0, 0, 1])
    else:
        z_hat = z_hat / z_hat_norm

    x0 = g_world - np.dot(g_world, z_hat) * z_hat
    n = np.linalg.norm(x0)
    if n < 1e-6:
        tmp = np.array([1, 0, 0]) if abs(z_hat[0]) < 0.9 else np.array([0, 1, 0])
        x0 = tmp - np.dot(tmp, z_hat) * z_hat
        x0 /= np.linalg.norm(x0)
    else:
        x0 /= n

    roll_rel = np.radians(roll_deg - roll_calib_deg)
    Rz = rotation_matrix_around_axis(z_hat, roll_rel)
    x_hat = Rz @ x0
    y_hat = np.cross(z_hat, x_hat)
    y_hat_norm = np.linalg.norm(y_hat)
    if y_hat_norm < 1e-6:
        y_hat = np.array([0, 1, 0])
    else:
        y_hat /= y_hat_norm

    return x_hat, y_hat, z_hat

def world_to_wrist_coords_using_roll(elbow, wrist, point_green, roll_deg, roll_calib_deg=0.0, g_world=np.array([0,-1,0])):
    z_hat = wrist - elbow
    norm_z = np.linalg.norm(z_hat)
    if norm_z < 1e-6:
        z_hat = np.array([0, 0, 1])
    else:
        z_hat /= norm_z

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

kinect_data = None
esp_data = None
esp_addr = None   

ROLL_CALIB_DEG = 0.0
G_WORLD = np.array([0, -1, 0])

# ======================
# Rung logic
# ======================
ON_MAX  = 0.8
ON_MIN  = 0.2
OFF_MAX = 0.2
OFF_MIN = 0.05

lastVibrate = 0.0
isVibrating = False
curOn, curOff = 0.5, 0.2
currentDir = -1
handover_start = None
handover_done = False
vibration_level = 3  # từ 1–5

# Handover detection parameters
HANDOVER_THRESHOLD_M = 0.10  # 0.1 m = 10 cm
HANDOVER_CONFIRM_S = 2.0     # changed to 2 seconds

# ======================
# GUI setup
# ======================
root = tk.Tk()
root.title("Kinect Handover Monitor")

status_label = tk.Label(root, text="Waiting...", font=("Arial", 20), width=30, height=3)
status_label.pack(pady=10)

coords_label = tk.Label(root, text="Coords: (0.00, 0.00, 0.00)", font=("Arial", 14))
coords_label.pack(pady=5)

canvas = tk.Canvas(root, width=320, height=320, bg="white")
canvas.pack(pady=10)

center_x, center_y = 160, 160
radius = 100
motor_radius = 25

motor_positions = {}
names = ["front", "front-right", "back-right", "back", "back-left", "front-left"]
for i, name in enumerate(names):
    angle = math.radians(60 * i - 30)
    x = center_x + radius * math.sin(angle)
    y = center_y - radius * math.cos(angle)
    motor_positions[name] = (x, y)

motor_ids = {}
for name, (x, y) in motor_positions.items():
    motor_ids[name] = canvas.create_oval(
        x - motor_radius, y - motor_radius,
        x + motor_radius, y + motor_radius,
        fill="white"
    )

def set_motor_visual(front, fr, br, back, bl, fl):
    canvas.itemconfig(motor_ids["front"], fill="red" if front else "white")
    canvas.itemconfig(motor_ids["front-right"], fill="red" if fr else "white")
    canvas.itemconfig(motor_ids["back-right"], fill="red" if br else "white")
    canvas.itemconfig(motor_ids["back"], fill="red" if back else "white")
    canvas.itemconfig(motor_ids["back-left"], fill="red" if bl else "white")
    canvas.itemconfig(motor_ids["front-left"], fill="red" if fl else "white")

# ======================
# Nút điều khiển độ rung
# ======================
control_frame = tk.Frame(root)
control_frame.pack(pady=10)

level_label = tk.Label(control_frame, text=f"Độ rung: {vibration_level}", font=("Arial", 14))
level_label.grid(row=0, column=1, padx=10)

def update_level(delta):
    global vibration_level
    vibration_level = max(1, min(5, vibration_level + delta))
    level_label.config(text=f"Độ rung: {vibration_level}")

tk.Button(control_frame, text="-", font=("Arial", 16), width=3, command=lambda: update_level(-1)).grid(row=0, column=0)
tk.Button(control_frame, text="+", font=("Arial", 16), width=3, command=lambda: update_level(+1)).grid(row=0, column=2)

def reject_action():
    """Stop everything permanently: called by GUI button or by ESP state signal."""
    global isVibrating, handover_done, handover_start
    isVibrating = False
    handover_done = True   # lock system so it never vibrates again
    handover_start = None
    set_motor_visual(0,0,0,0,0,0)

    status_label.config(text="🚫 Đã từ chối", bg="gray", fg="white")
    coords_label.config(text="Coords: (0.00, 0.00, 0.00)")
    print("System rejected: motors off and vibration disabled permanently.")

tk.Button(root, text="TỪ CHỐI", font=("Arial", 16), bg="red", fg="white", width=10, command=reject_action).pack(pady=5)

# ======================
# Rung mẫu
# ======================
def vibrate_pattern_front(mode):
    if mode == 1:
        pattern = [(300, 300)] * 3
    elif mode == 2:
        pattern = [(600, 300)] * 2
    elif mode == 3:
        pattern = [(200, 100), (200, 200)] * 3
    else:
        return

    def step(i=0):
        if i >= len(pattern):
            set_motor_visual(0,0,0,0,0,0)
            return
        on_ms, off_ms = pattern[i]
        set_motor_visual(1,0,0,0,0,0)
        root.after(on_ms, lambda: (
            set_motor_visual(0,0,0,0,0,0),
            root.after(off_ms, lambda: step(i+1))
        ))

    step(0)

btn_frame = tk.Frame(root)
btn_frame.pack(pady=5)
for i in range(1, 4):
    tk.Button(btn_frame, text=f"{i}", font=("Arial", 16), width=5, height=2,
              command=lambda m=i: vibrate_pattern_front(m)).grid(row=0, column=i-1, padx=5)

# ======================
# Direction logic
# ======================
def set_direction_visual(angle_deg):
    angle = angle_deg % 360
    set_motor_visual(0,0,0,0,0,0)

    front = front_right = back_right = back = back_left = front_left = 0

    if -15 <= angle_deg <= 15 or angle >= 345:
        front = 1
    elif 15 < angle <= 45:
        front = 1; front_right = 1
    elif 45 < angle <= 75:
        front_right = 1
    elif 75 < angle <= 105:
        front_right = 1; back_right = 1
    elif 105 < angle <= 135:
        back_right = 1
    elif 135 < angle <= 165:
        back_right = 1; back = 1
    elif 165 < angle <= 195:
        back = 1
    elif 195 < angle <= 225:
        back = 1; back_left = 1
    elif 225 < angle <= 255:
        back_left = 1
    elif 255 < angle <= 285:
        back_left = 1; front_left = 1
    elif 285 < angle <= 315:
        front_left = 1
    elif 315 < angle < 345:
        front_left = 1; front = 1

    set_motor_visual(front, front_right, back_right, back, back_left, front_left)

# ======================
# Update UI
# ======================
def update_status_ui(status, dist, coords=None):
    status_label.config(text=f"{status}\n(d={dist:.2f} m)")
    if "Nguy hiểm" in status:
        status_label.config(bg="red", fg="white")
    elif "giao đồ" in status:
        status_label.config(bg="yellow", fg="black")
    elif "Hoàn thành" in status or "xong" in status or "Hoàn tất" in status:
        status_label.config(bg="blue", fg="white")
    elif "Đã từ chối" in status or "TỪ CHỐI" in status:
        status_label.config(bg="gray", fg="white")
    else:
        status_label.config(bg="green", fg="white")
    if coords is not None:
        coords_label.config(text=f"Coords: ({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})")
    root.update_idletasks()

# ======================
# Main loop
# ======================
last_time = 0.0
PRINT_INTERVAL = 0.05

def tick():
    global kinect_data, esp_data, esp_addr
    global isVibrating, lastVibrate, handover_start, handover_done, last_time

    # Poll sockets (non-blocking)
    try:
        rlist, _, _ = select.select([sock_kinect, sock_esp], [], [], 0)
    except Exception as e:
        rlist = []

    for s in rlist:
        data, addr = s.recvfrom(2048)
        msg = data.decode("utf-8").strip()
        if s == sock_kinect:
            try:
                values = list(map(float, msg.split(",")))
                if len(values) == 13:
                    dist_red = values[0]
                    wrist = np.array(values[1:4])
                    elbow = np.array(values[4:7])
                    green = np.array(values[7:10])
                    red   = np.array(values[10:13])
                    kinect_data = (elbow, wrist, green, red, dist_red)
                print("Kinect raw:", msg)
            except Exception as e:
                # ignore malformed kinect packets
                print("Malformed Kinect packet:", e)
                pass
        elif s == sock_esp:
            try:
                data_json = json.loads(msg)
                # parse additional 'state' optional field
                esp_data = {
                    "pitch": data_json.get("pitch", 0),
                    "roll":  data_json.get("roll", 0),
                    "state": data_json.get("state", 0),  # <-- new optional field
                    "ts":    data_json.get("ts", 0)
                }
                esp_addr = addr
                print("ESP data:", esp_data)
            except Exception as e:
                print("Malformed ESP packet:", e)
                pass

    now = time.time()
    # only process visualization/logic at PRINT_INTERVAL
    if kinect_data and now - last_time >= PRINT_INTERVAL:
        elbow, wrist, green, red, dist_red = kinect_data
        roll = esp_data["roll"] if esp_data else 0
        p_wrist = world_to_wrist_coords_using_roll(elbow, wrist, green, roll, ROLL_CALIB_DEG, G_WORLD)
        x, y, z = p_wrist
        red_dist = float(np.linalg.norm(red - green))

        if esp_data and esp_data.get("state", 0) == 1:
            print("🚫 ESP requested reject -> reject_action()")
            reject_action()      # sẽ hiển thị từ chối, tắt motor
            last_time = now
            root.after(10, tick)
            return


        # Handover check (green relative to wrist)
        r_marker = float(np.sqrt(x*x + y*y + z*z))

        if not handover_done:
            if r_marker < HANDOVER_THRESHOLD_M:
                # If wrist is in the handover zone: vibrate all 6 motors while counting
                set_motor_visual(1,1,1,1,1,1)
                if handover_start is None:
                    handover_start = now
                    print("Handover proximity detected: starting countdown...")
                else:
                    # check if held long enough
                    if now - handover_start >= HANDOVER_CONFIRM_S:
                        handover_done = True
                        set_motor_visual(0,0,0,0,0,0)
                        print("🎉 Handover completed!")
            else:
                # not in handover zone — reset timer
                if handover_start is not None:
                    print("Handover proximity lost: reset countdown.")
                handover_start = None

        # If handover_done (or rejected), show final state and skip further ringing logic
        if handover_done:
            # If the system was set to done by reject_action(), the status text is already set there.
            # For handover completion (not reject), set the status accordingly.
            if not (esp_data and esp_data.get("state", 0) == 1):
                update_status_ui("✅ Hoàn thành", dist_red, p_wrist)
            # Make sure motors are off
            set_motor_visual(0,0,0,0,0,0)
            last_time = now
            root.after(10, tick)
            return

        # Determine proximity status to the robot (dist_red)
        # note: dist_red == 0 indicates no robot data
        if dist_red <= 0.45 and dist_red > 0:
            status = "⚠ Nguy hiểm!"
        elif dist_red <= 1.5 and dist_red > 0:
            status = "🤝 Vùng giao đồ"
        elif dist_red == 0:
            status = "Không có Robot"
        else:
            status = "✅ An toàn"

        update_status_ui(status, dist_red, p_wrist)

        # ==========================================================
        # RUNG LOGIC THEO YÊU CẦU
        # - Only when in "Vùng giao đồ"
        # - If red_dist <= 0.10 => directional on/off pattern
        # - Else => motors 1,2,6 (front, front-right, front-left) continuous
        # ==========================================================
        if status == "🤝 Vùng giao đồ":
            angle = math.degrees(math.atan2(y, x))
            if angle < 0:
                angle += 360

            # Directional vibration only if red-green very close (<=10cm)
            if red_dist <= 0.10:
                if isVibrating:
                    if now - lastVibrate >= curOn:
                        isVibrating = False
                        lastVibrate = now
                        set_motor_visual(0,0,0,0,0,0)
                else:
                    if now - lastVibrate >= curOff:
                        isVibrating = True
                        lastVibrate = now
                        set_direction_visual(angle)
            else:
                # Indicate approach by turning on front, front-right, front-left continuously
                set_motor_visual(1, 1, 0, 0, 0, 1)

            last_time = now
            root.after(10, tick)
            return
        # ==========================================================

        last_time = now

    root.after(10, tick)

# Start loop
root.after(10, tick)
root.mainloop()
