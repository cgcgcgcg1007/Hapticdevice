import socket
import json
import select
import numpy as np
import time
import tkinter as tk
import math
# ======================
# Task timing & recognition
# ======================
task_start_time = None
current_task = None   # 1, 2, 3
task_done = False
current_condition = "Điều kiện A: Vòng Thesis không có nút"
tray_open = False
force_directional = False   # ép coi đỏ = xanh

# ======================
# 3-minute timer
# ======================
timer_running = False
timer_start_time = None
TIMER_DURATION = 180  # 3 phút (giây)

ESP32_IP = "172.20.10.3"   # đổi đúng IP ESP32 của bạn
ESP32_PORT = 23456         # port ESP32 lắng nghe

sock_esp_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_vibration_to_esp(mode, motors):
    """
    mode: 1 = rung 3 động cơ
          2 = rung theo hướng
    motors: list 6 phần tử [0/1]
    """
    data = {
        "mode": mode,
        "motors": motors
    }
    try:
        sock_esp_tx.sendto(
            json.dumps(data).encode(),
            (ESP32_IP, ESP32_PORT)
        )
    except Exception as e:
        print("ESP send error:", e)

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
gyro_label = tk.Label(
    root,
    text="GYRO: OFF",
    font=("Arial", 14, "bold"),
    fg="red"
)
gyro_label.pack(pady=(0, 10))

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
def toggle_force_directional():
    global force_directional
    force_directional = not force_directional

    if force_directional:
        force_btn.config(
            text="🧭 HƯỚNG: ÉP BẬT",
            bg="green",
            fg="white"
        )
    else:
        force_btn.config(
            text="🧭 HƯỚNG: TỰ ĐỘNG",
            bg="gray",
            fg="white"
        )

force_btn = tk.Button(
    root,
    text="🧭 HƯỚNG: TỰ ĐỘNG",
    font=("Arial", 14),
    width=25,
    bg="gray",
    fg="white",
    command=toggle_force_directional
)
force_btn.pack(pady=5)

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
    global isVibrating, handover_done, handover_start
    global force_directional, timer_running, timer_start_time

    isVibrating = False
    handover_done = True
    handover_start = None

    force_directional = False
    force_btn.config(text="🧭 HƯỚNG: TỰ ĐỘNG", bg="gray", fg="white")

    set_motor_visual(0,0,0,0,0,0)
    status_label.config(text="🚫 Đã từ chối", bg="gray", fg="white")
    coords_label.config(text="Coords: (0.00, 0.00, 0.00)")

    timer_running = False
    timer_start_time = None
    timer_label.config(text="⏱ 00:00 / 03:00")
    timer_button.config(text="▶ Bắt đầu đếm thời gian")

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
    btn = tk.Button(
        btn_frame,
        text=f"{i}",
        font=("Arial", 16),
        width=5,
        height=2,
        command=lambda m=i: (vibrate_pattern_front(m), start_task(m))
    )
    btn.grid(row=0, column=i-1, padx=5)

# ======================
# Task result display (below buttons 1-2-3)
# ======================
task_label = tk.Label(
    root,
    text="Chưa có thao tác",
    font=("Arial", 14),
    width=30,
    height=2,
    bg="white",
    fg="black",
    relief="solid"
)
task_label.pack(pady=5)
# ======================
# Condition dropdown tray
# ======================
tray_button = tk.Button(
    root,
    text=current_condition + "  ▼",
    font=("Arial", 13),
    width=40,
    relief="raised",
    command=lambda: toggle_tray()
)
tray_button.pack(pady=(5, 0))
tray_frame = tk.Frame(root, bd=1, relief="solid")
# KHÔNG pack ở đây → mặc định ẩn
conditions = [
    "Điều kiện A: Vòng Thesis không có nút",
    "Điều kiện B: Vòng đầy đủ chức năng",
    "Điều kiện C: Không có vòng"
]
# ======================
# Timer display
# ======================
timer_label = tk.Label(
    root,
    text="⏱ 00:00 / 03:00",
    font=("Arial", 14),
    width=30,
    height=2,
    bg="#222222",
    fg="white",
    relief="solid"
)
timer_label.pack(pady=(5, 5))
timer_button = tk.Button(
    root,
    text="▶ Bắt đầu đếm thời gian",
    font=("Arial", 14),
    width=25,
    bg="#333333",
    fg="white",
    activebackground="#555555",
    command=lambda: start_timer()
)
timer_button.pack(pady=(0, 10))


for c in conditions:
    tk.Button(
        tray_frame,
        text=c,
        anchor="w",
        font=("Arial", 12),
        width=38,
        relief="flat",
        command=lambda t=c: select_condition(t)
    ).pack(fill="x", padx=5, pady=2)


# ======================
# Direction logic
# ======================
def gyro_enabled():
    """Chỉ cho phép gyro khi ở Điều kiện B"""
    return current_condition == "Điều kiện B: Vòng đầy đủ chức năng"
def update_gyro_label():
    if gyro_enabled():
        gyro_label.config(text="GYRO: ON", fg="green")
    else:
        gyro_label.config(text="GYRO: OFF", fg="red")

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
    return [front, front_right, back_right, back, back_left, front_left]
def toggle_tray():
    global tray_open

    if tray_open:
        tray_frame.place_forget()
        tray_open = False
        tray_button.config(text=current_condition + "  ▼")
    else:
        # Lấy vị trí nút tray_button
        x = tray_button.winfo_x()
        y = tray_button.winfo_y() + tray_button.winfo_height()

        tray_frame.place(x=x, y=y)
        tray_frame.lift()   # luôn nổi trên cùng

        tray_open = True
        tray_button.config(text=current_condition + "  ▲")

def select_condition(text):
    global current_condition, tray_open, esp_data, force_directional
    current_condition = text

    # Reset gyro khi đổi điều kiện
    if not gyro_enabled():
        esp_data = {
            "pitch": 0.0,
            "roll":  0.0,
            "state": 0,
            "ts": 0
        }

    # Reset ép hướng khi đổi điều kiện
    force_directional = False
    force_btn.config(text="🧭 HƯỚNG: TỰ ĐỘNG", bg="gray", fg="white")

    tray_button.config(text=current_condition + "  ▼")
    tray_frame.place_forget()
    tray_open = False
    update_gyro_label()


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
def start_timer():
    global timer_running, timer_start_time

    # Nếu đang chạy HOẶC đã chạy xong → RESET
    if timer_running or timer_start_time is not None:
        timer_running = False
        timer_start_time = None

        timer_label.config(text="⏱ 00:00 / 03:00")
        timer_button.config(text="▶ Bắt đầu đếm thời gian")
        return

    # Nếu đang ở 00:00 → START
    timer_running = True
    timer_start_time = time.time()
    timer_button.config(text="🔄 Reset")
    update_timer()

def update_timer():
    global timer_running

    if not timer_running:
        return

    elapsed = time.time() - timer_start_time

    if elapsed >= TIMER_DURATION:
        elapsed = TIMER_DURATION
        timer_running = False
        timer_button.config(text="🔄 Reset")

    minutes = int(elapsed // 60)
    seconds = int(elapsed % 60)

    timer_label.config(
        text=f"⏱ {minutes:02d}:{seconds:02d} / 03:00"
    )

    if timer_running:
        root.after(500, update_timer)  # cập nhật mỗi 0.5s

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

                if gyro_enabled():
                    # Điều kiện B → nhận gyro thật
                    esp_data = {
                        "pitch": data_json.get("pitch", 0),
                        "roll":  data_json.get("roll", 0),
                        "state": data_json.get("state", 0),
                        "ts":    data_json.get("ts", 0)
                    }
                else:
                    # Điều kiện A, C → gyro luôn 0
                    esp_data = {
                        "pitch": 0.0,
                        "roll":  0.0,
                        "state": data_json.get("state", 0),
                        "ts":    data_json.get("ts", 0)
                    }

                esp_addr = addr
                print("ESP data:", esp_data)

            except Exception as e:
                print("Malformed ESP packet:", e)


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
        if status == "🤝 Vùng giao đồ" or force_directional:
            angle = math.degrees(math.atan2(y, x))
            if angle < 0:
                angle += 360

            if red_dist <= 0.10 or force_directional:
                if isVibrating:
                    if now - lastVibrate >= curOn:
                        isVibrating = False
                        lastVibrate = now
                        set_motor_visual(0,0,0,0,0,0)
                        send_vibration_to_esp(2, [0,0,0,0,0,0])

                else:
                    if now - lastVibrate >= curOff:
                        isVibrating = True
                        lastVibrate = now

                        motors = set_direction_visual(angle)
                        send_vibration_to_esp(2, motors)

            else:
                set_motor_visual(1,1,0,0,0,1)
                send_vibration_to_esp(1, [1,1,0,0,0,1])

            last_time = now
            root.after(10, tick)
            return

        # ==========================================================

        last_time = now

    root.after(10, tick)
def start_task(task_id):
    global task_start_time, current_task, task_done
    task_start_time = time.time()
    current_task = task_id
    task_done = False

    task_label.config(
        text=f"🟡 Bắt đầu đồ {task_id}\nĐang tính thời gian...",
        bg="orange",
        fg="black"
    )

def finish_task(direction):
    global task_start_time, current_task, task_done

    if task_start_time is None or task_done:
        return

    elapsed = time.time() - task_start_time
    task_done = True

    correct = False
    if current_task in (1, 2) and direction == "down":
        correct = True
    elif current_task == 3 and direction == "right":
        correct = True

    if correct:
        task_label.config(
            text=f"✅ ĐÚNG\n⏱ {elapsed:.2f} s",
            bg="lightgreen",
            fg="black"
        )
    else:
        task_label.config(
            text=f"❌ NHẬN BIẾT NHẦM\n⏱ {elapsed:.2f} s",
            bg="red",
            fg="white"
        )

    task_start_time = None
    current_task = None

def on_key_press(event):
    if event.keysym == "1":
        start_task(1)
    elif event.keysym == "2":
        start_task(2)
    elif event.keysym == "3":
        start_task(3)
    elif event.keysym == "Down":
        finish_task("down")
    elif event.keysym == "Right":
        finish_task("right")

root.bind("<KeyPress>", on_key_press)
root.focus_set()

# Start loop
root.after(10, tick)
root.mainloop()
