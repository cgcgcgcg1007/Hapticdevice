import socket 
import json
import select
import numpy as np
import time
import tkinter as tk

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
# GUI setup
# ======================
root = tk.Tk()
root.title("Kinect Handover Monitor")

status_label = tk.Label(root, text="Waiting...", font=("Arial", 24), width=25, height=3)
status_label.pack(pady=10)

coords_label = tk.Label(root, text="Coords: (0.00, 0.00, 0.00)", font=("Arial", 16))
coords_label.pack(pady=5)

# Hàm update GUI
def update_status(status, dist, coords=None):
    status_label.config(text=f"{status}\n(d={dist:.2f} m)")
    if "Nguy hiểm" in status:
        status_label.config(bg="red", fg="white")
    elif "giao đồ" in status:
        status_label.config(bg="yellow", fg="black")
    else:
        status_label.config(bg="green", fg="white")

    if coords is not None:
        coords_label.config(text=f"Coords: ({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})")

    root.update_idletasks()

# Hàm gửi nhãn vật
def send_label(label_value):
    global esp_addr
    if esp_addr:  # chỉ gửi khi đã có ESP32
        msg_out = json.dumps({
            "type": "label",
            "value": label_value
        })
        sock_esp.sendto(msg_out.encode("utf-8"), esp_addr)
        print(f"Sent label to ESP32: {msg_out}")
    else:
        print("⚠ ESP32 chưa kết nối, không gửi được.")

# 3 nút nhãn vật
btn_frame = tk.Frame(root)
btn_frame.pack(pady=10)

for i in range(1, 4):
    btn = tk.Button(btn_frame, text=str(i), font=("Arial", 18),
                    width=5, height=2, 
                    command=lambda v=i: send_label(v))
    btn.grid(row=0, column=i-1, padx=5)

# ======================
# Main loop
# ======================
last_time = 0
PRINT_INTERVAL = 0.05  # giây (20Hz)

while True:
    rlist, _, _ = select.select([sock_kinect, sock_esp], [], [], 0.01)
    for s in rlist:
        data, addr = s.recvfrom(1024)
        msg = data.decode("utf-8").strip()

        if s == sock_kinect:
            try:
                values = list(map(float, msg.split(",")))
                if len(values) != 10:
                    continue

                dist_red = values[0]   
                wrist = np.array(values[1:4])
                elbow = np.array(values[4:7])
                green = np.array(values[7:10])  

                kinect_data = (elbow, wrist, green, dist_red)

            except ValueError:
                pass

        elif s == sock_esp:
            try:
                data_json = json.loads(msg)
                esp_data = {
                    "pitch": data_json.get("pitch", 0),
                    "roll": data_json.get("roll", 0),
                    "ts": data_json.get("ts", 0)
                }
                esp_addr = addr   
            except json.JSONDecodeError:
                pass

    now = time.time()
    if now - last_time >= PRINT_INTERVAL and kinect_data:
        elbow, wrist, green, dist_red = kinect_data
        roll = esp_data["roll"] if esp_data else 0  

        p_wrist = world_to_wrist_coords_using_roll(
            elbow, wrist, green,
            roll_deg=roll,
            roll_calib_deg=ROLL_CALIB_DEG,
            g_world=G_WORLD
        )

        if dist_red <= 0.45:
            status = "⚠ Nguy hiểm!"
        elif dist_red <= 1.5:
            status = "🤝 Vùng giao đồ"
        else:
            status = "✅ An toàn"

        print(f"[Transformed] Green in wrist coords: {p_wrist}")
        print(f"Status: {status} (d={dist_red:.2f} m)")
        print("-" * 40)

        update_status(status, dist_red, p_wrist)

        if esp_addr:
            msg_out = json.dumps({
                "type": "coords",
                "x": float(p_wrist[0]),
                "y": float(p_wrist[1]),
                "z": float(p_wrist[2]),
                "status": status
            })
            sock_esp.sendto(msg_out.encode("utf-8"), esp_addr)
            print(f"Sent to ESP32: {msg_out}")

        last_time = now

    root.update()
