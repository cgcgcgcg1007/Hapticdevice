import tkinter as tk
import socket
import json
import time
import random
import csv
import os

# ======================
# ESP32 CONFIG
# ======================
ESP32_IP = "172.20.10.3"
ESP32_PORT = 23456
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_to_esp(motor_id):
    data = {
        "mode": 1,
        "motors": [1 if i == motor_id else 0 for i in range(6)]
    }
    sock.sendto(json.dumps(data).encode(), (ESP32_IP, ESP32_PORT))

# ======================
# CSV
# ======================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(BASE_DIR, "result.csv")

if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, "w", newline="") as f:
        csv.writer(f).writerow(["trial", "t1", "t2", "t3", "t4", "t5", "t6"])

print("CSV file:", CSV_FILE)

# ======================
# CONSTANTS
# ======================
TOTAL_TIME = 60.0
NUM_SLOTS = 6
SLOT_TIME = 10.0
START_DELAY = 5.0
TIMEOUT_VALUE = 10.000

# ======================
# GUI
# ======================
root = tk.Tk()
root.title("Reaction Test (Manual + 1 Minute)")

status_label = tk.Label(root, text="READY", font=("Arial", 16))
status_label.pack(pady=5)

reaction_label = tk.Label(root, text="0.000 s", font=("Arial", 20))
reaction_label.pack()

test_timer_label = tk.Label(root, text="", font=("Arial", 14))
test_timer_label.pack(pady=5)

# ======================
# MOTOR BUTTONS
# ======================
frame = tk.Frame(root)
frame.pack(pady=5)

buttons = []

def set_visual(idx):
    for i, b in enumerate(buttons):
        b.config(bg="red" if i == idx else "white")

def reset_visual():
    for b in buttons:
        b.config(bg="white")

# ======================
# STATE
# ======================
manual_mode = False
test_mode = False

trial_count = 1
slot_index = 0
reaction_times = []

reaction_start_time = None
slot_start_time = None
slot_timeout_id = None
test_start_time = None

# ======================
# TIMERS
# ======================
def update_reaction_timer():
    if reaction_start_time is None:
        return
    elapsed = time.time() - reaction_start_time
    reaction_label.config(text=f"{elapsed:.3f} s")
    root.after(10, update_reaction_timer)

def update_test_timer():
    if not test_mode:
        return
    elapsed = time.time() - test_start_time
    remain = max(0, TOTAL_TIME - elapsed)
    test_timer_label.config(text=f"⏱ {remain:.1f} s")
    root.after(100, update_test_timer)

# ======================
# MANUAL MODE
# ======================
def manual_trigger(idx):
    global manual_mode, reaction_start_time
    if test_mode:
        return
    manual_mode = True
    set_visual(idx)
    send_to_esp(idx)
    reaction_start_time = time.time()
    status_label.config(text="MANUAL MODE")
    update_reaction_timer()

# ======================
# TEST MODE (1 MIN)
# ======================
def start_reaction(motor_id):
    global reaction_start_time
    set_visual(motor_id)
    send_to_esp(motor_id)
    reaction_start_time = time.time()
    update_reaction_timer()

def start_slot():
    global slot_start_time, slot_timeout_id
    if slot_index >= NUM_SLOTS:
        finish_test()
        return
    slot_start_time = time.time()
    motor = random.randint(0, 5)
    start_reaction(motor)
    slot_timeout_id = root.after(int(SLOT_TIME * 1000), slot_timeout)

def slot_timeout():
    global reaction_start_time
    if reaction_start_time is not None:
        reaction_times.append(TIMEOUT_VALUE)
        reaction_start_time = None
    wait_next_slot()

def wait_next_slot():
    global slot_index
    reset_visual()
    slot_index += 1
    delay = max(0, SLOT_TIME - (time.time() - slot_start_time))
    root.after(int(delay * 1000), start_slot)

# ======================
# KEY EVENT
# ======================
def on_key(event):
    global reaction_start_time
    if event.keysym == "space" and reaction_start_time is not None:
        elapsed = time.time() - reaction_start_time
        reaction_label.config(text=f"{elapsed:.3f} s")
        reaction_start_time = None

        if test_mode:
            reaction_times.append(round(elapsed, 3))
            root.after_cancel(slot_timeout_id)
            wait_next_slot()
        else:
            reset_visual()
            status_label.config(text="MANUAL DONE")

root.bind("<KeyPress>", on_key)
root.focus_set()

# ======================
# START / RESET (1 MIN)
# ======================
def start_test():
    global test_mode, manual_mode
    global slot_index, reaction_times, test_start_time

    if test_mode:
        return

    manual_mode = False
    test_mode = True
    slot_index = 0
    reaction_times = []
    test_start_time = time.time()

    status_label.config(text="GET READY (5s)")
    test_timer_label.config(text="⏱ 60.0 s")
    update_test_timer()

    root.after(int(START_DELAY * 1000), start_slot)

# ======================
# FINISH
# ======================
def finish_test():
    global test_mode, trial_count
    test_mode = False

    with open(CSV_FILE, "a", newline="") as f:
        csv.writer(f).writerow([trial_count] + reaction_times)

    trial_count += 1
    reset_visual()
    test_timer_label.config(text="")
    status_label.config(text="TEST DONE")

# ======================
# BUTTONS
# ======================
for i in range(6):
    btn = tk.Button(
        frame, text=str(i+1),
        font=("Arial", 16), width=5, height=2,
        command=lambda i=i: manual_trigger(i)
    )
    btn.grid(row=0, column=i, padx=5)
    buttons.append(btn)

tk.Button(
    root,
    text="▶ START 1 MINUTE TEST",
    font=("Arial", 14),
    width=25,
    command=start_test
).pack(pady=10)

root.mainloop()
