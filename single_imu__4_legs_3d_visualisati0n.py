import smbus2
import time
import math
import sys
import select
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ================= MPU SETUP =================
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus2.SMBus(1)
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

# ================= CALIBRATION OFFSETS =================
ACCEL_BIAS = [-0.0287, -0.0047, -0.0051]
GYRO_BIAS  = [-5.1417, 1.4964, 0.2930]

# ================= LIMITS =================
ANGLE_STABLE_BAND = 3.0      # degrees
ANGLE_MAX = 30.0             # degrees → 10/10

GYRO_STABLE_BAND = 0.5       # deg/s
GYRO_MAX = {
    'x_pos': 115.40,
    'x_neg': -85.20,
    'y_pos': 111.82,
    'y_neg': -131.28
}

# ================= LOW LEVEL IO =================
def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val

def get_accel():
    ax = read_word(ACCEL_XOUT_H) / 16384.0 - ACCEL_BIAS[0]
    ay = read_word(ACCEL_XOUT_H + 2) / 16384.0 - ACCEL_BIAS[1]
    az = read_word(ACCEL_XOUT_H + 4) / 16384.0 - ACCEL_BIAS[2]
    return ax, ay, az

def get_gyro():
    gx = read_word(GYRO_XOUT_H) / 131.0 - GYRO_BIAS[0]
    gy = read_word(GYRO_XOUT_H + 2) / 131.0 - GYRO_BIAS[1]
    gz = read_word(GYRO_XOUT_H + 4) / 131.0 - GYRO_BIAS[2]
    return gx, gy, gz

# ================= ORIENTATION =================
def get_roll_pitch(ax, ay, az):
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    return roll, pitch

# ================= ROTATION =================
def rotation_matrix(roll, pitch):
    r = math.radians(roll)
    p = math.radians(-pitch)  # IMPORTANT: pitch sign correction

    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(r), -math.sin(r)],
        [0, math.sin(r),  math.cos(r)]
    ])

    Ry = np.array([
        [ math.cos(p), 0, math.sin(p)],
        [0, 1, 0],
        [-math.sin(p), 0, math.cos(p)]
    ])

    return Ry @ Rx


# ================= SCALING =================
def angle_to_level(angle):
    if abs(angle) <= ANGLE_STABLE_BAND:
        return 0
    lvl = (abs(angle) - ANGLE_STABLE_BAND) / (ANGLE_MAX - ANGLE_STABLE_BAND)
    return int(max(0.0, min(lvl, 1.0)) * 10)

def gyro_to_level(g, g_ref, g_pos_max, g_neg_max):
    diff = g - g_ref
    if abs(diff) <= GYRO_STABLE_BAND:
        return 0
    if diff > 0:
        if diff >= g_pos_max:
            return 10
        lvl = (diff - GYRO_STABLE_BAND) / (g_pos_max - GYRO_STABLE_BAND)
    else:
        if diff <= g_neg_max:
            return 10
        lvl = (abs(diff) - GYRO_STABLE_BAND) / (abs(g_neg_max) - GYRO_STABLE_BAND)
    return int(max(0.0, min(lvl, 1.0)) * 10)

# ================= COLOR =================
def intensity_color(level):
    if level <= 2:
        return "green"
    elif level <= 5:
        return "yellow"
    elif level <= 8:
        return "orange"
    else:
        return "red"

# ================= FRONT ARROW =================
def draw_front_arrow(ax):
    ax.quiver(
        0, 0, 0,      # origin
        1.5, 0, 0,    # +X is FRONT
        color='red',
        linewidth=2,
        arrow_length_ratio=0.15
    )
    ax.text(1.6, 0, 0, "FRONT", color='red', weight='bold')

# ================= REFERENCE =================
def take_reference(samples=600):
    print("\nKeep IMU stable... Taking reference")
    sgx = sgy = sr = sp = 0.0

    for _ in range(samples):
        gx, gy, _ = get_gyro()
        ax, ay, az = get_accel()
        roll, pitch = get_roll_pitch(ax, ay, az)

        sgx += gx
        sgy += gy
        sr += roll
        sp += pitch
        time.sleep(0.005)

    print("Reference captured\n")
    return sgx/samples, sgy/samples, sr/samples, sp/samples

# ================= POSTURE =================
def posture_status(gx, gy, roll, pitch,
                   ref_gx, ref_gy, ref_roll, ref_pitch):

    roll_angle = roll - ref_roll
    pitch_angle = pitch - ref_pitch

    roll_lvl = angle_to_level(roll_angle)
    pitch_lvl = angle_to_level(pitch_angle)

    roll_rate = gyro_to_level(gx, ref_gx, GYRO_MAX['x_pos'], GYRO_MAX['x_neg'])
    pitch_rate = gyro_to_level(gy, ref_gy, GYRO_MAX['y_pos'], GYRO_MAX['y_neg'])

    intensity = max(roll_lvl, pitch_lvl, roll_rate, pitch_rate)

    status = "Stable → All legs neutral (0/10)"

    if roll_lvl == 0 and pitch_lvl == 0:
        return intensity, roll_angle, pitch_angle, status

    if roll_angle < 0 and abs(pitch_angle) < ANGLE_STABLE_BAND:
        status = f"Right tilt → A,D extend | B,C retract ({intensity}/10)"
    elif roll_angle > 0 and abs(pitch_angle) < ANGLE_STABLE_BAND:
        status = f"Left tilt → B,C extend | A,D retract ({intensity}/10)"
    elif pitch_angle < 0 and abs(roll_angle) < ANGLE_STABLE_BAND:
        status = f"Forward tilt → C,D extend | A,B retract ({intensity}/10)"
    elif pitch_angle > 0 and abs(roll_angle) < ANGLE_STABLE_BAND:
        status = f"Backward tilt → A,B extend | C,D retract ({intensity}/10)"
    elif roll_angle < 0 and pitch_angle < 0:
        status = f"Front-Right diagonal → All extend except B ({intensity}/10)"
    elif roll_angle > 0 and pitch_angle < 0:
        status = f"Front-Left diagonal → All extend except A ({intensity}/10)"
    elif roll_angle < 0 and pitch_angle > 0:
        status = f"Rear-Right diagonal → All extend except C ({intensity}/10)"
    elif roll_angle > 0 and pitch_angle > 0:
        status = f"Rear-Left diagonal → All extend except D ({intensity}/10)"

    return intensity, roll_angle, pitch_angle, status

# ================= BODY MODEL =================
BODY = np.array([
    [ 1, -0.6, 0],   # Front Right
    [ 1,  0.6, 0],   # Front Left
    [-1,  0.6, 0],   # Rear Left
    [-1, -0.6, 0],   # Rear Right
    [ 1, -0.6, 0]
])



# ================= MAIN =================
ref_gx = ref_gy = ref_roll = ref_pitch = None

plt.ion()
fig = plt.figure(figsize=(7,7))
ax = fig.add_subplot(111, projection='3d')

print("\nControls:")
print("R → Take reference")
print("S → Start sensing")
print("Q → Stop sensing")
print("X → Exit\n")

while True:
    cmd = input("Enter command: ").strip().upper()

    if cmd == "R":
        ref_gx, ref_gy, ref_roll, ref_pitch = take_reference()

    elif cmd == "S":
        if ref_gx is None:
            print("Take reference first (R)")
            continue

        print("\nSensing started (Q to stop)\n")

        while True:
            gx, gy, _ = get_gyro()
            axx, ayy, azz = get_accel()
            roll, pitch = get_roll_pitch(axx, ayy, azz)

            level, roll_rel, pitch_rel, status = posture_status(
                gx, gy, roll, pitch,
                ref_gx, ref_gy, ref_roll, ref_pitch
            )

            Rm = rotation_matrix(roll_rel, pitch_rel)
            body_rot = BODY @ Rm.T
            color = intensity_color(level)

            ax.cla()
            ax.plot(body_rot[:,0], body_rot[:,1], body_rot[:,2],
                    linewidth=4, color=color)
            ax.scatter(0,0,0, color='black')

            draw_front_arrow(ax)

            ax.set_xlim(-2,2)
            ax.set_ylim(-2,2)
            ax.set_zlim(-2,2)

            ax.text2D(0.05, 0.95, status,
                      transform=ax.transAxes,
                      fontsize=11, weight='bold')

            ax.set_title(
                f"Roll: {roll_rel:.1f}° | Pitch: {pitch_rel:.1f}° | Intensity: {level}/10"
            )

            plt.pause(0.01)
            time.sleep(0.05)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                if input().strip().upper() == "Q":
                    print("\nSensing stopped\n")
                    break

    elif cmd == "X":
        print("Exiting program")
        break

