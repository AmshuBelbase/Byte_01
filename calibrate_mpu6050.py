import smbus2
import time

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus2.SMBus(1)
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val

def get_accel():
    ax = read_word(ACCEL_XOUT_H) / 16384.0
    ay = read_word(ACCEL_XOUT_H + 2) / 16384.0
    az = read_word(ACCEL_XOUT_H + 4) / 16384.0
    return ax, ay, az

def get_gyro():
    gx = read_word(GYRO_XOUT_H) / 131.0
    gy = read_word(GYRO_XOUT_H + 2) / 131.0
    gz = read_word(GYRO_XOUT_H + 4) / 131.0
    return gx, gy, gz

# ------------------ CALIBRATION ------------------

samples = 1000
accel_bias = [0.0, 0.0, 0.0]
gyro_bias = [0.0, 0.0, 0.0]

print("Calibrating... DO NOT MOVE THE SENSOR")
time.sleep(2)

for i in range(samples):
    ax, ay, az = get_accel()
    gx, gy, gz = get_gyro()

    accel_bias[0] += ax
    accel_bias[1] += ay
    accel_bias[2] += az - 1.0  # gravity compensation

    gyro_bias[0] += gx
    gyro_bias[1] += gy
    gyro_bias[2] += gz

    time.sleep(0.002)

accel_bias = [x / samples for x in accel_bias]
gyro_bias = [x / samples for x in gyro_bias]

print("\n=== CALIBRATION OFFSETS ===")
print(f"Accel Bias (g): X={accel_bias[0]:.4f} Y={accel_bias[1]:.4f} Z={accel_bias[2]:.4f}")
print(f"Gyro Bias (°/s): X={gyro_bias[0]:.4f} Y={gyro_bias[1]:.4f} Z={gyro_bias[2]:.4f}")
