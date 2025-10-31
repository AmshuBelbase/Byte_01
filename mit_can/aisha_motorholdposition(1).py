# filename: run_servo_mode.py

from TMotorCANControl.mit_can import CAN_Manager_servo
import time

# Replace this with your motor's CAN ID
MOTOR_ID = 12 

# Target parameters
TARGET_ANGLE = 90.0     # degrees
MAX_SPEED = 5000        # in electrical RPM (ERPM)
ACCELERATION = 1000     # ramp acceleration

def main():
    print("🔧 Initializing CAN and Motor...")
    # 1️⃣ Initialize CAN bus and manager
    can_manager = CAN_Manager_servo()

    # 2️⃣ Optional: Set current position as zero
    can_manager.comm_can_set_origin(MOTOR_ID)
    print(f"✅ Motor {MOTOR_ID} origin set to 0°")

    time.sleep(0.5)

    # 3️⃣ Command the motor to move to the target angle in servo mode
    print(f"➡️ Moving motor {MOTOR_ID} to {TARGET_ANGLE}° ...")
    can_manager.comm_can_set_pos_spd(
        controller_id=MOTOR_ID,
        pos=TARGET_ANGLE,
        spd=MAX_SPEED,
        RPA=ACCELERATION
    )

    # Wait for the motor to reach
    time.sleep(2)

    # 4️⃣ Move the motor back to origin
    print(f"⬅️ Returning motor {MOTOR_ID} to 0° ...")
    can_manager.comm_can_set_pos_spd(
        controller_id=MOTOR_ID,
        pos=0.0,
        spd=MAX_SPEED,
        RPA=ACCELERATION
    )

    time.sleep(2)
    print("✅ Servo mode test complete.")

if __name__ == "__main__":
    main()
