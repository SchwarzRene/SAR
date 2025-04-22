import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Configuration ---
SERVO_FREQUENCY = 50  # Standard frequency for most analog servos (Hz)
NUM_SERVOS = 16       # PCA9685 has 16 channels (0-15)

# --- !! PRECISION TUNING !! ---
# These values are CRITICAL for accuracy. You MUST adjust them for YOUR servos.
# Find these by experimentation:
# - Gradually increase pulse width from ~500 until the servo stops moving at one end (min_pulse).
# - Gradually decrease pulse width from ~2500 until the servo stops moving at the other end (max_pulse).
# - The actuation_range should correspond to the angle range covered by min/max pulse (usually 180).
SERVO_MIN_PULSE = 500   # Default: 500. Adjust this value for your specific servos!
SERVO_MAX_PULSE = 2500  # Default: 2500. Adjust this value for your specific servos!
SERVO_ACTUATION_RANGE = 180 # Angle range (usually 180 degrees) covered by min/max pulse

# --- SMOOTH MOVEMENT PARAMETERS ---
INITIAL_ANGLE = 90      # Angle to set initially
SWEEP_START_ANGLE = 60  # Example: Start angle for a smooth sweep
SWEEP_END_ANGLE = 120   # Example: End angle for a smooth sweep
SWEEP_STEP = 1          # Angle increment for smooth movement (smaller = smoother, slower)
SWEEP_DELAY = 0.015     # Delay between steps in seconds (larger = slower)

# --- Initialization ---
print("Initializing I2C...")
try:
    i2c = busio.I2C(board.SCL, board.SDA)
except ValueError as e:
    print(f"Error initializing I2C: {e}")
    print("Ensure I2C is enabled in raspi-config and wiring is correct.")
    exit()

print("Initializing PCA9685...")
try:
    pca = PCA9685(i2c)
    pca.frequency = SERVO_FREQUENCY
    print(f"PCA9685 found and frequency set to {SERVO_FREQUENCY} Hz.")
except OSError as e:
     print(f"Error initializing PCA9685: {e}")
     print("Could not find PCA9685 at default address 0x40.")
     print("Check wiring, power to PCA9685, and I2C address.")
     exit()
except ValueError as e:
     print(f"Error initializing PCA9685: {e}")
     print("Check wiring and ensure the PCA9685 is powered.")
     exit()

# --- Create Servo Objects with Calibration ---
servos = []
print(f"Creating {NUM_SERVOS} servo objects with specified pulse range...")
for i in range(NUM_SERVOS):
    try:
        # Pass the calibrated pulse width and range to the Servo constructor
        servo_obj = servo.Servo(
            pca.channels[i],
            min_pulse=SERVO_MIN_PULSE,
            max_pulse=SERVO_MAX_PULSE,
            actuation_range=SERVO_ACTUATION_RANGE
        )
        servos.append(servo_obj)
        # print(f"  Servo object created for channel {i}") # Optional: uncomment for verbose output
    except Exception as e:
        print(f"Error creating servo object for channel {i}: {e}")
        # Decide if you want to exit or just skip this servo
        # exit() # Uncomment to stop if one servo fails
        servos.append(None) # Add a placeholder if you want to continue

# Filter out any servos that failed to initialize
active_servos = [s for s in servos if s is not None]
if not active_servos:
    print("No servos were initialized successfully. Exiting.")
    exit()

print(f"\nSuccessfully initialized {len(active_servos)} servos.")

# --- Function for Smooth Movement ---
def move_servos_smoothly(target_angle, current_angles, step=1, delay=0.01):
    """Moves all active servos towards the target angle incrementally."""
    print(f"Moving servos towards {target_angle}°...")
    max_angle = SERVO_ACTUATION_RANGE # Use the defined range
    min_angle = 0

    # Clamp target angle to valid range
    target_angle = max(min_angle, min(max_angle, target_angle))

    # Determine the maximum change needed for any servo
    max_delta = 0
    for i, s in enumerate(active_servos):
        if current_angles[i] is not None:
             max_delta = max(max_delta, abs(target_angle - current_angles[i]))

    # Calculate number of steps needed based on the largest movement
    num_steps = int(max_delta / step)
    if num_steps == 0:
        num_steps = 1 # Ensure at least one step if already at target or very close

    # Calculate the increment for each servo for each step
    increments = []
    for i, s in enumerate(active_servos):
         if current_angles[i] is not None:
             increments.append((target_angle - current_angles[i]) / num_steps)
         else:
             increments.append(0) # Should not happen if filtering worked


    # Perform the incremental movement
    for step_num in range(num_steps):
        for i, s in enumerate(active_servos):
             if current_angles[i] is not None:
                new_angle = current_angles[i] + increments[i]
                # Clamp intermediate angle to valid range
                s.angle = max(min_angle, min(max_angle, new_angle))
                current_angles[i] = new_angle # Update current angle position tracker

        time.sleep(delay)

    # Final adjustment to ensure all servos reach the exact target angle
    # (compensates for potential floating point inaccuracies)
    print(f"Fine-tuning servos to exactly {target_angle}°...")
    for i, s in enumerate(active_servos):
         if current_angles[i] is not None:
            s.angle = target_angle
            current_angles[i] = target_angle
    print("Movement complete.")
    return current_angles # Return the updated list of current angles


# --- Main Execution ---
try:
    # Initialize servo angles (can be instant or smooth)
    print(f"\nSetting initial position to {INITIAL_ANGLE}°...")
    current_angles = [None] * len(active_servos) # Track current angle for smooth movement
    for i, s in enumerate(active_servos):
        s.angle = INITIAL_ANGLE
        current_angles[i] = INITIAL_ANGLE
    time.sleep(1) # Give servos time to reach initial position

    # --- Example: Smooth Sweep ---
    print("\nStarting smooth sweep example...")
    while True: # Loop forever (or change to run once)
        current_angles = move_servos_smoothly(SWEEP_END_ANGLE, current_angles, step=SWEEP_STEP, delay=SWEEP_DELAY)
        time.sleep(0.5) # Pause at end angle

        current_angles = move_servos_smoothly(SWEEP_START_ANGLE, current_angles, step=SWEEP_STEP, delay=SWEEP_DELAY)
        time.sleep(0.5) # Pause at start angle

except KeyboardInterrupt:
    print("\nCtrl+C detected. Stopping servos.")
except Exception as e:
    print(f"\nAn error occurred during execution: {e}")
finally:
    # --- Optional: Release PCA9685 resources ---
    # Uncomment below if you want servos to go limp when script stops.
    # print("\nDeinitializing PCA9685...")
    # pca.deinit()
    # print("PCA9685 deinitialized. PWM signals stopped.")
    print("Script finished.")