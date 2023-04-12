import time
from Stepper_Driver_TMC2208.py import Stepper_Driver

# Create Stepper_Driver instances for each axis
x_motor = Stepper_Driver(pinStep_x, pinDir_x, pinEn_x, pulsesPerStep, stepsPerRev, accel0, accelMax, accelMin, speed0, speedMax, speedMin)
y_motor = Stepper_Driver(pinStep_y, pinDir_y, pinEn_y, pulsesPerStep, stepsPerRev, accel0, accelMax, accelMin, speed0, speedMax, speedMin)
z_motor = Stepper_Driver(pinStep_z, pinDir_z, pinEn_z, pulsesPerStep, stepsPerRev, accel0, accelMax, accelMin, speed0, speedMax, speedMin)

# Configure the motors
for motor in [x_motor, y_motor, z_motor]:
    motor.setDirection(1) # Assuming 1 for forward direction
    motor.setMovementMode(1) # Assuming 1 for your desired movement mode
    motor.setPowered(True)
    motor.setEnabled(True)

# Kinematics (assuming direct mapping between motor rev counts and position)
def position_to_rev_counts(pos):
    lead_screw_pitch = 2.0  # mm
    motor_steps_per_revolution = 200

    steps_per_mm = motor_steps_per_revolution / lead_screw_pitch

    rev_counts = [p * steps_per_mm / motor_steps_per_revolution for p in pos]
    return rev_counts

# Kinematics (assuming direct mapping between motor rev counts and position)
def rev_counts_to_position(rev_counts):
    lead_screw_pitch = 2.0  # mm
    motor_steps_per_revolution = 200

    steps_per_mm = motor_steps_per_revolution / lead_screw_pitch

    pos = [r * motor_steps_per_revolution / steps_per_mm for r in rev_counts]
    return pos

# Trajectory generation (linear interpolation)
def linear_trajectory(start, end, duration, t):
    return [
        start[i] + (end[i] - start[i]) * (t / duration)
        for i in range(len(start))
    ]

# Define start and end points
start_point = [0, 0, 0]
end_point = [100, 50, 20]
duration = 5.0

# Control loop
start_time = time.time()
while True:
    current_time = time.time() - start_time
    if current_time > duration:
        break

    # Calculate the desired position using trajectory generation
    desired_position = linear_trajectory(start_point, end_point, duration, current_time)

    # Convert desired_position to motor revolution counts
    desired_rev_counts = position_to_rev_counts(desired_position)

    # Send the desired motor revolution counts to the motors
    x_motor.runToRevCount(desired_rev_counts[0])
    y_motor.runToRevCount(desired_rev_counts[1])
    z_motor.runToRevCount(desired_rev_counts[2])

    # Sleep for a short duration to avoid overloading the control system
    time.sleep(0.01)
