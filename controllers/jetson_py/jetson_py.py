from controller import Robot, Keyboard, Lidar, GPS, Compass
import math
from collections import deque

# -------------------------
# INITIAL SETUP
# -------------------------
TIME_STEP = 64
MAX_SPEED = 6.28        # wheel speed limit (rad/s)
OBSTACLE_LIMIT = 0.8     # meters – safe distance

robot = Robot()
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

# Devices
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
lidar = robot.getDevice("lidar")
gps.enable(TIME_STEP)
compass.enable(TIME_STEP)
lidar.enable(TIME_STEP)

left_motor = robot.getDevice("left_wheel_hinge")
right_motor = robot.getDevice("right_wheel_hinge")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Task queue for multiple destinations
task_queue = deque()

# Control parameters
KP_HEADING = 2.0
KP_DISTANCE = 1.0

# -------------------------
# HELPER FUNCTIONS
# -------------------------
def get_position():
    """Return (x,z) coordinates from GPS."""
    pos = gps.getValues()
    return pos[0], pos[2]

def get_heading():
    """Return compass heading in degrees."""
    north = compass.getValues()
    rad = math.atan2(north[0], north[2])
    deg = (rad - 1.5708) / math.pi * 180.0
    return deg

def distance_to(x_t, z_t):
    """Compute distance to target."""
    x, z = get_position()
    return math.sqrt((x_t - x)**2 + (z_t - z)**2)

def heading_error(x_t, z_t):
    """Calculate angle difference between robot heading and target direction."""
    x, z = get_position()
    robot_heading = get_heading()
    target_angle = math.degrees(math.atan2(z_t - z, x_t - x))
    err = target_angle - robot_heading
    if err > 180: err -= 360
    if err < -180: err += 360
    return err

def obstacle_ahead():
    """Return True if LiDAR detects obstacle ahead."""
    scan = lidar.getRangeImage()
    mid = scan[len(scan)//2]
    return mid < OBSTACLE_LIMIT

# -------------------------
# CORE CONTROL FUNCTIONS
# -------------------------
def move_to_target(x_t, z_t):
    """Proportional controller for navigation + avoidance."""
    err = heading_error(x_t, z_t)
    dist = distance_to(x_t, z_t)

    fwd = min(MAX_SPEED * 0.4, KP_DISTANCE * dist)
    turn = KP_HEADING * (err / 180.0)

    # Reactive avoidance: stop & turn if obstacle detected
    if obstacle_ahead():
        fwd = 0
        turn = 0.6

    left = fwd - turn
    right = fwd + turn
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))

    if dist < 0.4:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        return True
    return False

# -------------------------
# MAIN LOOP
# -------------------------
print("Mission 2 controller running. Press 'T' to add a target.")

while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()

    # Add new waypoint
    if key == ord('T'):
        x, z = get_position()
        task_queue.append((x + 1.5, z + 1.5))
        print(f"New target queued: {task_queue[-1]}")

    # If no active target
    if not task_queue:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        continue

    tx, tz = task_queue[0]
    done = move_to_target(tx, tz)

    if done:
        print(f"Reached target {tx:.2f}, {tz:.2f}")
        task_queue.popleft()
