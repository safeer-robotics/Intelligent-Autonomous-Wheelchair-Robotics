from controller import Robot, Keyboard
import math, time, os, csv

# ============================================================
# Smart Wheelchair (Webots / Pioneer3dx) Controller
# Modes:
#   - MANUAL: user control (WASD/arrows) + safety slow/stop only (no auto steering)
#   - AUTO: goal-seeking + reactive ultrasonic obstacle avoidance + stuck recovery
#
# Keys:
#   M = Manual mode
#   G = Auto mode (go to goal)
#   Space = Emergency stop (both modes)
# ============================================================

TIME_STEP = 32
MAX_WHEEL_SPEED = 5.24  # Pioneer3dx typical max wheel rad/s

# -----------------------------
# Goal in the world (goal marker)
# -----------------------------
GOAL_X, GOAL_Y = 4.0, 0.0
GOAL_TOL = 0.35

# -----------------------------
# AUTO thresholds (meters)
# -----------------------------
FRONT_BLOCK = 0.80      # below this -> start avoiding
FRONT_CLEAR = 1.10      # above this -> resume goal seeking
E_STOP_DIST = 0.18      # emergency stop in AUTO only

# -----------------------------
# AUTO tuning
# -----------------------------
AUTO_FWD = 0.85
AUTO_TURN_GAIN = 1.4
AVOID_FWD = 0.55
AVOID_TURN = 0.85
MIN_AVOID_TIME = 0.8

# -----------------------------
# AUTO stuck recovery
# -----------------------------
STUCK_TIME = 1.8
REC_BACK_TIME = 0.55
REC_TURN_TIME = 0.75

# -----------------------------
# MANUAL improvements
# -----------------------------
MANUAL_FWD = 0.9
MANUAL_REV = 0.7
MANUAL_TURN = 0.9
KEY_HOLD_SEC = 0.18  # allows move+turn together (Webots keyboard events)

# Manual safety (slow/stop only; no auto steering)
MANUAL_STOP_FRONT = 0.22
MANUAL_SLOW_FRONT = 0.90
MANUAL_STOP_SIDE  = 0.16
MANUAL_SLOW_SIDE  = 0.35
MANUAL_STOP_REAR  = 0.20
MANUAL_SLOW_REAR  = 0.70

# Logging (CSV)
ENABLE_LOG = True


def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def angle_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def heading_from_compass(c):
    # Common Webots compass convention
    return math.atan2(c[0], c[2])

def set_wheels(left_motor, right_motor, v, w):
    """
    v: forward command [-1..1]
    w: turn command    [-1..1] (positive = left)
    """
    base = v * MAX_WHEEL_SPEED
    turn = w * (0.60 * MAX_WHEEL_SPEED)
    l = clamp(base - turn, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    r = clamp(base + turn, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    left_motor.setVelocity(l)
    right_motor.setVelocity(r)

def speed_scale_linear(dist, stop_d, slow_d):
    """Scale 0..1 between stop and slow distances."""
    if dist <= stop_d:
        return 0.0
    if dist >= slow_d:
        return 1.0
    return clamp((dist - stop_d) / (slow_d - stop_d + 1e-9), 0.0, 1.0)


# ---------------- Init ----------------
robot = Robot()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
set_wheels(left_motor, right_motor, 0.0, 0.0)

sensor_names = [
    "us_front", "us_front_left", "us_front_right",
    "us_left", "us_right",
    "us_rear", "us_rear_left", "us_rear_right"
]
sensors = {}
for n in sensor_names:
    ds = robot.getDevice(n)
    ds.enable(TIME_STEP)
    sensors[n] = ds

gps = robot.getDevice("gps"); gps.enable(TIME_STEP)
compass = robot.getDevice("compass"); compass.enable(TIME_STEP)

led = robot.getDevice("warning_led")  # binary LED in .wbt

# Logging
if ENABLE_LOG:
    log_path = os.path.join(os.path.dirname(__file__), "run_log.csv")
    f = open(log_path, "w", newline="", encoding="utf-8")
    wcsv = csv.writer(f)
    wcsv.writerow(["t","mode","state","x","y","frontMin","L","R","v","w","note"])
    f.flush()
else:
    f = None
    wcsv = None

def log_row(t, mode, state, x, y, front_min, left_space, right_space, v, w, note=""):
    if not ENABLE_LOG:
        return
    wcsv.writerow([f"{t:.2f}", mode, state, f"{x:.2f}", f"{y:.2f}",
                   f"{front_min:.2f}", f"{left_space:.2f}", f"{right_space:.2f}",
                   f"{v:.2f}", f"{w:.2f}", note])
    # flush roughly once per second
    if int(t*10) % 10 == 0:
        f.flush()

MODE_MANUAL = "MANUAL"
MODE_AUTO = "AUTO"
mode = MODE_MANUAL

STATE_GOAL = "GOAL_SEEK"
STATE_AVOID = "AVOID"
STATE_REC_BACK = "REC_BACK"
STATE_REC_TURN = "REC_TURN"
state = STATE_GOAL

avoid_dir = 0
avoid_until = 0.0

t0 = time.time()
def now():
    return time.time() - t0

# AUTO stuck detection
last_progress_t = 0.0
last_dist = None

# AUTO recovery timing
rec_end_time = 0.0
rec_turn_dir = 1

# MANUAL key-hold memory (to allow move+turn together)
last_fwd_cmd = 0.0
last_turn_cmd = 0.0
last_fwd_t = -1e9
last_turn_t = -1e9

print("READY: M=Manual, G=Auto, Space=Stop")

while robot.step(TIME_STEP) != -1:
    t = now()

    # Read all keyboard events in the queue
    key = keyboard.getKey()
    last_key = -1
    while key != -1:
        last_key = key
        key = keyboard.getKey()

    # Space = emergency stop (both modes)
    if last_key == 32:
        set_wheels(left_motor, right_motor, 0.0, 0.0)
        led.set(1)
        continue
    else:
        led.set(0)

    # Read sensors (meters)
    d = {k: float(sensors[k].getValue()) for k in sensor_names}
    front_min = min(d["us_front"], d["us_front_left"], d["us_front_right"])
    left_space  = min(d["us_left"],  d["us_front_left"])
    right_space = min(d["us_right"], d["us_front_right"])

    # Pose
    x, y, _ = gps.getValues()
    x = float(x); y = float(y)

    # Mode switching
    if last_key == ord('M'):
        mode = MODE_MANUAL
        state = STATE_GOAL
        print("[MODE] MANUAL")
    elif last_key == ord('G'):
        mode = MODE_AUTO
        state = STATE_GOAL
        avoid_dir = 0
        avoid_until = 0.0
        # reset progress tracking
        last_dist = None
        last_progress_t = t
        print("[MODE] AUTO")

    # ============================================================
    # MANUAL MODE (User control + safety slow/stop only)
    # ============================================================
    if mode == MODE_MANUAL:
        # Update remembered forward command
        if last_key in (Keyboard.UP, ord('W')):
            last_fwd_cmd = MANUAL_FWD
            last_fwd_t = t
        elif last_key in (Keyboard.DOWN, ord('S')):
            last_fwd_cmd = -MANUAL_REV
            last_fwd_t = t

        # Update remembered turn command
        if last_key in (Keyboard.LEFT, ord('A')):
            last_turn_cmd = MANUAL_TURN
            last_turn_t = t
        elif last_key in (Keyboard.RIGHT, ord('D')):
            last_turn_cmd = -MANUAL_TURN
            last_turn_t = t

        # Key-hold window: enables move+turn together smoothly
        v = last_fwd_cmd if (t - last_fwd_t) <= KEY_HOLD_SEC else 0.0
        wturn = last_turn_cmd if (t - last_turn_t) <= KEY_HOLD_SEC else 0.0

        # Safety scaling (slow down / stop) — no auto steering
        side_min = min(d["us_left"], d["us_right"])
        rear_min = min(d["us_rear"], d["us_rear_left"], d["us_rear_right"])

        speed_scale = 1.0
        note = ""

        if v > 0:
            s_front = speed_scale_linear(front_min, MANUAL_STOP_FRONT, MANUAL_SLOW_FRONT)
            s_side  = speed_scale_linear(side_min,  MANUAL_STOP_SIDE,  MANUAL_SLOW_SIDE)
            speed_scale = min(s_front, s_side)
        elif v < 0:
            s_rear = speed_scale_linear(rear_min, MANUAL_STOP_REAR, MANUAL_SLOW_REAR)
            s_side = speed_scale_linear(side_min, MANUAL_STOP_SIDE, MANUAL_SLOW_SIDE)
            speed_scale = min(s_rear, s_side)
        else:
            # turning in place: prevent scraping if extremely close to side
            if side_min <= MANUAL_STOP_SIDE:
                speed_scale = 0.0

        if speed_scale <= 0.001:
            v_cmd, w_cmd = 0.0, 0.0
            led.set(1)
            note = "MANUAL_ESTOP"
        else:
            v_cmd = v * speed_scale
            w_cmd = wturn * speed_scale
            if speed_scale < 1.0 and abs(v) > 0:
                note = "MANUAL_SLOW"

        set_wheels(left_motor, right_motor, v_cmd, w_cmd)
        log_row(t, MODE_MANUAL, "MANUAL", x, y, front_min, left_space, right_space, v_cmd, w_cmd, note)
        continue

    # ============================================================
    # AUTO MODE (Working goal+avoid code)
    # ============================================================
    dist = math.hypot(GOAL_X - x, GOAL_Y - y)

    # Stop at goal
    if dist <= GOAL_TOL:
        set_wheels(left_motor, right_motor, 0.0, 0.0)
        log_row(t, MODE_AUTO, "AT_GOAL", x, y, front_min, left_space, right_space, 0.0, 0.0, "goal")
        continue

    # True emergency stop only if extremely close (AUTO)
    if front_min <= E_STOP_DIST:
        set_wheels(left_motor, right_motor, 0.0, 0.0)
        led.set(1)
        log_row(t, MODE_AUTO, "ESTOP", x, y, front_min, left_space, right_space, 0.0, 0.0, "estop")
        continue

    # Stuck detector: if distance to goal isn't decreasing -> recover
    if last_dist is None:
        last_dist = dist
        last_progress_t = t
    else:
        if dist < (last_dist - 0.03):
            last_dist = dist
            last_progress_t = t

    if (t - last_progress_t) > STUCK_TIME and state not in (STATE_REC_BACK, STATE_REC_TURN):
        rec_turn_dir = 1 if left_space > right_space else -1
        state = STATE_REC_BACK
        rec_end_time = t + REC_BACK_TIME
        last_progress_t = t
        log_row(t, MODE_AUTO, state, x, y, front_min, left_space, right_space, 0.0, 0.0, "stuck->recover")

    # Recovery: back up
    if state == STATE_REC_BACK:
        set_wheels(left_motor, right_motor, -0.45, 0.0)
        if t >= rec_end_time:
            state = STATE_REC_TURN
            rec_end_time = t + REC_TURN_TIME
        log_row(t, MODE_AUTO, state, x, y, front_min, left_space, right_space, -0.45, 0.0)
        continue

    # Recovery: turn
    if state == STATE_REC_TURN:
        set_wheels(left_motor, right_motor, 0.15, 0.95 * rec_turn_dir)
        if t >= rec_end_time:
            state = STATE_GOAL
            avoid_dir = 0
        log_row(t, MODE_AUTO, state, x, y, front_min, left_space, right_space, 0.15, 0.95 * rec_turn_dir)
        continue

    # GOAL SEEK
    if state == STATE_GOAL:
        if front_min < FRONT_BLOCK:
            avoid_dir = 1 if left_space > right_space else -1
            state = STATE_AVOID
            avoid_until = t + MIN_AVOID_TIME
        else:
            dx = GOAL_X - x
            dy = GOAL_Y - y
            desired = math.atan2(dy, dx)
            heading = heading_from_compass(compass.getValues())
            err = angle_wrap(desired - heading)

            wturn = clamp(AUTO_TURN_GAIN * err, -0.65, 0.65)
            set_wheels(left_motor, right_motor, AUTO_FWD, wturn)
            log_row(t, MODE_AUTO, state, x, y, front_min, left_space, right_space, AUTO_FWD, wturn)
            continue

    # AVOID
    if state == STATE_AVOID:
        if (t >= avoid_until) and (front_min >= FRONT_CLEAR):
            state = STATE_GOAL
            avoid_dir = 0
        else:
            bias = 0.0
            if avoid_dir == 1 and d["us_left"] < 0.35:
                bias = -0.25
            if avoid_dir == -1 and d["us_right"] < 0.35:
                bias = 0.25

            wturn = clamp(AVOID_TURN * avoid_dir + bias, -1.0, 1.0)
            set_wheels(left_motor, right_motor, AVOID_FWD, wturn)
            log_row(t, MODE_AUTO, state, x, y, front_min, left_space, right_space, AVOID_FWD, wturn, f"dir={avoid_dir}")
            continue

# Close log cleanly
if ENABLE_LOG and f is not None:
    f.close()
