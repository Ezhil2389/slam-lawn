import RPi.GPIO as GPIO
from time import sleep, time
import cv2
import curses

# Motor pins
in1, in2, en1 = 24, 23, 25
in3, in4, en2 = 17, 22, 18

# Ultrasonic sensor pins
trigger_pin, echo_pin = 27, 4

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup([in1, in2, en1, in3, in4, en2, trigger_pin], GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

# Motor PWM setup
p1 = GPIO.PWM(en1, 1000)
p2 = GPIO.PWM(en2, 1000)
p1.start(100)
p2.start(100)

# Camera setup
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("[WARNING] Camera not detected. Camera features disabled.")

# Initialize motors
GPIO.output([in1, in2, in3, in4], GPIO.LOW)

# Flags for modes
obstacle_avoidance = False
distance_threshold = 35  # cm


def stop_motors():
    """Stop both motors."""
    GPIO.output([in1, in2, in3, in4], GPIO.LOW)


def forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)


def backward():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)


def turn_right():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)


def measure_distance():
    """Measure distance using ultrasonic sensor."""
    GPIO.output(trigger_pin, GPIO.HIGH)
    sleep(0.00001)
    GPIO.output(trigger_pin, GPIO.LOW)

    start_time = time()
    stop_time = time()

    while GPIO.input(echo_pin) == 0:
        start_time = time()

    while GPIO.input(echo_pin) == 1:
        stop_time = time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # cm
    return distance


def detect_obstacles(frame):
    """Simple obstacle detection using camera."""
    if frame is None:
        return False
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Threshold area
            return True  # Obstacle detected
    return False


def obstacle_avoidance_mode(stdscr):
    """Obstacle detection and stop mode."""
    global obstacle_avoidance
    stdscr.clear()
    stdscr.addstr(0, 0, "Obstacle Avoidance Mode (Press 'O' to exit)")
    stdscr.refresh()

    while obstacle_avoidance:
        # Ultrasonic distance measurement
        distance = measure_distance()
        stdscr.addstr(2, 0, f"Distance: {distance:.2f} cm")
        
        # Camera obstacle detection (if camera is available)
        camera_obstacle = False
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                camera_obstacle = detect_obstacles(frame)
        
        # Check for obstacle conditions
        if distance <= distance_threshold or camera_obstacle:
            stdscr.addstr(4, 0, "[OBSTACLE] Stopping and turning...")
            stop_motors()
            sleep(1)
            turn_right()
            sleep(0.2)
            stop_motors()
        else:
            forward()
        
        # Check for key to exit mode
        key = stdscr.getch()
        if key in [ord('o'), ord('O')]:
            obstacle_avoidance = False
            break
        
        stdscr.refresh()
        sleep(0.1)

    stop_motors()
    stdscr.clear()
    stdscr.addstr(0, 0, "Obstacle Avoidance Mode Ended")
    stdscr.refresh()
    sleep(1)


def main(stdscr):
    global obstacle_avoidance
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "RC Car Controls: W (↑), S (↓), A (←), D (→), O (Obstacle Mode), Q to quit")

    try:
        while True:
            key = stdscr.getch()
            if key in [ord('w')]:  # Forward
                forward()
            elif key in [ord('s')]:  # Backward
                backward()
            elif key in [ord('a')]:  # Turn Left
                GPIO.output(in1, GPIO.HIGH)
                GPIO.output(in2, GPIO.LOW)
                GPIO.output(in3, GPIO.LOW)
                GPIO.output(in4, GPIO.HIGH)
            elif key in [ord('d')]:  # Turn Right
                turn_right()
            elif key in [ord('o'), ord('O')]:  # Toggle obstacle avoidance mode
                obstacle_avoidance = True
                obstacle_avoidance_mode(stdscr)
            elif key in [ord('q')]:  # Quit
                break
            else:
                stop_motors()
            sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Keyboard Interrupt detected. Exiting...")

    finally:
        stop_motors()
        GPIO.cleanup()
        cap.release()
        print("[INFO] Cleanup complete.")


if __name__ == "__main__":
    curses.wrapper(main)
