import eventlet
eventlet.monkey_patch()

from flask import Flask, Response, jsonify, make_response, request
import cv2
import time
import RPi.GPIO as GPIO
import logging
from threading import Thread, Lock
import threading
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
import json
import asyncio
import websockets
from hypercorn.config import Config
from hypercorn.asyncio import serve
import sys
from flask_socketio import SocketIO
from engineio.async_drivers import eventlet
from flask_cors import CORS

# Basic logging setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='eventlet',
    logger=True,
    engineio_logger=True,
    ping_timeout=60,
    ping_interval=25
)

# Global camera variable
camera = None

# GPIO Setup for L298N Motor Drivers
# First L298N Driver (Rear Right and Rear Left Motors)
ENA_REAR_RIGHT, IN1_REAR_RIGHT, IN2_REAR_RIGHT = 25, 23, 24  # Right rear motor
ENB_REAR_LEFT, IN3_REAR_LEFT, IN4_REAR_LEFT = 18, 17, 22     # Left rear motor

# Second L298N Driver (Front Right and Front Left Motors)
ENA_FRONT_RIGHT, IN1_FRONT_RIGHT, IN2_FRONT_RIGHT = 5, 6, 13  # Right front motor
ENB_FRONT_LEFT, IN3_FRONT_LEFT, IN4_FRONT_LEFT = 19, 26, 12   # Left front motor

# Ultrasonic sensor pins
TRIGGER_PIN, ECHO_PIN = 27, 4

# Global PWM variables
pwm_rear_right = None
pwm_rear_left = None
pwm_front_right = None
pwm_front_left = None

obstacle_avoidance_active = False
obstacle_avoidance_thread = None
autonomous_navigation_active = False
autonomous_navigation_thread = None
distance_threshold = 35  # cm

# Variables for visual odometry and path tracking
prev_gray = None
prev_points = None
robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
pose_lock = threading.Lock()
path_history = deque(maxlen=5000)  # Store past positions to help return to path

# Add these global variables
map_points = []  # 3D points in world coordinates
map_descriptors = []  # Feature descriptors for map points
keyframes = []  # Store key frames for loop closure
keyframe_poses = []  # Store poses of key frames
local_map = None  # Current local map
orb = cv2.ORB_create(nfeatures=1000)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
min_matches = 30  # Minimum matches needed for tracking
keyframe_threshold = 50  # Create new keyframe if matches fall below this
map_lock = threading.Lock()
pose_graph = []  # Store pose history for optimization
local_window_size = 7  # Number of frames for local optimization
correction_threshold = 0.8  # Threshold for loop closure confidence
websocket_clients = set()
last_update_time = time.time()
last_command_time = time.time()
current_linear_speed = 0.0
current_angular_speed = 0.0
WHEEL_DIAMETER = 0.11  # 11cm in meters
MOTOR_RPM = 31.5  # Adjusted RPM for 8-9V supply
LINEAR_SPEED = (MOTOR_RPM * np.pi * WHEEL_DIAMETER) / 60  # ~0.1814 m/s
ANGULAR_SPEED = LINEAR_SPEED / (0.3/2)  # Using robot width (30cm) for rotation

# Motor control functions for 4 motors
def set_motor_rear_right(speed):
    if speed > 0:
        GPIO.output(IN1_REAR_RIGHT, GPIO.HIGH)
        GPIO.output(IN2_REAR_RIGHT, GPIO.LOW)
    else:
        GPIO.output(IN1_REAR_RIGHT, GPIO.LOW)
        GPIO.output(IN2_REAR_RIGHT, GPIO.HIGH)
    pwm_rear_right.ChangeDutyCycle(abs(speed))

def set_motor_rear_left(speed):
    if speed > 0:
        GPIO.output(IN3_REAR_LEFT, GPIO.HIGH)
        GPIO.output(IN4_REAR_LEFT, GPIO.LOW)
    else:
        GPIO.output(IN3_REAR_LEFT, GPIO.LOW)
        GPIO.output(IN4_REAR_LEFT, GPIO.HIGH)
    pwm_rear_left.ChangeDutyCycle(abs(speed))

def set_motor_front_right(speed):
    if speed > 0:
        GPIO.output(IN1_FRONT_RIGHT, GPIO.HIGH)
        GPIO.output(IN2_FRONT_RIGHT, GPIO.LOW)
    else:
        GPIO.output(IN1_FRONT_RIGHT, GPIO.LOW)
        GPIO.output(IN2_FRONT_RIGHT, GPIO.HIGH)
    pwm_front_right.ChangeDutyCycle(abs(speed))

def set_motor_front_left(speed):
    if speed > 0:
        GPIO.output(IN3_FRONT_LEFT, GPIO.HIGH)
        GPIO.output(IN4_FRONT_LEFT, GPIO.LOW)
    else:
        GPIO.output(IN3_FRONT_LEFT, GPIO.LOW)
        GPIO.output(IN4_FRONT_LEFT, GPIO.HIGH)
    pwm_front_left.ChangeDutyCycle(abs(speed))

def stop_motors():
    # Stop all motors
    pwm_rear_right.ChangeDutyCycle(0)
    pwm_rear_left.ChangeDutyCycle(0)
    pwm_front_right.ChangeDutyCycle(0)
    pwm_front_left.ChangeDutyCycle(0)

    # Set all motor control pins to LOW
    GPIO.output(IN1_REAR_RIGHT, GPIO.LOW)
    GPIO.output(IN2_REAR_RIGHT, GPIO.LOW)
    GPIO.output(IN3_REAR_LEFT, GPIO.LOW)
    GPIO.output(IN4_REAR_LEFT, GPIO.LOW)
    GPIO.output(IN1_FRONT_RIGHT, GPIO.LOW)
    GPIO.output(IN2_FRONT_RIGHT, GPIO.LOW)
    GPIO.output(IN3_FRONT_LEFT, GPIO.LOW)
    GPIO.output(IN4_FRONT_LEFT, GPIO.LOW)

# CORS decorator
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response

def setup_camera():
    # Try different video backends
    cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cam.isOpened():
        cam = cv2.VideoCapture(0)

    if cam.isOpened():
        # Set camera parameters
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cam.set(cv2.CAP_PROP_FPS, 20)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 2)

        # Give camera time to initialize
        time.sleep(2)

        logger.info("Camera initialized successfully")
        return cam
    else:
        logger.error("Failed to open camera")
        return None

def setup_gpio():
    global pwm_rear_right, pwm_rear_left, pwm_front_right, pwm_front_left
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor pins for both drivers
    pins = [
        ENA_REAR_RIGHT, IN1_REAR_RIGHT, IN2_REAR_RIGHT,
        ENB_REAR_LEFT, IN3_REAR_LEFT, IN4_REAR_LEFT,
        ENA_FRONT_RIGHT, IN1_FRONT_RIGHT, IN2_FRONT_RIGHT,
        ENB_FRONT_LEFT, IN3_FRONT_LEFT, IN4_FRONT_LEFT
    ]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)

    # Ultrasonic sensor pins
    GPIO.setup(TRIGGER_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    # Initialize PWM for all motors
    pwm_rear_right = GPIO.PWM(ENA_REAR_RIGHT, 100)
    pwm_rear_left = GPIO.PWM(ENB_REAR_LEFT, 100)
    pwm_front_right = GPIO.PWM(ENA_FRONT_RIGHT, 100)
    pwm_front_left = GPIO.PWM(ENB_FRONT_LEFT, 100)

    # Start PWM
    pwm_rear_right.start(0)
    pwm_rear_left.start(0)
    pwm_front_right.start(0)
    pwm_front_left.start(0)

    return pwm_rear_right, pwm_rear_left, pwm_front_right, pwm_front_left

def measure_distance():
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIGGER_PIN, GPIO.LOW)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # cm
    return distance

def detect_obstacles(frame):
    if frame is None:
        return False

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 500:
            return True
    return False

def generate_frames():
    global camera
    while True:
        try:
            if camera is None:
                camera = setup_camera()
                if camera is None:
                    time.sleep(1)
                    continue

            ret, frame = camera.read()
            if not ret or frame is None:
                logger.error("Failed to grab frame, reinitializing camera...")
                camera.release()
                camera = None
                time.sleep(1)
                continue

            # Encode frame to JPEG
            ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ret:
                continue

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

            # Small delay to prevent overwhelming the Pi
            time.sleep(0.01)

        except Exception as e:
            logger.error(f"Frame generation error: {e}")
            time.sleep(1)
            if camera is not None:
                camera.release()
                camera = None

def perform_visual_odometry(frame):
    global prev_gray, prev_points, robot_pose, path_history
    global pose_lock

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if prev_gray is None or prev_points is None or len(prev_points) < 10:
        # Initialize previous frame and points
        prev_gray = gray
        # Use feature detector to find initial points
        prev_points = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
        return

    # Calculate optical flow to track points
    next_points, status, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_points, None)

    # Select good points
    good_new = next_points[status == 1]
    good_old = prev_points[status == 1]

    # Estimate motion between frames
    if len(good_new) >= 4:
        # Compute the transformation matrix (Affine) between old and new points
        M, inliers = cv2.estimateAffinePartial2D(good_old, good_new)
        if M is not None:
            # Extract rotation and translation
            dx = M[0, 2]
            dy = M[1, 2]
            # Approximate rotation angle
            da = np.arctan2(M[1, 0], M[0, 0])

            # Scaling factors to convert pixels to meters (calibration needed)
            scale_x = 0.001  # meters per pixel (example value)
            scale_y = 0.001  # meters per pixel

            # Update pose
            with pose_lock:
                robot_pose[0] += dx * scale_x
                robot_pose[1] += dy * scale_y
                robot_pose[2] += da

                # Record the new position in the path history
                path_history.append(robot_pose.copy())
        else:
            # Transformation matrix not found
            pass
    else:
        # Not enough good points
        pass

    # Update previous frame and points
    prev_gray = gray.copy()
    prev_points = good_new.reshape(-1, 1, 2)

def autonomous_navigation_loop():
    global autonomous_navigation_active, robot_pose, pose_lock, path_history
    global camera, target_area_length, target_area_breadth

    max_speed = 50  # Adjust as needed
    speed = max_speed

    # Initialize the state
    turning = False
    turn_direction = 1  # 1 for right turn, -1 for left turn

    while autonomous_navigation_active:
        try:
            # Get current frame from camera
            if camera is not None:
                ret, frame = camera.read()
                if not ret or frame is None:
                    continue

                # Perform visual odometry
                perform_visual_odometry(frame)

                # Get the current pose
                with pose_lock:
                    x, y, theta = robot_pose.copy()

                # Check for obstacles using ultrasonic sensor
                distance = measure_distance()

                # Check for obstacles using camera
                camera_obstacle = detect_obstacles(frame)

                # Handle obstacle detection
                if distance <= distance_threshold or camera_obstacle:
                    stop_motors()
                    time.sleep(0.1)
                    # Turn to avoid obstacle
                    turn_time = 0.5  # Adjust time to achieve desired turn
                    set_motor_rear_left(-turn_direction * speed)
                    set_motor_rear_right(turn_direction * speed)
                    set_motor_front_left(turn_direction * speed)
                    set_motor_front_right(-turn_direction * speed)
                    time.sleep(turn_time)
                    stop_motors()
                    time.sleep(0.1)

                    # Modify pose to reflect the turn
                    with pose_lock:
                        robot_pose[2] += turn_direction * np.pi / 4  # Adjust angle as needed
                else:
                    # If we previously turned to avoid an obstacle, attempt to return to the path
                    if len(path_history) > 0:
                        with pose_lock:
                            # Get the last known good position
                            target_pose = path_history[-1]
                            dx = target_pose[0] - robot_pose[0]
                            dy = target_pose[1] - robot_pose[1]
                            distance_to_target = np.hypot(dx, dy)

                            if distance_to_target > 0.1:  # Threshold to decide if we need to correct
                                # Calculate the angle to the target
                                angle_to_target = np.arctan2(dy, dx)
                                angle_diff = angle_to_target - robot_pose[2]

                                # Normalize angle_diff to [-pi, pi]
                                angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

                                # Decide whether to turn or move forward
                                if abs(angle_diff) > 0.1:  # If angle difference is significant
                                    # Rotate towards the target
                                    rotation_speed = speed * np.sign(angle_diff)
                                    set_motor_rear_left(rotation_speed)
                                    set_motor_rear_right(-rotation_speed)
                                    set_motor_front_left(-rotation_speed)
                                    set_motor_front_right(rotation_speed)
                                else:
                                    # Move forward towards the target
                                    set_motor_rear_left(speed)
                                    set_motor_rear_right(speed)
                                    set_motor_front_left(speed)
                                    set_motor_front_right(speed)
                            else:
                                # We are back on the path; continue normal navigation
                                set_motor_rear_left(speed)
                                set_motor_rear_right(speed)
                                set_motor_front_left(speed)
                                set_motor_front_right(speed)
                    else:
                        # No path history; move forward
                        set_motor_rear_left(speed)
                        set_motor_rear_right(speed)
                        set_motor_front_left(speed)
                        set_motor_front_right(speed)

                # Check if we have reached the boundary of the area
                with pose_lock:
                    half_length = target_area_length / 2
                    half_breadth = target_area_breadth / 2
                    
                    # Check if we've reached either the length or breadth boundaries
                    if abs(x) >= half_length or abs(y) >= half_breadth:
                        # At boundary, turn
                        stop_motors()
                        time.sleep(0.1)
                        # Rotate 90 degrees
                        rotate_time = 1.0  # Adjust this time to achieve 90-degree rotation
                        rotation_speed = 30  # Adjust as needed
                        set_motor_rear_left(-turn_direction * rotation_speed)
                        set_motor_rear_right(turn_direction * rotation_speed)
                        set_motor_front_left(-turn_direction * rotation_speed)
                        set_motor_front_right(turn_direction * rotation_speed)
                        time.sleep(rotate_time)
                        stop_motors()
                        time.sleep(0.1)

                        # Update robot pose to reflect the turn
                        robot_pose[2] += turn_direction * (np.pi / 2)

                        # Change turn direction for next boundary
                        turn_direction *= -1

                time.sleep(0.1)
            else:
                # Camera not available
                pass
        except Exception as e:
            logger.error(f"Error in autonomous navigation loop: {e}")
            stop_motors()
            break

    stop_motors()

@app.route('/control', methods=['POST', 'OPTIONS'])
def control():
    if request.method == 'OPTIONS':
        return add_cors_headers(make_response())

    try:
        if autonomous_navigation_active:
            return add_cors_headers(jsonify({'error': 'Cannot control robot manually while autonomous navigation is active'})), 400

        command = request.json.get('command')
        logger.info(f"Received control command: {command}")  # Debug log
        
        speed = 100  # Maximum speed (100% duty cycle)

        if command == 'forward':
            # All motors forward
            set_motor_rear_left(speed)
            set_motor_rear_right(speed)
            set_motor_front_left(speed)
            set_motor_front_right(speed)
            current_linear_speed = LINEAR_SPEED
            current_angular_speed = 0
        elif command == 'backward':
            # All motors backward
            set_motor_rear_left(-speed)
            set_motor_rear_right(-speed)
            set_motor_front_left(-speed)
            set_motor_front_right(-speed)
            current_linear_speed = -LINEAR_SPEED
            current_angular_speed = 0
        elif command == 'left':
            # Rotate in place (opposite rotation of side motors)
            set_motor_rear_left(speed)
            set_motor_rear_right(-speed)
            set_motor_front_left(-speed)
            set_motor_front_right(speed)
            current_linear_speed = 0
            current_angular_speed = ANGULAR_SPEED
        elif command == 'right':
            # Rotate in place in opposite direction
            set_motor_rear_left(-speed)
            set_motor_rear_right(speed)
            set_motor_front_left(speed)
            set_motor_front_right(-speed)
            current_linear_speed = 0
            current_angular_speed = -ANGULAR_SPEED
        elif command == 'stop':
            stop_motors()
            current_linear_speed = 0
            current_angular_speed = 0
        else:
            return add_cors_headers(jsonify({'error': 'Invalid command'})), 400

        # Update pose after command
        update_manual_pose()
        
        logger.info(f"Updated speeds - Linear: {current_linear_speed}, Angular: {current_angular_speed}")
        
        return add_cors_headers(jsonify({'status': 'success', 'command': command}))

    except Exception as e:
        logger.error(f"Control error: {e}")
        return add_cors_headers(jsonify({'error': str(e)})), 500

@app.route('/status')
def status():
    """Endpoint to check if the server is running and camera is working"""
    try:
        camera_status = camera is not None and camera.isOpened()
        response = jsonify({
            'status': 'online',
            'camera': 'connected' if camera_status else 'disconnected',
            'timestamp': time.time()
        })
        return add_cors_headers(response)
    except Exception as e:
        logger.error(f"Status check error: {e}")
        response = jsonify({
            'status': 'error',
            'message': str(e)
        })
        return add_cors_headers(response), 500

@app.route('/stream')
def video_feed():
    response = Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
    return add_cors_headers(response)

@app.route('/')
def index():
    response = make_response("""
    <html>
        <body style="text-align: center;">
            <h1>USB Camera Stream</h1>
            <img src="/stream" width="320" height="240" />
        </body>
    </html>
    """)
    return add_cors_headers(response)

@app.route('/obstacle-mode', methods=['POST', 'OPTIONS'])
def toggle_obstacle_mode():
    if request.method == 'OPTIONS':
        return add_cors_headers(make_response())

    try:
        global autonomous_navigation_active, autonomous_navigation_thread

        data = request.json
        enabled = data.get('enabled', False)

        if enabled and not autonomous_navigation_active:
            autonomous_navigation_active = True
            autonomous_navigation_thread = Thread(target=autonomous_navigation_loop)
            autonomous_navigation_thread.start()
            message = "Autonomous navigation mode enabled"
        elif not enabled and autonomous_navigation_active:
            autonomous_navigation_active = False
            if autonomous_navigation_thread:
                autonomous_navigation_thread.join(timeout=2.0)
            stop_motors()
            message = "Autonomous navigation mode disabled"
        else:
            message = "No change in autonomous navigation mode"

        return add_cors_headers(jsonify({
            'status': 'success',
            'message': message,
            'enabled': autonomous_navigation_active
        }))

    except Exception as e:
        logger.error(f"Autonomous mode error: {e}")
        return add_cors_headers(jsonify({'error': str(e)})), 500

# Handle OPTIONS requests for CORS
@app.route('/', methods=['OPTIONS'])
@app.route('/status', methods=['OPTIONS'])
@app.route('/stream', methods=['OPTIONS'])
def handle_options():
    response = make_response()
    return add_cors_headers(response)

@app.route('/geofence', methods=['POST', 'OPTIONS'])
def set_geofence():
    if request.method == 'OPTIONS':
        return add_cors_headers(make_response())

    try:
        global target_area_length, target_area_breadth
        data = request.json
        length = float(data.get('length', 10.0))
        breadth = float(data.get('breadth', 10.0))

        # Update the global variables
        target_area_length = length
        target_area_breadth = breadth

        return add_cors_headers(jsonify({
            'status': 'success',
            'message': f'Geofence set to {length}m x {breadth}m',
            'length': length,
            'breadth': breadth
        }))

    except Exception as e:
        logger.error(f"Geofence setting error: {e}")
        return add_cors_headers(jsonify({'error': str(e)})), 500

def initialize_slam(frame):
    """Initialize SLAM system with first frame"""
    global local_map, keyframes, keyframe_poses, map_points, map_descriptors
    
    # Extract features from first frame
    kp, desc = orb.detectAndCompute(frame, None)
    if len(kp) < min_matches:
        return False
        
    # Initialize first keyframe
    keyframes.append((kp, desc))
    keyframe_poses.append(np.eye(4))  # Identity pose for first frame
    
    # Initialize map with 2D points (will be converted to 3D later)
    points = np.array([kp_.pt for kp_ in kp])
    map_points = points
    map_descriptors = desc
    
    local_map = {
        'points': points,
        'descriptors': desc
    }
    
    return True

def optimize_local_poses(poses, points_3d, observations, camera_matrix):
    """
    Local bundle adjustment to optimize recent poses
    """
    def project_points(points, pose):
        # Convert pose vector to matrix
        rotation = R.from_rotvec(pose[:3]).as_matrix()
        translation = pose[3:].reshape(3, 1)
        
        # Transform points
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        transformed_points = np.dot(points_homogeneous, np.vstack((np.hstack((rotation, translation)), [0, 0, 0, 1])).T)
        
        # Project to image plane
        projected = np.dot(transformed_points[:, :3], camera_matrix.T)
        projected = projected[:, :2] / projected[:, 2:]
        return projected

    def residuals(params):
        n_poses = len(poses)
        pose_params = params[:n_poses * 6].reshape(n_poses, 6)
        errors = []
        
        for i, pose in enumerate(pose_params):
            projected = project_points(points_3d, pose)
            errors.append((projected - observations[i]).ravel())
            
        return np.concatenate(errors)

    # Initial parameters (convert poses to axis-angle representation)
    initial_params = []
    for pose in poses:
        rot_vec = R.from_matrix(pose[:3, :3]).as_rotvec()
        trans_vec = pose[:3, 3]
        initial_params.extend(np.concatenate([rot_vec, trans_vec]))

    # Optimize
    result = least_squares(residuals, initial_params, method='dogbox', max_nfev=20)
    
    # Convert back to pose matrices
    optimized_poses = []
    params = result.x.reshape(-1, 6)
    for param in params:
        pose = np.eye(4)
        pose[:3, :3] = R.from_rotvec(param[:3]).as_matrix()
        pose[:3, 3] = param[3:]
        optimized_poses.append(pose)
        
    return optimized_poses

def correct_loop_closure(current_pose, matched_keyframe_pose, matched_keyframe_idx):
    """
    Correct pose graph when loop closure is detected
    """
    global pose_graph, keyframe_poses
    
    # Calculate drift
    drift = np.dot(current_pose, np.linalg.inv(matched_keyframe_pose))
    
    # Create correction transform
    correction = np.linalg.inv(drift)
    
    # Apply correction to all poses after the matched keyframe
    with pose_lock:
        for i in range(matched_keyframe_idx + 1, len(keyframe_poses)):
            keyframe_poses[i] = np.dot(correction, keyframe_poses[i])
        
        # Update current robot pose
        robot_pose[:] = np.dot(correction, current_pose)

def detect_and_correct_loop_closure(current_kp, current_desc, current_pose):
    """
    Enhanced loop closure detection with pose correction
    """
    if len(keyframes) < 20:  # Need sufficient history
        return False
        
    best_match_idx = -1
    best_match_count = 0
    best_confidence = 0
    
    # Compare with old keyframes
    for i, (kp, desc) in enumerate(keyframes[:-20]):  # Skip recent frames
        matches = matcher.match(current_desc, desc)
        matches = sorted(matches, key=lambda x: x.distance)
        
        # Calculate confidence score
        if len(matches) > min_matches * 2:
            good_matches = [m for m in matches if m.distance < 0.7 * matches[0].distance]
            confidence = len(good_matches) / len(current_kp)
            
            if confidence > correction_threshold and len(good_matches) > best_match_count:
                best_match_idx = i
                best_match_count = len(good_matches)
                best_confidence = confidence
    
    # If good loop closure found, correct poses
    if best_match_idx >= 0:
        logger.info(f"Loop closure detected with keyframe {best_match_idx} (confidence: {best_confidence:.2f})")
        correct_loop_closure(current_pose, keyframe_poses[best_match_idx], best_match_idx)
        return True
        
    return False

def track_and_map(frame):
    """Main SLAM function for tracking and mapping with pose correction"""
    global local_map, robot_pose, map_points, map_descriptors, keyframes, keyframe_poses, pose_graph
    
    # Extract features from current frame
    kp_current, desc_current = orb.detectAndCompute(frame, None)
    if len(kp_current) < min_matches:
        return False
        
    with map_lock:
        if local_map is None:
            return initialize_slam(frame)
            
        # Match features with local map
        matches = matcher.match(desc_current, local_map['descriptors'])
        matches = sorted(matches, key=lambda x: x.distance)
        
        if len(matches) < min_matches:
            return False
            
        # Get matched point pairs
        points_current = np.float32([kp_current[m.queryIdx].pt for m in matches])
        points_map = np.float32([local_map['points'][m.trainIdx] for m in matches])
        
        # Estimate essential matrix and recover pose
        E, mask = cv2.findEssentialMat(points_current, points_map, 
                                     focal=frame.shape[1],
                                     pp=(frame.shape[1]/2, frame.shape[0]/2))
                                     
        _, R, t, mask = cv2.recoverPose(E, points_current, points_map)
        
        # Update robot pose
        with pose_lock:
            current_pose = np.eye(4)
            current_pose[:3, :3] = R
            current_pose[:3, 3] = t.reshape(-1)
            robot_pose = update_robot_pose(robot_pose, current_pose)
            
            # Add to pose graph
            pose_graph.append(robot_pose.copy())
            
            # Local pose optimization
            if len(pose_graph) >= local_window_size:
                recent_poses = pose_graph[-local_window_size:]
                # Note: You'll need to maintain corresponding 3D points and observations
                # This is a simplified version
                optimized_poses = optimize_local_poses(
                    recent_poses,
                    map_points[-len(points_current):],
                    points_current,
                    np.array([[frame.shape[1], 0, frame.shape[1]/2],
                            [0, frame.shape[1], frame.shape[0]/2],
                            [0, 0, 1]]))
                # Update most recent pose
                robot_pose[:] = optimized_poses[-1]
            
        # Check if we need to create new keyframe
        if len(matches) < keyframe_threshold:
            create_keyframe(frame, kp_current, desc_current, robot_pose.copy())
            
        # Update local map
        update_local_map(points_current, desc_current)
        
        # Perform loop closure detection periodically
        if len(keyframes) % 10 == 0:
            detect_and_correct_loop_closure(kp_current, desc_current, robot_pose.copy())
            
    return True

def update_robot_pose(current_pose, delta_pose):
    """Update robot pose with new transformation"""
    new_pose = np.dot(current_pose, delta_pose)
    return new_pose

def create_keyframe(frame, keypoints, descriptors, pose):
    """Add new keyframe to the map"""
    global keyframes, keyframe_poses
    
    keyframes.append((keypoints, descriptors))
    keyframe_poses.append(pose)
    
    # Limit number of keyframes to prevent memory issues
    if len(keyframes) > 1000:
        keyframes.pop(0)
        keyframe_poses.pop(0)

def update_local_map(new_points, new_descriptors):
    """Update local map with new observations"""
    global local_map, map_points, map_descriptors
    
    # Simple update - just add new points
    if len(map_points) == 0:
        map_points = new_points
        map_descriptors = new_descriptors
    else:
        map_points = np.vstack((map_points, new_points))
        map_descriptors = np.vstack((map_descriptors, new_descriptors))
    
    # Update local map
    local_map = {
        'points': map_points[-1000:],  # Keep last 1000 points
        'descriptors': map_descriptors[-1000:]
    }

def update_manual_pose():
    """Update robot pose based on manual control commands"""
    global robot_pose, current_linear_speed, current_angular_speed, last_command_time
    
    current_time = time.time()
    dt = current_time - last_command_time
    
    # Debug log
    logger.info(f"Updating pose - dt: {dt}, linear_speed: {current_linear_speed}, angular_speed: {current_angular_speed}")
    
    # Update position based on current speeds
    if current_linear_speed != 0 or current_angular_speed != 0:
        # Update theta
        new_theta = robot_pose[2] + (current_angular_speed * dt)
        
        # Update x and y based on linear speed and heading
        dx = current_linear_speed * dt * np.cos(robot_pose[2])
        dy = current_linear_speed * dt * np.sin(robot_pose[2])
        
        logger.info(f"Position updates - dx: {dx}, dy: {dy}, dtheta: {current_angular_speed * dt}")
        
        with pose_lock:
            robot_pose[0] += dx
            robot_pose[1] += dy
            robot_pose[2] = new_theta
            
            logger.info(f"New pose - x: {robot_pose[0]}, y: {robot_pose[1]}, theta: {robot_pose[2]}")
    
    last_command_time = current_time

def sendCommand(command):
    """Update manual control commands and speeds"""
    global current_linear_speed, current_angular_speed
    
    logger.info(f"Received command: {command}")
    
    if command == 'forward':
        current_linear_speed = LINEAR_SPEED
        current_angular_speed = 0
    elif command == 'backward':
        current_linear_speed = -LINEAR_SPEED
        current_angular_speed = 0
    elif command == 'left':
        current_linear_speed = 0
        current_angular_speed = ANGULAR_SPEED
    elif command == 'right':
        current_linear_speed = 0
        current_angular_speed = -ANGULAR_SPEED
    elif command == 'stop':
        current_linear_speed = 0
        current_angular_speed = 0
    
    logger.info(f"Updated speeds - Linear: {current_linear_speed}, Angular: {current_angular_speed}")
    
    # Update pose immediately after command
    update_manual_pose()

def emit_pose_update():
    """Emit pose updates via Socket.IO"""
    try:
        global last_update_time, current_linear_speed, current_angular_speed
        
        current_time = time.time()
        
        # Update pose if in manual mode
        if not autonomous_navigation_active:
            update_manual_pose()
        
        with pose_lock:
            # Calculate distance traveled
            if not hasattr(emit_pose_update, 'total_distance'):
                emit_pose_update.total_distance = 0.0
            
            # Update total distance based on current speed
            dt = current_time - last_update_time
            if current_linear_speed != 0:
                emit_pose_update.total_distance += abs(current_linear_speed * dt)

            # Calculate angular velocity (degrees per second)
            angular_velocity = current_angular_speed * 180 / np.pi
            
            pose_data = {
                'x': float(robot_pose[0]),
                'y': float(robot_pose[1]),
                'theta': float(robot_pose[2]),
                'distanceTraveled': float(emit_pose_update.total_distance),
                'angularVelocity': float(angular_velocity),
                'isManual': not autonomous_navigation_active
            }
        
            logger.info(f"Emitting pose: {pose_data}")
            socketio.emit('pose_update', pose_data)
        
        last_update_time = current_time
    except Exception as e:
        logger.error(f"Error in emit_pose_update: {e}")

# Add Socket.IO event handlers
@socketio.on('connect')
def handle_connect():
    logger.info('Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    logger.info('Client disconnected')

@app.route('/pose', methods=['GET', 'OPTIONS'])
def get_pose():
    """Get current robot pose and stats"""
    try:
        # For OPTIONS request (CORS preflight)
        if request.method == 'OPTIONS':
            response = make_response()
            response.headers.add('Access-Control-Allow-Origin', '*')
            response.headers.add('Access-Control-Allow-Headers', '*')
            response.headers.add('Access-Control-Allow-Methods', '*')
            return response

        with pose_lock:
            # Calculate angular velocity (degrees per second)
            angular_velocity = current_angular_speed * 180 / np.pi
            
            # Calculate total distance if not exists
            if not hasattr(get_pose, 'total_distance'):
                get_pose.total_distance = 0.0
            
            # Update total distance based on current speed
            current_time = time.time()
            if not hasattr(get_pose, 'last_update'):
                get_pose.last_update = current_time
            
            dt = current_time - get_pose.last_update
            if current_linear_speed != 0:
                get_pose.total_distance += abs(current_linear_speed * dt)
            
            get_pose.last_update = current_time
            
            pose_data = {
                'x': float(robot_pose[0]),
                'y': float(robot_pose[1]),
                'theta': float(robot_pose[2]),
                'distanceTraveled': float(get_pose.total_distance),
                'angularVelocity': float(angular_velocity),
                'isManual': not autonomous_navigation_active
            }
            
            response = jsonify(pose_data)
            response.headers.add('Access-Control-Allow-Origin', '*')
            return response
            
    except Exception as e:
        logger.error(f"Error getting pose: {e}")
        response = jsonify({'error': str(e)})
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response, 500

if __name__ == '__main__':
    try:
        # Initialize camera
        camera = setup_camera()
        if camera is None:
            raise Exception("Failed to initialize camera")
        logger.info("Camera initialized successfully")

        # Setup GPIO
        pwm_rear_right, pwm_rear_left, pwm_front_right, pwm_front_left = setup_gpio()
        logger.info("GPIO setup completed")

        # Run the server
        logger.info("Starting server...")
        app.run(
            host='0.0.0.0',
            port=8000,
            debug=False,  # Disable debug mode to prevent auto-reloading
            use_reloader=False  # Explicitly disable reloader
        )

    except Exception as e:
        logger.error(f"Startup error: {e}")
        if camera is not None:
            camera.release()
        try:
            stop_motors()
        except:
            pass  # Ignore errors during cleanup
        if autonomous_navigation_active:
            autonomous_navigation_active = False
            if autonomous_navigation_thread:
                autonomous_navigation_thread.join(timeout=2.0)
        if obstacle_avoidance_active:
            obstacle_avoidance_active = False
            if obstacle_avoidance_thread:
                obstacle_avoidance_thread.join(timeout=2.0)
        GPIO.cleanup()
        raise e