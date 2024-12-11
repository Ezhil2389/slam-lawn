from flask import Flask, Response, jsonify, make_response, request
import cv2
import time
import RPi.GPIO as GPIO
import logging
from threading import Thread
import time

# Basic logging setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

# Global camera variable
camera = None

# GPIO Setup for L298N Motor Driver
ENA, IN1, IN2 = 25, 23, 24  # Left motor
ENB, IN3, IN4 = 18, 17, 22  # Right motor
# Add these GPIO pins after the motor pins
TRIGGER_PIN, ECHO_PIN = 27, 4


# Global PWM variables
pwm_a = None
pwm_b = None

obstacle_avoidance_active = False
obstacle_avoidance_thread = None
distance_threshold = 35  # cm


# Motor control functions
def set_motor_left(speed):
    if speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(abs(speed))

def set_motor_right(speed):
    if speed > 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    pwm_b.ChangeDutyCycle(abs(speed))

def stop_motors():
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

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
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Existing motor pins
    pins = [ENA, IN1, IN2, ENB, IN3, IN4]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)
    
    # Add ultrasonic sensor pins
    GPIO.setup(TRIGGER_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    
    pwm_a = GPIO.PWM(ENA, 100)
    pwm_b = GPIO.PWM(ENB, 100)
    pwm_a.start(0)
    pwm_b.start(0)
    return pwm_a, pwm_b

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

def obstacle_avoidance_loop():
    global obstacle_avoidance_active
    
    while obstacle_avoidance_active:
        try:
            # Check ultrasonic distance
            distance = measure_distance()
            
            # Check camera for obstacles
            if camera is not None:
                ret, frame = camera.read()
                camera_obstacle = detect_obstacles(frame) if ret else False
            else:
                camera_obstacle = False
            
            # Handle obstacle detection
            if distance <= distance_threshold or camera_obstacle:
                stop_motors()
                time.sleep(1)
                # Turn right to avoid obstacle
                set_motor_left(-100)
                set_motor_right(-100)
                time.sleep(0.5)
                stop_motors()
            else:
                # Move forward
                set_motor_left(-100)
                set_motor_right(100)
            
            time.sleep(0.1)
        except Exception as e:
            logger.error(f"Error in obstacle avoidance loop: {e}")
            stop_motors()
            break
    
    stop_motors()


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

@app.route('/control', methods=['POST', 'OPTIONS'])
def control():
    if request.method == 'OPTIONS':
        return add_cors_headers(make_response())
        
    try:
        command = request.json.get('command')
        speed = 100  # Maximum speed (100% duty cycle)
        
        if command == 'forward':
            set_motor_left(-speed)
            set_motor_right(speed)
        elif command == 'backward':
            set_motor_left(speed)
            set_motor_right(-speed)
        elif command == 'left':
            set_motor_left(speed)
            set_motor_right(speed)
        elif command == 'right':
            set_motor_left(-speed)
            set_motor_right(-speed)
        elif command == 'stop':
            stop_motors()
        else:
            return add_cors_headers(jsonify({'error': 'Invalid command'})), 400
            
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
        global obstacle_avoidance_active, obstacle_avoidance_thread
        
        data = request.json
        enabled = data.get('enabled', False)
        
        if enabled and not obstacle_avoidance_active:
            obstacle_avoidance_active = True
            obstacle_avoidance_thread = Thread(target=obstacle_avoidance_loop)
            obstacle_avoidance_thread.start()
            message = "Obstacle avoidance mode enabled"
        elif not enabled and obstacle_avoidance_active:
            obstacle_avoidance_active = False
            if obstacle_avoidance_thread:
                obstacle_avoidance_thread.join(timeout=2.0)
            stop_motors()
            message = "Obstacle avoidance mode disabled"
        else:
            message = "No change in obstacle avoidance mode"
            
        return add_cors_headers(jsonify({
            'status': 'success',
            'message': message,
            'enabled': obstacle_avoidance_active
        }))
        
    except Exception as e:
        logger.error(f"Obstacle mode error: {e}")
        return add_cors_headers(jsonify({'error': str(e)})), 500

# Handle OPTIONS requests for CORS
@app.route('/', methods=['OPTIONS'])
@app.route('/status', methods=['OPTIONS'])
@app.route('/stream', methods=['OPTIONS'])
def handle_options():
    response = make_response()
    return add_cors_headers(response)

if __name__ == '__main__':
    try:
        # Initialize camera
        camera = setup_camera()
        if camera is None:
            raise Exception("Failed to initialize camera")
            
        # Setup GPIO
        pwm_a, pwm_b = setup_gpio()
        
        logger.info("Starting web server...")
        app.run(host='0.0.0.0', port=8000, threaded=True)
        
    except Exception as e:
        logger.error(f"Startup error: {e}")
    
    finally:
        if camera is not None:
            camera.release()
        stop_motors()
        if obstacle_avoidance_active:
            obstacle_avoidance_active = False
            if obstacle_avoidance_thread:
                obstacle_avoidance_thread.join(timeout=2.0)
        GPIO.cleanup()
