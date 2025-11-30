from http.server import HTTPServer, BaseHTTPRequestHandler
import RPi.GPIO as GPIO
import time
import math
import json
import urllib.request
from shifter import Shifter 

def parsePOSTdata(data):
    data_dict = {}
    data_pairs = data.split('&')
    
    for pair in data_pairs:
        key_val = pair.split('=')
        if len(key_val) == 2:
            data_dict[key_val[0]] = key_val[1]

    return data_dict

class Stepper:
    num_steppers = 0
    shifter_outputs = 0  
    
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay = 1200
    
    steps_per_degree = 4096.0 / 360.0 

    def __init__(self, shifter):
        self.s = shifter
        self.angle = 0.0
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers
        
        Stepper.num_steppers += 1

    def __sgn(self, x):
        if x == 0: return 0
        else: return int(abs(x)/x)

    def __step(self, dir):
        self.step_state += dir
        self.step_state %= 8
        
        mask = ~(0b1111 << self.shifter_bit_start)
        Stepper.shifter_outputs &= mask
        Stepper.shifter_outputs |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        
        self.s.shiftByte(Stepper.shifter_outputs)

        self.angle = (self.angle + dir / Stepper.steps_per_degree) % 360.0

    def rotate_relative(self, delta):
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        direction = self.__sgn(delta)
        
        for _ in range(numSteps):
            self.__step(direction)
            time.sleep(Stepper.delay / 1e6)

    def go_angle(self, angle):
        angle %= 360.0
        current = self.angle
        delta = angle - current

        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        self.rotate_relative(delta)

    def zero(self):
        self.angle = 0.0

PORT = 8080
POWER_PIN = 4  #laser pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(POWER_PIN, GPIO.OUT)
GPIO.output(POWER_PIN, GPIO.LOW)

LASER_ON = False

SHIFTER_DATA = 16
SHIFTER_LATCH = 20
SHIFTER_CLOCK = 21

shifter = Shifter(data=SHIFTER_DATA, latch=SHIFTER_LATCH, clock=SHIFTER_CLOCK)

m_ud = Stepper(shifter) 
m_lr = Stepper(shifter)

#targetting
def deg_from_rad(x):
    return x * 180.0 / math.pi

def normalize_deg(a):
    a %= 360.0
    return a

def compute_target_angles(my_r, my_theta, target_r, target_theta, target_z=0.0, turret_height=0.0):
    
    # Calculate vector to target relative to me
    my_x = my_r * math.cos(my_theta)
    my_y = my_r * math.sin(my_theta)
    
    target_x = target_r * math.cos(target_theta)
    target_y = target_r * math.sin(target_theta)
    
    dx = target_x - my_x
    dy = target_y - my_y
    
    # abs angle to target
    abs_angle_rad = math.atan2(dy, dx)
    
    # convert rad to degrees
    lr_cmd = normalize_deg(deg_from_rad(abs_angle_rad))
    
    # up/down control
    dist_horiz = math.sqrt(dx**2 + dy**2)
    dist_horiz = max(dist_horiz, 1e-6) # prevent div by zero
    
    vert = target_z - turret_height
    ud_cmd = deg_from_rad(math.atan2(vert, dist_horiz))
    
    return lr_cmd, ud_cmd

def fire_laser_sequence():
    global LASER_ON
    GPIO.output(POWER_PIN, GPIO.HIGH)
    LASER_ON = True
    time.sleep(3.0)
    GPIO.output(POWER_PIN, GPIO.LOW)
    LASER_ON = False

class TurretHandler(BaseHTTPRequestHandler):
    
    def _generate_html(self, msg=""):
        lr = round(m_lr.angle, 1)
        ud = round(m_ud.angle, 1)
        laser_status = "ON" if LASER_ON else "OFF"
        
        html = f"""
<!DOCTYPE html>
<html>
<head><title>Turret Control</title></head>
<body>
    <h1>Turret Control</h1>
    <p><b>Laser Status:</b> {laser_status}</p>
    
    <form method="POST">
        <button name="action" value="toggle_laser">Toggle Laser</button>
    </form>
    
    <hr>
    
    <h3>Manual Control</h3>
    <form method="POST">
        LR Angle: <input type="text" name="lr_deg" value="{lr}" size="5">
        UD Angle: <input type="text" name="ud_deg" value="{ud}" size="5">
        <br><br>
        <button name="action" value="move_angles">Move Motors</button>
        <button name="action" value="zero_motors">Zero Motors</button>
    </form>
    
    <hr>
    
    <h3>Autonomous</h3>
    <form method="POST">
        Team ID: <input type="text" name="team_id" value="1" size="5">
        <br><br>
        <button name="action" value="start_autonomous">Start Sequence</button>
    </form>
    
    <hr>
    <p>{msg}</p>
</body>
</html>
"""
        return html

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(self._generate_html().encode("utf-8"))

    def do_POST(self):
        global LASER_ON
        
        length = int(self.headers.get('Content-Length', 0))
        raw = self.rfile.read(length).decode('utf-8')
        data = parsePOSTdata(raw)
        
        action = data.get('action', '')
        msg = ""
        
        # toggling the laser on/off
        if action == "toggle_laser":
            LASER_ON = not LASER_ON
            GPIO.output(POWER_PIN, GPIO.HIGH if LASER_ON else GPIO.LOW)
            msg = f"Laser set to {LASER_ON}"

        # moving the motors
        elif action == "move_angles":
            try:
                target_lr = float(data.get('lr_deg', m_lr.angle))
                target_ud = float(data.get('ud_deg', m_ud.angle))
                m_lr.go_angle(target_lr)
                m_ud.go_angle(target_ud)
                msg = "Moved to coordinates."
            except ValueError:
                msg = "Invalid angle input."

        # 3. ZERO MOTORS
        elif action == "zero_motors":
            m_lr.zero()
            m_ud.zero()
            msg = "Motors zeroed."

        # 4. AUTONOMOUS
        elif action == "start_autonomous":
            try:
                team_id = str(data.get('team_id', '1'))
                url = "http://192.168.1.254:8000/positions.json"
                
                # Fetch JSON
                with urllib.request.urlopen(url) as r:
                    j = json.loads(r.read().decode('utf-8'))
                
                if team_id not in j['turrets']:
                    msg = "Team ID not found in JSON."
                else:
                    # Get my data
                    my_data = j['turrets'][team_id]
                    my_r = float(my_data['r'])
                    my_theta = float(my_data['theta'])
                    
                    targets = []
                    
                    # Add Enemy Turrets
                    for tid, tdata in j['turrets'].items():
                        if tid != team_id:
                            targets.append((float(tdata['r']), float(tdata['theta']), 0.0))
                            
                    # Add Globes
                    for globe in j['globes']:
                        targets.append((float(globe['r']), float(globe['theta']), float(globe['z'])))
                    
                    # Execute Sequence
                    for tr, tt, tz in targets:
                        lr, ud = compute_target_angles(my_r, my_theta, tr, tt, tz)
                        m_lr.go_angle(lr)
                        m_ud.go_angle(ud)
                        time.sleep(0.5) # Stabilize
                        fire_laser_sequence()
                        
                    msg = "Autonomous sequence finished."
                    
            except Exception as e:
                msg = f"Autonomous error: {e}"

        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(self._generate_html(msg).encode("utf-8"))

def run_server():
    server_address = ('', PORT)
    httpd = HTTPServer(server_address, TurretHandler)
    print(f"Turret Server running on port {PORT}...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up.")

if __name__ == '__main__':
    run_server()
