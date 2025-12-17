from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.request
import RPi.GPIO as GPIO
import json
import time
import math
from shifter import Shifter

def parsePOSTdata(data): # parse function from lecture
    data_dict = {}
    data_pairs = data.split('&')
    for pair in data_pairs:
        key_val = pair.split('=')
        if len(key_val) == 2:
            data_dict[key_val[0]] = key_val[1]
    return data_dict

# pin set-up

dataPin = 16
latchPin = 20
clockPin = 21
laserPin = 4

GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)

GPIO.setup(dataPin, GPIO.OUT)
GPIO.setup(latchPin, GPIO.OUT)
GPIO.setup(clockPin, GPIO.OUT)

GPIO.setup(laserPin, GPIO.OUT)
GPIO.output(laserPin, GPIO.LOW)

# motor steps
step_delay = 0.003
lr_steps = 4096/360   
ud_steps = 4096/360  

sequence = [0b0001, 0b0011, 0b0010, 0b0110,
            0b0100, 0b1100, 0b1000, 0b1001]

shift = Shifter(dataPin, latchPin, clockPin)

# initializing motor positions
lr_pos = 0.0
ud_pos = 0.0

# ===== ADDED (autonomous control flags) =====
autonomous_running = False
autonomous_stop = False
# ===========================================

def step_motor(seq_index, motor): 
    pattern = sequence[seq_index]
    out_byte = pattern if motor == 0 else pattern << 4
    shift.shiftByte(out_byte)

def shortest_angle_delta(target, current):
    delta = target - current
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta

def move_motor_degs(motor, current_deg, target_deg, steps_per_deg):
    delta = shortest_angle_delta(target_deg, current_deg)

    max_angle = 90
    desired = max(-max_angle, min(max_angle, current_deg + delta))
    delta = desired - current_deg

    steps = int(abs(delta) * steps_per_deg)
    direction = 1 if delta < 0 else -1
    seq = range(8) if direction > 0 else range(7, -1, -1)

    for i in range(steps):
        # ===== ADDED (mid-move stop) =====
        if autonomous_stop:
            return current_deg
        # =================================

        step_motor(seq[i % 8], motor)
        time.sleep(step_delay)

    return current_deg + delta

def zero_motor(motor):
    global lr_pos, ud_pos
    for _ in range(200):
        step_motor(0, motor)
        time.sleep(step_delay)
    if motor == 0:
        lr_pos = 0.0
    else:
        ud_pos = 0.0
    return lr_pos, ud_pos

def compute_angles(my_r, my_theta, targ_r, targ_theta, targ_z=0):
    xm = my_r * math.cos(my_theta)
    ym = my_r * math.sin(my_theta)

    xt = targ_r * math.cos(targ_theta)
    yt = targ_r * math.sin(targ_theta)

    dx = xt - xm
    dy = yt - ym

    bearing = math.degrees(math.atan2(dy, dx))
    forward = math.degrees(my_theta) + 180.0
    lr_angle = forward - bearing

    while lr_angle > 180:
        lr_angle -= 360
    while lr_angle < -180:
        lr_angle += 360

    ud_angle = math.degrees(
        math.atan2(targ_z, math.sqrt(dx*dx + dy*dy))
    )

    return lr_angle, ud_angle

laser_status = "OFF"
msg = ""

class TurretHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        global lr_pos, ud_pos, laser_status, autonomous_running

# HTML generated with LLM assistance
        html = f"""
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Turret Control</title>
</head>
<body>
    <h1>Turret Control</h1>

    <p><b>Laser:</b> {laser_status}</p>
    <p>
      <b>Autonomous:</b>
      <span style="color:{'red' if autonomous_running else 'green'};">
        {'RUNNING' if autonomous_running else 'IDLE'}
      </span>
    </p>

    <form method="POST">
      <button name="action" value="toggle_laser">Toggle Laser</button>
    </form>

    <hr>

    <h3>Manual Move</h3>
    <form method="POST">
      <label>
        Left / Right (deg):
        <span style="display:inline-block; width:60px; text-align:right;">
          {lr_pos:.1f}
        </span>
      </label>
      <input name="lr_deg" type="range" min="-90" max="90" step="0.1"
             value="{lr_pos}">
      <br><br>

      <label>
        Up / Down (deg):
        <span style="display:inline-block; width:60px; text-align:right;">
          {ud_pos:.1f}
        </span>
      </label>
      <input name="ud_deg" type="range" min="-90" max="90" step="0.1"
             value="{ud_pos}">
      <br><br>

      <button name="action" value="move_angles">Move to Angles</button>
      <button name="action" value="zero_motors">Zero Motors</button>
    </form>

    <hr>

    <h3>Autonomous Targeting</h3>
    <form method="POST">
      <label>positions.json URL:
        <input type="text" name="json_url"
               value="http://192.168.1.254:8000/positions.json" size="50"/>
      </label><br><br>
      <label>Your team number:
        <input type="text" name="team_id" value="8" size="6"/>
      </label><br><br>
      <button name="action" value="start_autonomous">Start Autonomous</button>
    </form>

    <form method="POST">
      <button name="action" value="stop_autonomous"
              style="background-color:#c00;color:white;">
        STOP Autonomous
      </button>
    </form>

    <hr>
    <p>{msg}</p>
</body>
</html>
"""
        self.respond(html)

    def do_POST(self):
        global laser_status, msg, lr_pos, ud_pos, autonomous_stop

        length = int(self.headers.get("Content-Length"))
        data = parsePOSTdata(self.rfile.read(length).decode())
        action = data.get("action", "")

        if action == "toggle_laser":
            laser_status = "OFF" if laser_status == "ON" else "ON"
            GPIO.output(laserPin,
                        GPIO.HIGH if laser_status == "ON" else GPIO.LOW)
            msg = "Laser toggled"

        elif action == "move_angles":
            lr_pos = move_motor_degs(0, lr_pos, float(data["lr_deg"]), lr_steps)
            ud_pos = move_motor_degs(1, ud_pos, float(data["ud_deg"]), ud_steps)
            msg = "Manual move complete"

        elif action == "zero_motors":
            zero_motor(0)
            zero_motor(1)
            msg = "Motors zeroed"

        elif action == "start_autonomous":
            msg = self.autonomous_sequence(data)

        elif action == "stop_autonomous":
            autonomous_stop = True
            msg = "Autonomous stop requested"

        self.do_GET()

    def autonomous_sequence(self, data):
        global lr_pos, ud_pos, autonomous_running, autonomous_stop

        autonomous_running = True
        autonomous_stop = False

        try:
            with urllib.request.urlopen(data["json_url"]) as f:
                js = json.loads(f.read().decode())

            my = js["turrets"][data["team_id"]]

            targets = [(v["r"], v["theta"], 0)
                       for k, v in js["turrets"].items()
                       if k != data["team_id"]]
            targets += [(g["r"], g["theta"], g["z"]) for g in js["globes"]]

            for r, th, z in targets:
                if autonomous_stop:
                    autonomous_running = False
                    return "Autonomous stopped"

                lr_t, ud_t = compute_angles(my["r"], my["theta"], r, th, z)
                lr_pos = move_motor_degs(0, lr_pos, lr_t, lr_steps)
                ud_pos = move_motor_degs(1, ud_pos, ud_t, ud_steps)

                if autonomous_stop:
                    autonomous_running = False
                    return "Autonomous stopped"

                GPIO.output(laserPin, GPIO.HIGH)
                time.sleep(3)
                GPIO.output(laserPin, GPIO.LOW)

            autonomous_running = False
            return "Autonomous complete"

        except Exception as e:
            autonomous_running = False
            return f"Autonomous error: {e}"

    def respond(self, html):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(html.encode("utf-8"))

# run the code
if __name__ == "__main__":
    try:
        server = HTTPServer(("0.0.0.0", 8080), TurretHandler)
        print("Serving on port 8080...")
        server.serve_forever()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Server stopped.")
