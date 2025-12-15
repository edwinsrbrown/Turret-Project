from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.request
import RPi.GPIO as GPIO
import json
import time
import math
from shifter import Shifter

# ------------------------
# Utility functions
# ------------------------

def parsePOSTdata(data):
    data_dict = {}
    data_pairs = data.split('&')
    for pair in data_pairs:
        key_val = pair.split('=')
        if len(key_val) == 2:
            data_dict[key_val[0]] = key_val[1]
    return data_dict


def shortest_angle_delta(target, current):
    delta = target - current
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


# ------------------------
# GPIO setup
# ------------------------

dataPin = 16
latchPin = 20
clockPin = 21
laserPin = 4

GPIO.setmode(GPIO.BCM)

GPIO.setup(dataPin, GPIO.OUT)
GPIO.setup(latchPin, GPIO.OUT)
GPIO.setup(clockPin, GPIO.OUT)

GPIO.setup(laserPin, GPIO.OUT)
GPIO.output(laserPin, GPIO.LOW)

# ------------------------
# Motor setup
# ------------------------

step_delay = 0.003
lr_steps = 512 / 360
ud_steps = 512 / 360

sequence = [
    0b0001, 0b0011, 0b0010, 0b0110,
    0b0100, 0b1100, 0b1000, 0b1001
]

shift = Shifter(dataPin, latchPin, clockPin)

lr_pos = 0.0
ud_pos = 0.0


def step_motor(seq_index, motor):
    pattern = sequence[seq_index]
    if motor == 0:
        out_byte = pattern
    else:
        out_byte = pattern << 4
    shift.shiftByte(out_byte)


def move_motor_degs(motor, current_deg, target_deg, steps_per_deg):
    # Clamp to physical limits
    target_deg = max(-90, min(90, target_deg))

    delta = shortest_angle_delta(target_deg, current_deg)
    steps = int(abs(delta) * steps_per_deg)

    if delta > 0:
        direction = -1
    else:
        direction = 1

    if direction > 0:
        seq = range(8)
    else:
        seq = range(7, -1, -1)

    for _ in range(steps):
        for i in seq:
            step_motor(i, motor)
            time.sleep(step_delay)

    return normalize_angle(current_deg + delta)


def zero_motor(motor):
    global lr_pos, ud_pos
    for _ in range(200):
        step_motor(0, motor)
        time.sleep(step_delay)

    if motor == 0:
        lr_pos = 0.0
    else:
        ud_pos = 0.0


# ------------------------
# Autonomous math
# ------------------------

def compute_angles(my_r, my_theta, targ_r, targ_theta, targ_z=0):
    dx = targ_r * math.cos(targ_theta) - my_r * math.cos(my_theta)
    dy = targ_r * math.sin(targ_theta) - my_r * math.sin(my_theta)

    lr_angle = math.degrees(math.atan2(dy, dx))
    ud_angle = math.degrees(math.atan2(targ_z, math.sqrt(dx * dx + dy * dy)))

    return lr_angle, ud_angle


# ------------------------
# Web server
# ------------------------

laser_status = "OFF"
msg = ""


class TurretHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        global lr_pos, ud_pos, laser_status, msg

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

<form method="POST">
  <button name="action" value="toggle_laser">Toggle Laser</button>
</form>

<hr>

<h3>Manual Move</h3>
<form method="POST">
  <label>Left / Right (deg): <span id="lrdisp">{lr_pos:.1f}</span></label>
  <input type="range" name="lr_deg" min="-90" max="90" step="0.1"
         value="{lr_pos}"
         oninput="lrdisp.innerText=this.value">
  <br><br>

  <label>Up / Down (deg): <span id="uddisp">{ud_pos:.1f}</span></label>
  <input type="range" name="ud_deg" min="-90" max="90" step="0.1"
         value="{ud_pos}"
         oninput="uddisp.innerText=this.value">
  <br><br>

  <button name="action" value="move_angles">Move to Angles</button>
  <button name="action" value="zero_motors">Zero Motors</button>
</form>

<hr>

<h3>Autonomous Targeting</h3>
<form method="POST">
  <label>positions.json URL:
    <input type="text" name="json_url"
           value="http://192.168.1.254:8000/positions.json" size="50">
  </label>
  <br><br>

  <label>Your team number:
    <input type="text" name="team_id" value="1" size="6">
  </label>
  <br><br>

  <button name="action" value="start_autonomous">Start Autonomous</button>
</form>

<hr>
<p>{msg}</p>

</body>
</html>
"""
        self.respond(html)

    def do_POST(self):
        global laser_status, msg, lr_pos, ud_pos

        length = int(self.headers.get("Content-Length"))
        post_data = self.rfile.read(length).decode()
        data = parsePOSTdata(post_data)
        action = data.get("action", "")

        if action == "toggle_laser":
            if laser_status == "OFF":
                GPIO.output(laserPin, GPIO.HIGH)
                laser_status = "ON"
            else:
                GPIO.output(laserPin, GPIO.LOW)
                laser_status = "OFF"
            msg = "Laser toggled"

        elif action == "move_angles":
            try:
                lr_pos = move_motor_degs(0, lr_pos, float(data["lr_deg"]), lr_steps)
                ud_pos = move_motor_degs(1, ud_pos, float(data["ud_deg"]), ud_steps)
                msg = f"Moved to LR={lr_pos:.1f}, UD={ud_pos:.1f}"
            except:
                msg = "Invalid angle input"

        elif action == "zero_motors":
            zero_motor(0)
            zero_motor(1)
            msg = "Motors zeroed"

        elif action == "start_autonomous":
            msg = self.autonomous_sequence(data)

        self.do_GET()

    def autonomous_sequence(self, data):
        global lr_pos, ud_pos

        try:
            url = data["json_url"].replace("%3A", ":").replace("%2F", "/")
            team_id = data["team_id"]

            with urllib.request.urlopen(url) as f:
                js = json.loads(f.read().decode())

            my_r = js["turrets"][team_id]["r"]
            my_theta = js["turrets"][team_id]["theta"]

            targets = []

            for tid, vals in js["turrets"].items():
                if tid != team_id:
                    targets.append((vals["r"], vals["theta"], 0))

            for gl in js["globes"]:
                targets.append((gl["r"], gl["theta"], gl["z"]))

            for r, th, z in targets:
                lr_t, ud_t = compute_angles(my_r, my_theta, r, th, z)
                lr_pos = move_motor_degs(0, lr_pos, lr_t, lr_steps)
                ud_pos = move_motor_degs(1, ud_pos, ud_t, ud_steps)

                GPIO.output(laserPin, GPIO.HIGH)
                time.sleep(3)
                GPIO.output(laserPin, GPIO.LOW)

            return "Autonomous sequence complete"

        except Exception as e:
            return f"Autonomous error: {e}"

    def respond(self, html):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(html.encode("utf-8"))


# ------------------------
# Run server
# ------------------------

if __name__ == "__main__":
    try:
        server = HTTPServer(("0.0.0.0", 8080), TurretHandler)
        print("Serving on port 8080...")
        server.serve_forever()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Server stopped.")
