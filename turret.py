#imports
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
GPIO.setup(dataPin, GPIO.OUT)
GPIO.setup(latchPin, GPIO.OUT)
GPIO.setup(clockPin, GPIO.OUT)
GPIO.setup(laserPin, GPIO.OUT)
GPIO.output(laserPin, GPIO.LOW)

# motor steps
step_delay = 0.003
lr_steps = 4096/360  #conversion for left/right motor
ud_steps = 4096/360  #conversion for up/down motor

sequence = [0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001] # stepper motor sequence

shift = Shifter(dataPin, latchPin, clockPin) #initialize shifter object

# initializing motor positions
lr_pos = 0.0
ud_pos = 0.0

# sets motors to corresponding bits (motor 0 is lower 4 bits, motor 1 is upper 4 bits)
def step_motor(seq_index, motor): 
    pattern = sequence[seq_index] #picks pattern from step sequence

    if motor == 0:
        out_byte = pattern
    else:
        out_byte = pattern << 4

    shift.shiftByte(out_byte) #send 8 bits to the shift register

def shortest_angle_delta(target, current):
    delta = target - current
    #wrap angles so motor never rotates past +/- 180 degrees
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta

def move_motor_degs(motor, current_deg, target_deg, steps_per_deg):
    delta = shortest_angle_delta(target_deg, current_deg) #shortest rotation direction

    max_angle = 90 #motor can't go past 90 degrees either way
    desired = max(-max_angle, min(max_angle, current_deg + delta)) #clamps angle to +/- 90 degrees
    delta = desired - current_deg #new angle that is clamped

    steps = int(abs(delta) * steps_per_deg)

    #rotation direction
    if delta < 0:
        direction = 1
    else:
        direction = -1
    #forward or reverse stepping order
    if direction > 0:
        seq = range(8)
    else:
        range(7, -1, -1)

    #steps motor one step at a time
    for i in range(steps):
        step_motor(seq[i % 8], motor)
        time.sleep(step_delay)

    return current_deg + delta

# makes current position the "zero" position
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

#autonomous part
def compute_angles(my_r, my_theta, targ_r, targ_theta, targ_z=0):
    #polar to cartesian for my turret position
    xm = my_r * math.cos(my_theta)
    ym = my_r * math.sin(my_theta)

    #polar to cartesian for target turret position
    xt = targ_r * math.cos(targ_theta)
    yt = targ_r * math.sin(targ_theta)

    #x and y vectors from my turret to target turret
    dx = xt - xm
    dy = yt - ym

    #absolute angle to target
    abs_angle = math.degrees(math.atan2(dy, dx))

    #my turret's facing direction
    forward = math.degrees(my_theta) + 180.0

    # left/right motor angle
    lr_angle = forward - abs_angle

    # wrap to -180 to +180
    while lr_angle > 180:
        lr_angle -= 360
    while lr_angle < -180:
        lr_angle += 360

    # up/down angle
    ud_angle = math.degrees(
        math.atan2(targ_z, math.sqrt(dx*dx + dy*dy))
    )

    return lr_angle, ud_angle

# HTML generated with LLM assistance
laser_status = "OFF"
msg = ""

class TurretHandler(BaseHTTPRequestHandler):

    #sends the HTML page to display laser state, sliders, and autonomous functions
    def do_GET(self):
        global lr_pos, ud_pos, laser_status

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
      <label>
        Left / Right (deg):
        <span id="lrdisp"
              style="display:inline-block; width:60px; text-align:right;">
          {lr_pos:.1f}
        </span>
      </label>
      <input id="lr" name="lr_deg" type="range" min="-90" max="90" step="0.1"
             value="{lr_pos}"
             oninput="document.getElementById('lrdisp').innerText=this.value"/>
      <br><br>

      <label>
        Up / Down (deg):
        <span id="uddisp"
              style="display:inline-block; width:60px; text-align:right;">
          {ud_pos:.1f}
        </span>
      </label>
      <input id="ud" name="ud_deg" type="range" min="-90" max="90" step="0.1"
             value="{ud_pos}"
             oninput="document.getElementById('uddisp').innerText=this.value"/>
      <br><br>

      <button name="action" value="move_angles">Move to Angles</button>
      <button name="action" value="zero_motors">Zero Motors</button>
    </form>

    <hr>

    <h3>Autonomous Targeting</h3>
    <form method="POST">
      <label>positions.json URL:
        <input type="text" name="json_url"
               value="http://192.168.1.254:8000/positions.json"
               size="50"/>
      </label>
      <br><br>
      <label>Your team number:
        <input type="text" name="team_id" value="8" size="6"/>
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

    #runs when buttons are clicked
    def do_POST(self):
        global laser_status
        global msg
        global lr_pos
        global ud_pos

        length = int(self.headers.get("Content-Length"))
        post_data = self.rfile.read(length).decode()

        data = parsePOSTdata(post_data)

        action = data.get("action", "") #determines which button was clicked and assigns action to variable

        if action == "toggle_laser":
            if laser_status == "OFF":
                GPIO.output(laserPin, GPIO.HIGH)
                laser_status = "ON"
            else:
                GPIO.output(laserPin, GPIO.LOW)
                laser_status = "OFF"
            msg = "Laser toggled"

        elif action == "move_angles":
            new_lr = float(data["lr_deg"])
            new_ud = float(data["ud_deg"])

            lr_pos = move_motor_degs(0, lr_pos, new_lr, lr_steps)
            ud_pos = move_motor_degs(1, ud_pos, new_ud, ud_steps)

            msg = f"Moved to LR={lr_pos:.1f} and UD={ud_pos:.1f}"

        elif action == "zero_motors":
            zero_motor(0)
            zero_motor(1)
            msg = "Motors are zeroed"

        elif action == "start_autonomous":
            msg = self.autonomous_sequence(data)
        
        try:
            self.do_GET()
        except BrokenPipeError:
            pass

    def autonomous_sequence(self, data):
        global lr_pos, ud_pos

        try:
            url = data["json_url"]
            team_id = data["team_id"]

            url = url.replace("%3A", ":") # fixes for parsing
            url = url.replace("%2F", "/") #fixes for parsing

            #downloads the position.json file
            with urllib.request.urlopen(url) as f:
                js = json.loads(f.read().decode())

            my_r = js["turrets"][team_id]["r"]
            my_theta = js["turrets"][team_id]["theta"]

            targets = []

            for tid, vals in js["turrets"].items():
                if tid == team_id:
                    continue
                targets.append((vals["r"], vals["theta"], 0))

            for gl in js["globes"]:
                targets.append((gl["r"], gl["theta"], gl["z"]))

            for (r, th, z) in targets:

                lr_t, ud_t = compute_angles(my_r, my_theta, r, th, z)

                lr_pos = move_motor_degs(0, lr_pos, lr_t, lr_steps)
                ud_pos = move_motor_degs(1, ud_pos, ud_t, ud_steps)

                time.sleep(0.5)
                GPIO.output(laserPin, GPIO.HIGH)
                time.sleep(3)
                GPIO.output(laserPin, GPIO.LOW)
                time.sleep(0.5)

            print(targets)
    
            return "Autonomous sequence complete"

        except Exception as e:
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
