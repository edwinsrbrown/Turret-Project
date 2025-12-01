from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.request
import RPi.GPIO as GPIO
import json
import time
import math
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
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay_us = 1200
    steps_per_degree = 1024/360.0
    _shared_register = 0 

    def __init__(self, shifter, shifter_bit_start):
        self.s = shifter
        self.shifter_bit_start = shifter_bit_start
        self.step_state = 0
        self.angle = 0.0

    def _sgn(self, x):
        if x == 0: return 0
        return int(abs(x)/x)

    def __step(self, direction):
        self.step_state = (self.step_state + direction) % 8
        Stepper._shared_register &= ~(0b1111 << self.shifter_bit_start)
        Stepper._shared_register |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        self.s.shiftByte(Stepper._shared_register)
        self.angle = (self.angle + (direction / Stepper.steps_per_degree)) % 360.0

    def rotate_relative(self, delta_deg):
        steps = int(abs(delta_deg) * Stepper.steps_per_degree)
        direction = self._sgn(delta_deg)
        for _ in range(steps):
            self.__step(direction)
            time.sleep(Stepper.delay_us / 1e6)

    def go_angle(self, angle_deg):
        angle_deg = angle_deg % 360.0
        current = self.angle
        delta = angle_deg - current
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        self.rotate_relative(delta)

    def zero(self):
        self.angle = 0.0
        self.step_state = 0
        Stepper._shared_register &= ~(0b1111 << self.shifter_bit_start)
        Stepper._shared_register |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        self.s.shiftByte(Stepper._shared_register)

PORT = 8080
POWER_PIN = 4 #for laser

GPIO.setmode(GPIO.BCM)
GPIO.setup(POWER_PIN, GPIO.OUT)
GPIO.output(POWER_PIN, GPIO.LOW)

LASER_ON = False

SHIFTER_DATA = 16
SHIFTER_LATCH = 20
SHIFTER_CLOCK = 21

shifter = Shifter(data=SHIFTER_DATA, latch=SHIFTER_LATCH, clock=SHIFTER_CLOCK)

m_lr = Stepper(shifter, shifter_bit_start=0)   # Left/Right (azimuth)
m_ud = Stepper(shifter, shifter_bit_start=4)   # Up/Down (altitude)

def deg_from_rad(x):
    return x * 180.0 / math.pi

def normalize_deg(a):
    a %= 360.0
    return a

def compute_target_angles(my_r, my_theta, target_r, target_theta, target_z=0.0, turret_height=0.0):
    lr_cmd = normalize_deg(deg_from_rad(target_theta))

    delta_theta = abs(target_theta - my_theta)
    delta_theta = (delta_theta + math.pi) % (2*math.pi) - math.pi

    horiz = math.sqrt(
        my_r**2 + target_r**2 - 2*my_r*target_r*math.cos(delta_theta)
    )
    horiz = max(horiz, 1e-6)

    vert = target_z - turret_height
    ud_cmd = deg_from_rad(math.atan2(vert, horiz))

    return lr_cmd, ud_cmd

def fire_laser(duration_s=3.0):
    global LASER_ON
    GPIO.output(POWER_PIN, GPIO.HIGH)
    LASER_ON = True
    time.sleep(duration_s)
    GPIO.output(POWER_PIN, GPIO.LOW)
    LASER_ON = False

class TurretHandler(BaseHTTPRequestHandler):

    def _generate_html(self, msg=""):
        lr = round(m_lr.angle, 1)
        raw_ud = m_ud.angle
        if raw_ud > 180:
            raw_ud -= 360
        ud = round(raw_ud, 1)
        laser_status = "ON" if LASER_ON else "OFF"

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
      <label>Left / Right (deg): <span id="lrdisp">{lr}</span></label>
      <input id="lr" name="lr_deg" type="range" min="-90" max="90" step="0.1"
             value="{lr}" oninput="document.getElementById('lrdisp').innerText=this.value"/>
      <br><br>

      <label>Up / Down (deg): <span id="uddisp">{ud}</span></label>
      <input id="ud" name="ud_deg" type="range" min="-90" max="90" step="0.1"
             value="{ud}" oninput="document.getElementById('uddisp').innerText=this.value"/>
      <br><br>

      <button name="action" value="move_angles">Move to Angles</button>
      <button name="action" value="zero_motors">Zero Motors</button>
    </form>

    <hr>

    <h3>Autonomous Targeting</h3>
    <form method="POST">
      <label>positions.json URL:
        <input type="text" name="json_url" value="http://192.168.1.254:8000/positions.json" size="50"/>
      </label>
      <br><br>
      <label>Your team number:
        <input type="text" name="team_id" value="1" size="6"/>
      </label>
      <br><br>
      <button name="action" value="start_autonomous">Start Autonomous</button>
    </form>

    <hr>
    <p>{msg}</p>
</body>
</html>
"""
        return html

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type","text/html")
        self.end_headers()
        self.wfile.write(self._generate_html().encode("utf-8"))

    def do_POST(self):
        global LASER_ON

        length = int(self.headers.get('Content-Length',0))
        raw = self.rfile.read(length).decode('utf-8')
        
        data = parsePOSTdata(raw)

        action = data.get('action',"")
        msg = ""

        if action == "toggle_laser":
            LASER_ON = not LASER_ON
            GPIO.output(POWER_PIN, GPIO.HIGH if LASER_ON else GPIO.LOW)
            msg = f"Laser toggled to {LASER_ON}"

        elif action == "move_angles":
            try:
                lr = float(data.get("lr_deg", m_lr.angle))
                ud = float(data.get("ud_deg", m_ud.angle))
                msg = f"Moving to LR={lr}, UD={ud}..."
                m_lr.go_angle(lr)
                m_ud.go_angle(ud)
                msg += " done."
            except Exception as e:
                msg = f"Move error: {e}"

        elif action == "zero_motors":
            m_lr.zero()
            m_ud.zero()
            LASER_ON = False
            GPIO.output(POWER_PIN, GPIO.LOW)
            msg = "Motors zeroed. Laser off."

        elif action == "start_autonomous":
            try:
                url = data.get("json_url","")
                team = str(int(data.get("team_id","1")))

                # Still requires urllib.request for the GET
                with urllib.request.urlopen(url) as r:
                    j = json.loads(r.read().decode('utf-8'))

                turrets = j["turrets"]
                globes = j["globes"]

                if team not in turrets:
                    msg = f"Team {team} not found."
                else:
                    my = turrets[team]
                    my_r = my["r"]
                    my_t = my["theta"]

                    targets = []
                    for k,v in turrets.items():
                        if k==team: continue
                        targets.append(("turret",k,v["r"],v["theta"],0))
                    for i,g in enumerate(globes):
                        targets.append(("globe",i,g["r"],g["theta"],g["z"]))

                    msg = f"Autonomous: {len(targets)} targets..."

                    for typ,ident,r,t,z in targets:
                        lr_cmd, ud_cmd = compute_target_angles(my_r,my_t,r,t,z)
                        m_lr.go_angle(lr_cmd)
                        m_ud.go_angle(ud_cmd)
                        fire_laser(3)
                        time.sleep(0.5)

                    msg += " done."

            except Exception as e:
                msg = f"Autonomous error: {e}"

        else:
            msg = "Unknown action."

        self.send_response(200)
        self.send_header("Content-type","text/html")
        self.end_headers()
        self.wfile.write(self._generate_html(msg).encode("utf-8"))

def run_server():
    httpd = HTTPServer(("",PORT), TurretHandler)
    print(f"Turret server running at http://<pi-ip>:{PORT}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up.")

if __name__ == "__main__":
    run_server()
