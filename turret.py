#!/usr/bin/env python3
# turret_server.py
# Integrated HTTP + Stepper control for ENME441 Turret Project (Option B - set angles then Move)
#
# Assumptions:
# - Shifter class is available as `from shifter import Shifter`
# - m1 = azimuth motor, m2 = altitude motor
# - Single shift register controls both motors (4 bits each)
# - Laser controlled by GPIO POWER_PIN (use transistor drive recommended)

from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs
import urllib.request
import json
import time
import math
import threading
import signal
import sys

# Raspberry Pi GPIO
try:
    import RPi.GPIO as GPIO
except Exception:
    # Helpful fallback for dev machines to avoid immediate crashes.
    class DummyGPIO:
        BCM = BOARD = OUT = IN = LOW = HIGH = None
        def setmode(self, *a, **k): pass
        def setup(self, *a, **k): pass
        def output(self, *a, **k): pass
        def input(self, *a, **k): return False
        def cleanup(self): pass
    GPIO = DummyGPIO()

# Import your Shifter implementation (must be in PYTHONPATH)
from shifter import Shifter

# ---------- POST parsing ----------
def parsePOSTdata(raw):
    """
    Parse an application/x-www-form-urlencoded POST body into a dict of str->str.
    Uses parse_qs to handle URL-encoding properly.
    """
    qs = parse_qs(raw, keep_blank_values=True)
    return {k: v[0] for k, v in qs.items()}

# ---------- Stepper class (synchronous) ----------
class Stepper:
    """
    Synchronous Stepper class controlling one stepper (4 control bits) within
    a chain of shift register outputs.
    - shifter: Shifter instance with method shiftByte(int)
    - shifter_bit_start: starting bit index (0..)
    """
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]  # CCW
    delay_us = 1200  # microseconds between steps
    steps_per_degree = 4096/360.0  # if using 4096 steps/rev (modify if different)

    def __init__(self, shifter, shifter_bit_start):
        self.s = shifter
        self.shifter_bit_start = shifter_bit_start
        self.step_state = 0
        self.angle = 0.0  # degrees, 0..360
        # keep the whole register as an attribute (int)
        # We'll read/write it from the shifter object via a local variable.
        # No multiprocessing needed; synchronous usage assumed.
        self._register_state = 0

    def _set_register(self, reg_val):
        # Update internal and actually push to the shift register
        self._register_state = reg_val & 0xFFFFFFFF
        self.s.shiftByte(self._register_state)

    def _get_register(self):
        return self._register_state

    def _sgn(self, x):
        if x == 0: return 0
        return int(abs(x)/x)

    def __step(self, dir):
        """Single step in direction dir (+1 or -1)"""
        self.step_state = (self.step_state + dir) % len(Stepper.seq)
        sep = self._get_register()
        # Clear the 4 bits for this motor
        sep &= ~(0b1111 << self.shifter_bit_start)
        sep |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        self._set_register(sep)
        # update angle
        self.angle = (self.angle + (dir / Stepper.steps_per_degree)) % 360.0

    def rotate_relative(self, delta_deg):
        """Rotate relative angle delta_deg (can be negative). Runs synchronously."""
        steps = int(abs(delta_deg) * Stepper.steps_per_degree)
        dir = self._sgn(delta_deg)
        for _ in range(steps):
            self.__step(dir)
            time.sleep(Stepper.delay_us / 1e6)

    def go_angle(self, angle_deg):
        """Move to absolute angle (0..360) by the shortest path."""
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
        # optionally write a 'zero' step pattern:
        sep = self._get_register()
        sep &= ~(0b1111 << self.shifter_bit_start)
        sep |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        self._set_register(sep)

# ---------- GPIO + hardware setup ----------
PORT = 8080
POWER_PIN = 4  # laser control pin; consider using transistor; check wiring!

GPIO.setmode(GPIO.BCM)
GPIO.setup(POWER_PIN, GPIO.OUT)
GPIO.output(POWER_PIN, GPIO.LOW)

LASER_ON = False

# Setup Shifter and Steppers
# Replace data/latch/clock pin numbers with your wiring if different
SHIFTER_DATA = 16
SHIFTER_LATCH = 20
SHIFTER_CLOCK = 21

shifter = Shifter(data=SHIFTER_DATA, latch=SHIFTER_LATCH, clock=SHIFTER_CLOCK)
# Two steppers: m1 = azimuth (bits 4..7 if following your earlier pattern), m2 = altitude (bits 0..3)
# In your lab code the first instantiation used shifter_bit_start = 4*Stepper.num_steppers
# For two motors, choose:
m_az = Stepper(shifter, shifter_bit_start=4*0 + 4)  # if you prefer m1 uses the higher bits (adjust if needed)
m_alt = Stepper(shifter, shifter_bit_start=4*1 + 0) # adjust mapping if your hardware expects different ordering

# NOTE: Adjust the bit positions above to match your shifter wiring.
# If motors were instantiated in different order in your hardware, swap the bitstarts.

# ---------- Utility math functions ----------
def deg_from_rad(x):
    return x * 180.0 / math.pi

def normalize_deg(a):
    a %= 360.0
    if a < 0: a += 360.0
    return a

# ---------- Targeting logic ----------
def compute_target_angles(my_turret_r, my_turret_theta, target_r, target_theta, target_z=0.0, turret_height=0.0):
    """
    Given polar coordinates (r,theta) and globe z, compute required azimuth (deg) and altitude (deg)
    relative to global reference (0..360). We return absolute azimuth and altitude angles in degrees.
    Azimuth: angle difference (target_theta - my_theta), converted to degrees and normalized to 0..360.
    Altitude: compute horizontal separation (distance), then elevation = atan2(z - turret_height, horiz_dist)
    """
    # azimuth (global)
    az_rad = target_theta  # global angle of the target
    az_deg = deg_from_rad(az_rad)

    # convert to a relative angle that we will command as an absolute turret azimuth:
    # turret azimuth zero is assumed to be global 0 degrees; if you need to offset this, modify later.
    # We want turret to point to az = target_theta (converted to degrees).
    az_cmd = normalize_deg(az_deg)

    # horizontal distance between two points at radii r and r_turret with angles:
    # Convert to chord distance:
    # d = sqrt(r^2 + r_t^2 - 2*r*r_t*cos(delta_theta))
    delta_theta = abs(target_theta - my_turret_theta)
    # normalize delta to [0, pi]
    delta_theta = (delta_theta + math.pi) % (2*math.pi) - math.pi
    d = math.sqrt(target_r**2 + my_turret_r**2 - 2*target_r*my_turret_r*math.cos(delta_theta))

    # altitude = atan2(vertical_diff, horizontal_distance)
    vert = target_z - turret_height
    # Avoid divide-by-zero; if target sits directly at turret, set a small horizontal distance
    horiz = d if d > 1e-6 else 1e-6
    alt_rad = math.atan2(vert, horiz)
    alt_deg = deg_from_rad(alt_rad)
    # Convert altitude to a convenient positive angle (e.g., 0..90 for up)
    # We will store altitude as degrees; if negative (target below), leave as negative
    return az_cmd, alt_deg

def fire_laser(duration_s=3.0):
    global LASER_ON
    # Turn laser ON for exactly duration_s seconds, then OFF.
    # Make sure hardware driver handles current. Using transistor recommended.
    GPIO.output(POWER_PIN, GPIO.HIGH)
    LASER_ON = True
    time.sleep(duration_s)
    GPIO.output(POWER_PIN, GPIO.LOW)
    LASER_ON = False

# ---------- HTTP request handler ----------
class PowerHandler(BaseHTTPRequestHandler):

    def _generate_html(self, message=""):
        # Build a simple UI with sliders and buttons
        az = round(m_az.angle, 1)
        alt = round(m_alt.angle, 1)
        laser_status = "ON" if LASER_ON else "OFF"
        html = f"""
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Turret Project Control</title>
  <style>
    body {{ font-family: Arial, sans-serif; padding: 20px; }}
    .card {{ max-width: 640px; padding: 16px; border-radius: 8px; box-shadow: 0 0 8px rgba(0,0,0,0.08); }}
    label {{ display:block; margin-top:10px; }}
    input[type="range"] {{ width:100%; }}
    .row {{ display:flex; gap:8px; align-items:center; }}
    .btn {{ padding:8px 12px; margin-top:8px; }}
  </style>
</head>
<body>
  <div class="card">
    <h1>Turret Control</h1>
    <p><b>Laser:</b> {laser_status}</p>
    <form action="/" method="POST">
      <button name="action" value="toggle_laser" type="submit" class="btn">Toggle Laser</button>
    </form>

    <hr>

    <h3>Manual move</h3>
    <form action="/" method="POST">
      <label>Azimuth (deg): <span id="azdisp">{az}</span></label>
      <input id="az" name="az_deg" type="range" min="0" max="359.9" step="0.1" value="{az}" oninput="document.getElementById('azdisp').innerText=this.value"/>

      <label>Altitude (deg): <span id="altdisp">{alt}</span></label>
      <input id="alt" name="alt_deg" type="range" min="-90" max="90" step="0.1" value="{alt}" oninput="document.getElementById('altdisp').innerText=this.value"/>

      <div class="row">
        <button name="action" value="move_angles" type="submit" class="btn">Move to Angles</button>
        <button name="action" value="zero_motors" type="submit" class="btn">Zero Motors</button>
      </div>
    </form>

    <hr>

    <h3>Autonomous targeting</h3>
    <form action="/" method="POST">
      <label>positions.json URL:
        <input type="text" name="json_url" size="60" value="http://192.168.1.254:8000/positions.json"/>
      </label>
      <label>Your team number:
        <input type="text" name="team_id" size="6" value="1"/>
      </label>
      <div class="row">
        <button name="action" value="start_autonomous" type="submit" class="btn">Start Autonomous Targeting</button>
      </div>
    </form>

    <hr>
    <p style="color:green;">{message}</p>
    <p style="font-size:smaller;color:gray;">Note: Use transistor or current-limiting resistor for laser as required.</p>
  </div>
</body>
</html>
"""
        return html

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        html = self._generate_html()
        self.wfile.write(html.encode('utf-8'))

    def do_POST(self):
        global LASER_ON
        # Read body safely
        content_length = int(self.headers.get('Content-Length', 0))
        raw = self.rfile.read(content_length).decode('utf-8')
        parsed = parsePOSTdata(raw)

        message = ""
        action = parsed.get('action', '')

        # Toggle laser
        if action == 'toggle_laser':
            LASER_ON = not LASER_ON
            GPIO.output(POWER_PIN, GPIO.HIGH if LASER_ON else GPIO.LOW)
            message = f"Laser toggled to {'ON' if LASER_ON else 'OFF'}."

        # Move motors to specified angles
        elif action == 'move_angles':
            try:
                az = float(parsed.get('az_deg', m_az.angle))
                alt = float(parsed.get('alt_deg', m_alt.angle))
                message = f"Moving to Az={az:.2f}°, Alt={alt:.2f}°..."
                # Move az then alt (synchronous)
                m_az.go_angle(az)
                m_alt.go_angle(alt)
                message += " done."
            except Exception as e:
                message = f"Move error: {e}"

        # Zero motors
        elif action == 'zero_motors':
            m_az.zero()
            m_alt.zero()
            GPIO.output(POWER_PIN, GPIO.LOW)
            LASER_ON = False
            message = "Motors zeroed and laser turned off."

        # Autonomous targeting: fetch JSON, find my turret, iterate targets & turrets
        elif action == 'start_autonomous':
            json_url = parsed.get('json_url', 'http://192.168.1.254:8000/positions.json')
            team_id_str = parsed.get('team_id', None)
            if not team_id_str:
                message = "Team ID required."
            else:
                try:
                    team_key = str(int(team_id_str))  # keys in JSON are strings of int
                    # fetch JSON
                    with urllib.request.urlopen(json_url, timeout=10) as resp:
                        data = resp.read().decode('utf-8')
                        j = json.loads(data)
                    turrets = j.get('turrets', {})
                    globes = j.get('globes', [])

                    if team_key not in turrets:
                        message = f"Team {team_key} not found in JSON."
                    else:
                        my = turrets[team_key]
                        my_r = float(my['r'])
                        my_theta = float(my['theta'])

                        # Build list of targets: other turrets and globes
                        targets = []
                        # Other turrets:
                        for k, v in turrets.items():
                            if k == team_key: continue
                            t_r = float(v['r'])
                            t_theta = float(v['theta'])
                            targets.append(('turret', k, t_r, t_theta, 0.0))

                        # globes have z
                        for i, g in enumerate(globes):
                            g_r = float(g['r'])
                            g_theta = float(g['theta'])
                            g_z = float(g['z'])
                            targets.append(('globe', str(i), g_r, g_theta, g_z))

                        # Iterate targets and aim+fire
                        message = f"Autonomous: {len(targets)} targets found. Executing..."
                        # We'll perform this synchronously; if you want background threading, change here.
                        for t in targets:
                            typ, ident, tr, tt, tz = t
                            az_cmd, alt_cmd = compute_target_angles(my_r, my_theta, tr, tt, tz, turret_height=0.0)
                            # Move and fire (az first then alt)
                            m_az.go_angle(az_cmd)
                            m_alt.go_angle(alt_cmd)
                            # Fire for exactly 3 seconds
                            fire_laser(3.0)
                            # small pause between shots
                            time.sleep(0.5)
                        message += " Done."
                except Exception as e:
                    message = f"Autonomous error: {e}"

        else:
            message = "Unknown action."

        # After handling, redirect back to root with a small message (we simply regenerate page with message)
        self.send_response(303)
        self.send_header('Content-type', 'text/html')
        self.send_header('Location', '/')
        self.end_headers()
        # Note: The browser will GET / and will see the message disappear; we kept message generation server-side.
        # A better UI could use AJAX to display the message. For simplicity we redirect.

# ---------- Run server ----------
def run_server():
    server_address = ('', PORT)
    httpd = HTTPServer(server_address, PowerHandler)
    print(f"Starting Pi Control Server on port {PORT}. Open http://<pi-ip>:{PORT}/")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("Stopping server...")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up.")

if __name__ == '__main__':
    run_server()
