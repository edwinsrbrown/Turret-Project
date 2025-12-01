#!/usr/bin/env python3
import http.server
import socketserver
import urllib.parse
import threading
import subprocess
import time
import multiprocessing

from RPi import GPIO
from shifter import Shifter
from stepper_class_shiftregister_multiprocessing import Stepper

PORT = 8000

# ----------------- MOTOR SETUP FOR MANUAL CONTROL -----------------

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# One shared shift register for both motors (same wiring as turretmotors.py)
s = Shifter(data=17, clock=27, latch=22)

# Lock so only one motor updates shifter at a time (like your main code)
lock = multiprocessing.Lock()

# Two stepper motors on same shift register
m1 = Stepper(s, lock)   # motor on Qe–Qh
m2 = Stepper(s, lock)   # motor on Qa–Qd

# Zero both motors for manual session
m1.zero()
m2.zero()

print("[SERVER] Manual motors initialized and zeroed.")


# ----------------- HTML PAGE -----------------

PAGE = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Turret Web Interface</title>
  <style>
    body { font-family: Arial, sans-serif; max-width: 600px; margin: 20px auto; }
    fieldset { margin-bottom: 20px; padding: 15px; }
    label { display: inline-block; width: 150px; }
    input[type=number] { width: 120px; }
    button { padding: 6px 16px; margin-top: 8px; }
  </style>
</head>
<body>
  <h1>Turret Control</h1>

  <!-- Manual movement -->
  <fieldset>
    <legend>Manual Motor Movement (Relative)</legend>
    <form method="POST" action="/move">
      <p>
        <label for="turret">Turret Δangle (deg):</label>
        <input id="turret" name="turret" type="number" step="0.1" required>
      </p>
      <p>
        <label for="globe">Globe Δangle (deg):</label>
        <input id="globe" name="globe" type="number" step="0.1" required>
      </p>
      <button type="submit">Move Motors</button>
    </form>
    <p><small>Positive values rotate one way, negative values the opposite way.</small></p>
  </fieldset>

  <!-- Run full JSON-based turret sequence -->
  <fieldset>
    <legend>Run JSON Target Sequence (turretmotors.py)</legend>
    <form method="POST" action="/auto">
      <p>
        <label for="tid">Turret ID:</label>
        <input id="tid" name="tid" type="number" min="1" required>
      </p>
      <button type="submit">Run turretmotors.py with this ID</button>
    </form>
    <p><small>
      This will start <code>turretmotors.py</code> in the background and feed the ID
      into its <code>input("Enter our turret ID: ")</code> prompt.
    </small></p>
  </fieldset>

</body>
</html>
"""


# ----------------- HTTP HANDLER -----------------

class TurretHandler(http.server.BaseHTTPRequestHandler):

    def _send_html(self, html: str):
        data = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        # Always serve the main UI
        print("[SERVER] GET", self.path)
        self._send_html(PAGE)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode("utf-8")
        params = urllib.parse.parse_qs(body)

        print(f"[SERVER] POST {self.path} params={params}")

        if self.path == "/move":
            # Manual relative movement
            try:
                turret_delta = float(params.get("turret", ["0"])[0])
                globe_delta = float(params.get("globe", ["0"])[0])
            except ValueError:
                print("[SERVER] Invalid movement values.")
                return self._send_html(PAGE)

            # Rotate motors
            try:
                print(f"[MANUAL] m1.rotate({turret_delta})")
                m1.rotate(turret_delta)
                print(f"[MANUAL] m2.rotate({globe_delta})")
                m2.rotate(globe_delta)
            except Exception as e:
                print("[MANUAL] Error rotating motors:", e)

            return self._send_html(PAGE)

        elif self.path == "/auto":
            # Run turretmotors.py and feed turret ID into its input()
            tid_str = params.get("tid", [""])[0].strip()
            if not tid_str:
                print("[AUTO] No turret ID provided.")
                return self._send_html(PAGE)

            def worker(tid: str):
                print(f"[AUTO] Starting turretmotors.py with ID {tid}")
                try:
                    proc = subprocess.Popen(
                        ["python3", "turretmotors.py"],
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True
                    )

                    # Send the ID to its input() prompt
                    proc.stdin.write(tid + "\n")
                    proc.stdin.flush()

                    # Stream its output to server console
                    for line in proc.stdout:
                        print("[turretmotors]", line, end="")

                    proc.wait()
                    print(f"[AUTO] turretmotors.py finished with code {proc.returncode}")

                except Exception as e:
                    print("[AUTO] Error running turretmotors.py:", e)

            threading.Thread(target=worker, args=(tid_str,), daemon=True).start()
            return self._send_html(PAGE)

        else:
            # Unknown path
            return self._send_html("<h1>404 Not Found</h1>")


# ----------------- MAIN SERVER START -----------------

def main():
    with socketserver.TCPServer(("", PORT), TurretHandler) as httpd:
        print(f"[SERVER] Web UI running at http://<your_pi_ip>:{PORT}")
        print("[SERVER] Use 'hostname -I' on the Pi to find <your_pi_ip>.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[SERVER] Shutting down...")
        finally:
            GPIO.cleanup()


if __name__ == "__main__":
    main()
