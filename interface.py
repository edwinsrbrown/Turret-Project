from http.server import HTTPServer, BaseHTTPRequestHandler
import RPi.GPIO as GPIO

#Parse Function
def parsePOSTdata(data):
  data_dict = {}
  idx = data.find('\r\n\r\n')+4
  data = data[idx:]
  data_pairs = data.split('&')
  
  for pair in data_pairs:
    key_val = pair.split('=')
    if len(key_val) == 2:
      data_dict[key_val[0]] = key_val[1]

  return data_dict

PORT = 8080
POWER_PIN = 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(POWER_PIN, GPIO.OUT)
GPIO.output(POWER_PIN, GPIO.LOW)

POWER = GPIO.input(POWER_PIN)

class PowerHandler(BaseHTTPRequestHandler):
  
  def _generate_html(self):
    global POWER

    if POWER:
      status_text = "ON"
      button_label = "TURN OFF"

    else:
      status_text = "OFF"
      button_label = "TURN ON"

    html_content = f"""

<!DOCTYPE html>
<html>
<head>
  <title>Turret Project</title>
</head>
<body>
  <h1>Device Status: <b>{status_text}</b></h1> 
  <hr>

  <form action="/" method="POST">
    <button name="toggle_action" value="1">{button_label}</button>
  </form>
</body>
</html>
"""
    return html_content

  def do_GET(self):
    self.send_response(200)
    self.send_header("Content-type", "text/html")
    self.end_headers()
    self.wfile.write(self._generate_html().encode("utf-8"))

  def do_POST(self):
    global POWER
    
    content_length = int(self.headers.get('Content-Length', 0))
    post_data_raw = self.rfile.read(content_length).decode('utf-8')
    parsed_data = parsePOSTdata(post_data_raw)

    if 'toggle_action' in parsed_data:
      POWER = not POWER

      if POWER:
        GPIO.output(POWER_PIN, GPIO.HIGH)
      else:
        GPIO.output(POWER_PIN, GPIO.LOW)

    self.send_response(303)
    self.send_header('Location', '/')
    self.end_headers()

def run_server():
    server_address = ('', PORT)
    httpd = HTTPServer(server_address, PowerHandler)
    
    try:
        print(f"Starting Pi Control Server on http://172.20.10.2:{PORT}")
        print("Press Ctrl+C to stop the server.")
        httpd.serve_forever()
        
    except KeyboardInterrupt:
        print("\nServer stopping...")
    
    finally:
        # CRUCIAL: Clean up GPIO settings when the script exits
        GPIO.cleanup() 
        print("GPIO cleaned up and pins reset.")

if __name__ == '__main__':
    run_server()
