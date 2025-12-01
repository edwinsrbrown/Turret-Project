from Stepper import Stepper
import multiprocessing
import json
import urllib.request
import math
import time
import socket
from RPi import GPIO
GPIO.setmode(GPIO.BCM)

laz_GPIO_pin = 18
firetime = 3

def getTeams():
  with urllib.request.urlopen("http://mml.umd.edu/enme441/teams.json") as url:
    data = json.load(url)
  for teams in data:
    if teams["Team Name"] == 'Test':
      print(teams)
      return teams

def getTargets():
  with urllib.request.urlopen("http://mml.umd.edu/enme441/targets.json") as url:
      data = json.load(url)
  target_map = {}
  for target in data:
      target_map[target["target number"]] = [target["x"], target["y"],target["z"]]
  print(target_map)
  return target_map

def web_page(dynamic_text="",team_table_html="",target_table_html="",calibration_value=""):
  html_content = f"""
  <!DOCTYPE html>
  <form action="/" method="GET">
  <html lang="en">
  <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <title>Team Coordinates</title>
      <style>
          body {{ font-family: Arial, sans-serif; margin: 20px; }}
          table {{ border-collapse: collapse; width: 100%; }}
          th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
          th {{ background-color: #f4f4f4; }}
      </style>
  </head>
  <body>
      <h3>Phase 1</h3>
      <form action="/" method="GET">
          <p><button class="button" type="submit" name="button1" style="background-color: green; color: white; padding: 10px 20px; border-radius: 5px;">Start Phase 1</button></p>
      </form>
       <p><strong>{dynamic_text}</strong></p>
       {team_table_html}
       {target_table_html}
      <p></p>
      <form action="/" method="GET">
          <label for="calibration">Calibration:</label>
          <input type="text" id="calibration" name="calibration" value="{calibration_value}" placeholder="Enter Calibration Value"><br><br>

           <button type="submit" style="background-color: blue; color: white; padding: 10px 20px; border-radius: 5px;">Save Calibration</button>
       </form>
      <p></p>
      <h3>Phase 2</h3>

      <form action="/" method="GET">
  <label for="input1">Input 1:</label>
  <input type="text" id="input1" name="input1" placeholder="Enter text here"><br><br>
  <label for="input2">Input 2:</label>
  <input type="text" id="input2" name="input2" placeholder="Enter text here"><br><br>
  <label for="input3">Input 3:</label>
  <input type="text" id="input3" name="input3" placeholder="Enter text here"><br><br>
  <label for="input4">Input 4:</label>
  <input type="text" id="input4" name="input4" placeholder="Enter text here"><br><br>
  <button type="submit" name="button2" style="background-color: green; color: white; padding: 10px 20px; border-radius: 5px;">Start Phase 2</button>
</form>

  <p><strong>Current Target ID</strong></p>
      <p>Display Text</p>

      <button style="background-color: red; color: white; padding: 10px 20px; border-radius: 5px;">STOP</button>

  """
  return bytes(html_content, 'utf-8')

# function to get team coordinates from JSON
def getTeamsTable():
    with urllib.request.urlopen("http://mml.umd.edu/enme441/teams.json") as url:
      data = json.load(url)
    team_map = {}
    for teams in data:
      team_map[teams["Team Name"]] = [teams["x"], teams["y"]]
      if teams["Team Name"] == 'Beamin Baddiez':
        print(teams["Team Name"], teams["x"], teams["y"])

    print(team_map) #prints array with all teams and coordinates

    table_html = """
    <h4>Team Coordinates</h4>
    <table>
        <tr>
            <th>Team Name</th>
            <th>X Coordinate</th>
            <th>Y Coordinate</th>
        </tr>
    """
    # loops through the teams and add rows to the table
    for team in data:
        table_html += f"""
        <tr>
            <td>{team["Team Name"]}</td>
            <td>{team["x"]}</td>
            <td>{team["y"]}</td>
        </tr>
        """
    table_html += "</table>"
    return table_html

# function to get target coordinates from JSON
def getTargetsTable():
    with urllib.request.urlopen("http://mml.umd.edu/enme441/targets.json") as url:
        data = json.load(url)
    target_map = {}

    table_html = """
    <h4>Target Coordinates</h4>
    <table>
        <tr>
            <th>Target Number</th>
            <th>X Coordinate</th>
            <th>Y Coordinate</th>
            <th>Z Coordinate</th>
        </tr>
    """
    # loops through the targets and add rows to the table
    for target in data:
        target_map[target["target number"]] = [target["x"], target["y"],target["z"]]
        table_html += f"""
        <tr>
            <td>{target["target number"]}</td>
            <td>{target["x"]}</td>
            <td>{target["y"]}</td>
            <td>{target["z"]}</td>
        </tr>
        """

    table_html += "</table>"
    return table_html

def serve_webpage():
    try:
        while True:
            print("Waiting for connection...")
            conn, addr = s.accept()
            print(f'Connection from {addr}')
            data = conn.recv(1024).decode('utf-8')  # specify buffer size (max data to be received)
            print('\nGET request received:\n--------------------')
            print(data)
            data = data[data.find('GET')+6 : data.find('HTTP')]  # slice the GET data
            dynamic_text = ""  # Default value for dynamic text
            target_table_html = ""
            team_table_html = ""
            calibration_value = ""
            phase2_values = []

            if len(data) > 0:
                if "button1" in data:
                    print("Button 1 Pressed")
                    dynamic_text = "Phase 1 Activated..."  # Set dynamic content if button is pressed
                    phase1_operation() #Activates phase 1
                    target_table_html = getTargetsTable()
                    team_table_html = getTeamsTable()
                if "calibration" in data:
                    # Extract the calibration value from the URL (after the '=' sign)
                    calibration_value = data.split("calibration=")[1].split(" ")[0]
                    print(f"Calibration value saved: {calibration_value}")
                    calibrate(calibration_value) #Starts the calibration
                if "button2" in data:
                    # Extract values for Phase 2 inputs
                    print("Button 2 pressed")
                    inputs = ["input1", "input2", "input3", "input4"]
                    for inp in inputs:
                        if f"{inp}=" in data:
                            value = data.split(f"{inp}=")[1].split("&")[0]
                            phase2_values.append(value)
                    print(f"Phase 2 values saved: {phase2_values}")
                    phase2_operation(phase2_values) #Activates phase 2

            conn.send(b'HTTP/1.0 200 OK\n')         # status line
            conn.send(b'Content-type: text/html\n') # header (content type)
            conn.send(b'Connection: close\r\n\r\n') # header (tell client to close at end)
            conn.sendall(web_page(dynamic_text,team_table_html,target_table_html,calibration_value))   # Pass dynamic_text to web_page function
            conn.close()
    except Exception as e:
        print(e)
    conn.close()

# calculates angle from turret to target (theta x,y)
def calcAngle(turPos,targPos):
  x1 = float(turPos["x"])
  y1 = float(turPos["y"])
  x2 = float(targPos["x"])
  y2 = float(targPos["y"])

  angle = math.degrees(math.atan2(y2-y1,x2-x1))
  print(angle)
  return angle

# calculates angle from turret to target (theta z)
def aim(turPos,targPos):
  x1 = float(turPos["x"])
  z1 = 6.42 #some set value based on offset, make zero for now
  x2 = float(targPos["x"])
  z2 = float(targPos["z"])
  attackang = math.degrees(math.atan2(abs(z2-z1),abs(x2-x1)))
  print(attackang)
  return attackang


def fire():
  GPIO.output(laz_GPIO_pin,GPIO.HIGH)
  time.sleep(firetime)
  GPIO.output(laz_GPIO_pin,GPIO.LOW)

# PHASE 1 OPERATION
def phase1_operation():
  try:
    basestep.zero()
    lazstep.zero()
    tur_pos = getTeams()
    with urllib.request.urlopen("http://mml.umd.edu/enme441/targets.json") as url:
      data = json.load(url)
    for target in data:
      print("Firing at Target"+target["target number"])
      x_theta = calcAngle(tur_pos,target)
      z_theta = aim(tur_pos,target)
      basestep.goAngle(x_theta)
      time.sleep(2)
      lazstep.goAngle(z_theta)
      fire()
      time.sleep(2)

    print("Phase 1 Complete!")
    return True

  except Exception as e:
    print(f"Error during Phase 1: {e}")
    time.sleep(0.1) #prevents any busy waiting

#PHASE 2 OPERATION

def phase2_operation(targets):
    try:
        basestep.goAngle(0)
        lazstep.goAngle(0)
        tur_pos = getTeams()
        phase2targets = targets

        for target in phase2targets:
            print(f"Firing at target: {target['target number']}")
            xtheta = calcAngle(tur_pos, target)
            ztheta = aim(tur_pos,target)
            basestep.goAngle(xtheta)
            time.sleep(2)
            lazstep.goAngle(ztheta)
            time.sleep(2)
            fire()

        print("Phase 2 Complete!")
        return True
    except Exception as e:
        print(f"Error during Phase 2: {e}")
        time.sleep(0.1)

#CALIBRATION

def calibrate(calibrateval):
    value = calibrateval
    print(f"Calibrating Base: Rotating {value} degrees")
    basestep.goAngle(value)
    print("Calibration finished")

    #Needs second value for moving laser stepper manually, and a button to turn the laser on and off manually







if __name__ == '__main__':

    # execute multiple operations at the same time:
    lock1 = multiprocessing.Lock()
    lock2 = multiprocessing.Lock()
    # Instantiate 2 Steppers:
    basestep = Stepper([6,13,19,26], lock1)
    lazstep = Stepper([12,16,20,21], lock2)

    #Instantiate Laser Pin:
    GPIO.setup(laz_GPIO_pin,GPIO.OUT)

    # # Zero the motors:
    # basestep.zero()
    # lazstep.zero()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('', 8080))
    s.listen(3)  # up to 3 queued connections

try:
    serve_webpage()
except KeyboardInterrupt:

    print("Ending Program")
except Exception as e:
    print(f"An error has occurred: {e} ")
finally:
    print("Shutting down turret:")
    GPIO.output(laz_GPIO_pin, GPIO.LOW)
    basestep.goAngle(0)
    lazstep.goAngle(0)
    GPIO.cleanup()
