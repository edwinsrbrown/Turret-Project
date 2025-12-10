# Code to serve turret and globe JSON position data from
# a Pi with a fixed IP address. To set fixed IP, log in to
# class server and change settings as needed.

from http.server import HTTPServer, BaseHTTPRequestHandler
import json

with open("positions.json") as f:
    message = json.load(f)

class JSONHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/positions.json":
            response = json.dumps(message).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not found")

def run_server():
    HOST = "192.168.1.254"    # Replace with server IP address
    PORT = 8000
    server = HTTPServer((HOST, PORT), JSONHandler)
    server.serve_forever()

if __name__ == "__main__":
    run_server()
