import socket

SERVER_PORT = 8575

class ServerConnection:
  def __init__(self, ipaddr):
    self.ipaddr = ipaddr
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  def connect():
    self.sock.connect((self.ipaddr, SERVER_PORT))

  def send_msg_get_response(msg):
    self.sock.send(msg)
    response = self.sock.recv(len(msg))
    return response
