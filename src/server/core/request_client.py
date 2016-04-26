from socket import *
import struct
import json

if __name__ == '__main__':
  conn = socket(AF_INET, SOCK_STREAM)
  conn.connect(('127.0.0.1', 43230))

  uname = 'user'
  pwd = 'pass'
  uname_len = len(uname)
  pwd_len = len(pwd)
  info_cmd = 1
  info_cmd_len = 8
  info_hdr_fmt = '!hhhh'
  info_hdr = struct.pack(info_hdr_fmt, uname_len, pwd_len, info_cmd, info_cmd_len)
  cmd_data_fmt = '!q'
  cmd_data = struct.pack(cmd_data_fmt, 0)
  conn.send(info_hdr)
  conn.send(uname)
  conn.send(pwd)
  conn.send(cmd_data)
  rpl_hdr_fmt = '!ii'
  dat = conn.recv(struct.calcsize(rpl_hdr_fmt))
  (code, dl) = struct.unpack(rpl_hdr_fmt, dat)
  if code == 0:
    dat = conn.recv(dl)
    print 'user info:', dat
  conn.close()

  info = json.loads(dat)
  resources = info['data']
  info_cmd = 2
  for resource in resources:
    conn = socket(AF_INET, SOCK_STREAM)
    conn.connect(('127.0.0.1', 43230))
    info_cmd_len = len(resource)
    info_hdr = struct.pack(info_hdr_fmt, uname_len, pwd_len, info_cmd, info_cmd_len)
    conn.send(info_hdr)
    conn.send(uname)
    conn.send(pwd)
    conn.send(resource)
    rpl_hdr_fmt = '!iii'
    dat = conn.recv(struct.calcsize(rpl_hdr_fmt))
    (code, ml, dl) = struct.unpack(rpl_hdr_fmt, dat)
    meta = conn.recv(ml)
    data = conn.recv(dl)
    print 'meta file:', meta
    print 'file data:', data
