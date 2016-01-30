#ifndef _SERVER_UV_H_
#define _SERVER_UV_H_

#include <functional>
#include <vector>
#include <uv.h>

namespace uv{

using byte = unsigned char;

class EventLoop;
class ServerSocket;
class ClientSocket;

///
/// A C++ wrapper class around a uv_loop_t instance representing the main event
/// loop for the server.
///
class EventLoop {
 friend class ServerSocket;
 friend class ClientSocket;

 public:
  EventLoop();
  EventLoop(const EventLoop& other) = delete;
  EventLoop(EventLoop&& other) = delete;
  EventLoop& operator =(const EventLoop&) = delete;
  virtual ~EventLoop();

  int Run();
 private:
  uv_loop_t loop;
};

///
/// A C++ wrapper class around a uv_tcp_t instance owning the listening socket
/// for the server.
/// Ownership: Lifetime is connected with the whole loop. Disposed of when the
/// program terminates.
///
class ServerSocket {
 public:
  ServerSocket(EventLoop* loop, std::function<void(ClientSocket*)> cb);
  ServerSocket(const ServerSocket&) = delete;
  ServerSocket(ServerSocket&&) = delete;
  ServerSocket& operator=(const ServerSocket&) = delete;
  virtual ~ServerSocket() = default;

  bool Bind(const char* ip, int port);
  bool Listen();

  void OnNewConnection(int status);

 private:
  uv_tcp_t tcp_connection;
  EventLoop* loop;
  std::function<void(ClientSocket*)> on_new_connection_cb;
};

///
/// Object representing a uv_buf_t instance created for reading from and writing
/// to client sockets.
/// Ownership: the object owns it's internal buffer, which is disposed on when
/// it exits scope. Can be moved or can be shared as a const reference.
///
class Buffer {
 public:
  Buffer(char * buf, ssize_t len);
  Buffer(std::vector<byte>&& buf);
  Buffer() = default;
  Buffer(const Buffer&) = default;
  Buffer(Buffer&&) = default;
  Buffer& operator=(const Buffer&) = default;
  virtual ~Buffer() = default;

  const char* data();
  ssize_t length() const;
 private:
  std::vector<byte> buf;
};

///
/// A C++ wrapper class around a uv_tcp_t instance owning the connection to a
/// client socket.
/// Ownership: the object owning the connection to the client is responsible
/// for closing the connection and disposing of the ClientSocket instance.
///
class ClientSocket {
 friend class ServerSocket;

 public:
  ClientSocket(EventLoop* loop);
  ClientSocket(const ClientSocket&) = delete;
  ClientSocket(ClientSocket&&) = delete;
  ClientSocket& operator=(const ClientSocket&) = delete;
  virtual ~ClientSocket();

  void SetOnRead(std::function<void(Buffer, bool)> read_cb);
  void SetOnWrite(std::function<void(bool)> write_cb);
  void StartRead();
  void StartWrite(Buffer buf);
  void OnWriteDone(int status);
  void OnReadDone(const uv_buf_t* buf, ssize_t nread);

 private:
  uv_tcp_t tcp_connection;
  std::function<void(Buffer, bool)> on_read_done;
  std::function<void(bool)> on_write_done;
};

} // namespace uv

#endif
