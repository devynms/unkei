#ifndef _SERVER_CORE_NETUV_H_
#define _SERVER_CORE_NETUV_H_

#include <functional>
#include <vector>

#include "ccuv.h"

namespace uv{


class ServerSocket;
class ClientSocket;

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
/// A C++ wrapper class around a uv_tcp_t instance owning the connection to a
/// client socket.
/// Ownership: the object owning the connection to the client is responsible
/// for closing the connection and disposing of the ClientSocket instance.
/// Releasing memory: the owning object is responsible for calling StartClose(),
/// the provided OnClose callback should also free the memory of the
/// ClientSocket.
///
class ClientSocket {
 friend class ServerSocket;

 public:
  ClientSocket(EventLoop* loop);
  ClientSocket(const ClientSocket&) = delete;
  ClientSocket(ClientSocket&&) = delete;
  ClientSocket& operator=(const ClientSocket&) = delete;
  virtual ~ClientSocket() = default;

  void OnRead(std::function<void(Buffer, bool)> read_cb);
  void OnWrite(std::function<void(bool)> write_cb);
  void OnClose(std::function<void()> close_cb);

  void StartRead();
  void StartWrite(const Buffer& buf);
  void StartClose();

  void OnWriteDone(int status);
  void OnReadDone(const uv_buf_t* buf, ssize_t nread);
  void OnCloseDone();

 private:
  uv_tcp_t tcp_connection;
  std::function<void(Buffer, bool)> on_read_done;
  std::function<void(bool)> on_write_done;
  std::function<void()> on_close_done;
};

} // namespace uv

#endif
