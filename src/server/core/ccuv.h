#ifndef _SERVER_CORE_CCUV_H_
#define _SERVER_CORE_CCUV_H_

#include <uv.h>
#include <vector>

#define UNUSED(x) (void)(x)

namespace uv {

using byte = unsigned char;

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
  uv_loop_t* data();
 private:
  uv_loop_t loop;
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
  Buffer(std::vector<byte> && buf);
  Buffer(int reserve);
  Buffer() = default;
  Buffer(const Buffer&) = default;
  Buffer(Buffer&&) = default;
  Buffer& operator=(const Buffer&) = default;
  virtual ~Buffer() = default;

  void write(const byte* bytes, int nbytes);
  const char* data() const;
  char* data();
  ssize_t length() const;
 private:
  std::vector<byte> buf;
};

}

#endif // defined _SERVER_CORE_CCUV_H_
