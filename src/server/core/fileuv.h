#ifndef _SERVER_CORE_FILEUV_H_
#define _SERVER_CORE_FILEUV_H_

#include <string>
#include <functional>
#include <vector>

#include "ccuv.h"

namespace uv {

class File {
 public:
  File(EventLoop* loop, std::string path);
  File(EventLoop* loop);
  File(const File&) = delete;
  File(File&&) = default;
  virtual ~File() = default;

  void SetPath(std::string path);

  void OnRead(std::function<void(int read)> read_cb);
  void OnOpen(std::function<void(bool)> open_cb);
  void OnClose(std::function<void(bool)> close_cb);
  void OnWrite(std::function<void(int written)> write_cb);

  void StartOpen(int flags, int mode);
  void StartRead(Buffer& buf, int offset);
  void StartWrite(const Buffer& buf, int offset);
  // void StartWrite(const std::vector<Buffer>&);
  void StartClose();

  void OpenDone(int status);
  void CloseDone(int status);
  void ReadDone(int status);
  void WriteDone(int status);
 private:
  std::function<void(int)> on_read_done;
  std::function<void(int)> on_write_done;
  std::function<void(bool)> on_open_done;
  std::function<void(bool)> on_close_done;
  std::string path;
  EventLoop* loop;
  uv_file handle;
};

} // namespace uv

#endif // defined _SERVER_CORE_FILEUV_H_
