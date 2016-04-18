#ifndef _SERVER_CORE_CLIENT_FILE_HANDLER_H_
#define _SERVER_CORE_CLIENT_FILE_HANDLER_H_

#include "ccuv.h"
#include "netuv.h"
#include "BufferedReader.h"

namespace server{
namespace core {

class ClientFileHandler;

class ClientFileHandler {
 public:
  ClientFileHandler(uv::EventLoop*, uv::ClientSocket*);
  virtual ~ClientFileHandler();
  ClientFileHandler(const ClientFileHandler&) = delete;
  ClientFileHandler(ClientFileHandler&&) = delete;
  ClientFileHandler& operator=(const ClientFileHandler&) = delete;

  void Start();

 protected:
  void on_socket_read(uv::Buffer buf, bool success);
  void on_socket_write(bool success);

  void on_meta_open(bool success);
  void on_meta_close(bool sucecss);
  void on_meta_read(int read);
  void on_meta_size(uint64_t size);

  void destroy();

 private:
  void StartResponse();
  void NotifyFailure();
  std::string GetRealPath() const;

  uv::ClientSocket* connection;
  uv::file          meta_file;
  uv::file          requested_file;
  uint64_t  meta_file_size;
  uint64_t  requested_file_size;
  bool  connection_closed;
  bool  meta_file_closed;
  bool  requested_file_closed;
  BufferedReader    reader;
  std::string filename;
  std::string username;
  std::string password;
  int filename_len;
  int username_len;
  int password_len;
};

}
}

#endif
/* defined _SERVER_CORE_CLIENT_FILE_HANDLER_H_ */
