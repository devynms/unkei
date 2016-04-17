#ifndef _SERVER_CORE_CLIENT_HANDLER_H_
#define _SERVER_CORE_CLIENT_HANDLER_H_

#include <vector>

#include "BufferedReader.h"
#include "ccuv.h"
#include "netuv.h"
#include "fileuv.h"

namespace server {
namespace core {

class ClientHandler;

namespace impl {

enum class ClientHandlerState {
  INITIAL,
  WAITING,
  READING,
  WRITING,
  DONE,
  ERROR,
  INVALID
};

// TODO: move logic not dependent on callbacks to state machine class
class ClientHandlerStateMachine {
 public:
  ClientHandlerStateMachine(ClientHandler* parent);
  virtual ~ClientHandlerStateMachine() = default;

  ClientHandlerStateMachine(const ClientHandlerStateMachine&) = delete;
  ClientHandlerStateMachine(ClientHandlerStateMachine&&) = delete;
  ClientHandlerStateMachine& operator=(ClientHandlerStateMachine&) = delete;

  void AcceptBytes(const uv::Buffer& buf);
  void ConfirmWrite(int bytes);
  ClientHandlerState CurrentState() const;
  bool Done() const;
  void FileOpened();
 private:
  void UpdateStateMachine();

  ClientHandler* parent;
  BufferedReader reader;
  bool error_occured;
  bool file_opened;
  int expected_file_size;
  int bytes_read;
  int bytes_written;
  int bytes_confirmed;
};

}

///
/// ClientHandler
/// Closes when:
///   - Bytes are confirmed read and the state machine says it is done
///   - a socket write fails
///   - a socket read fails
///   - the state machine enters an error or invalid state
/// Should report error when:
///
class ClientHandler {
 public:
  ClientHandler(uv::EventLoop* loop, uv::ClientSocket* sock, int id);
  virtual ~ClientHandler();

  ClientHandler(const ClientHandler&) = delete;
  ClientHandler(ClientHandler&&) = delete;
  ClientHandler& operator=(const ClientHandler&) = delete;

  void Close();
  void PostBytes(const uv::Buffer& buf);

 protected:
  void on_socket_read(uv::Buffer buf, bool success);
  void on_socket_write(bool success);
  void on_file_write(int bytes);
  void destroy();

 private:
  uv::ClientSocket* connection;
  bool connection_closed;
  uv::File output_file;
  bool file_closed;
  impl::ClientHandlerStateMachine sm;
};

}
}

#endif // ifndef _SERVER_CORE_CLIENT_HANDLER_H_
