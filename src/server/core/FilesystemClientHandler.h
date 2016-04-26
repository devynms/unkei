#ifndef _SERVER_CORE_FILESYSTEM_CLIENT_HANDLER_H_
#define _SERVER_CORE_FILESYSTEM_CLIENT_HANDLER_H_

#include "ClientHandler.h"
#include "ClientConnection.h"

class FilesystemClientHandler final : public ClientHandler {
 public:
  FilesystemClientHandler(ClientConnection* conn);
  virtual ~FilesystemClientHandler();

  FilesystemClientHandler(const FilesystemClientHandler&) = delete;
  FilesystemClientHandler(FilesystemClientHandler&&) = delete;

  FilesystemClientHandler&
  operator=(const FilesystemClientHandler& other) = delete;

  virtual void handle();
  virtual void close();

 private:
  void
  ResourceRequestResponse(
    const std::vector<uint8_t>& uname_data,
    const std::vector<uint8_t>& cmd_data);

  void
  UserInfoRequestResponse(
    const std::vector<uint8_t>& uname_data,
    const std::vector<uint8_t>& cmd_data);

  void
  DeleteResourceRequestResponse(
    const std::vector<uint8_t>& uname_data,
    const std::vector<uint8_t>& cmd_data);

  ClientConnection* connection;
};

#endif
