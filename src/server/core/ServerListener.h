#ifndef _SERVER_CORE_SERVER_LISTENER_H_
#define _SERVER_CORE_SERVER_LISTENER_H_

#include <stdint.h>
#include <functional>

#include "ClientConnection.h"

class ServerListener {
 public:
  ServerListener(std::function<ClientConnection*(int)> connection_factory);
  ServerListener(const ServerListener&) = delete;
  ServerListener(const ServerListener&&) = delete;
  virtual ~ServerListener();

  ServerListener& operator=(const ServerListener&) = delete;

  bool Create();
  bool Bind(uint16_t portno);
  bool Listen();
  ClientConnection* Accept();

 private:
  std::function<ClientConnection*(int)> connection_factory;
  int sock_fd;
};

// ifndef _SERVER_CORE_SERVER_LISTENER_H_
#endif
