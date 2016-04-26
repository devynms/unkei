#include "ServerListener.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <functional>
#include <unistd.h>

static const int MAXPENDING = 10;

ServerListener::ServerListener(
  std::function<ClientConnection*(int)> connection_factory)
    : connection_factory(connection_factory)
{

}

ServerListener::~ServerListener()
{
  close(this->sock_fd);
}

bool
ServerListener::Create()
{
  this->sock_fd = socket(AF_INET, SOCK_STREAM, 0);
  return (this->sock_fd >= 0);
}

bool
ServerListener::Bind(uint16_t portno)
{
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(portno);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(this->sock_fd, (struct sockaddr *)&server_addr,
           sizeof(server_addr)) < 0) {
    return false;
  }

  return true;
}

bool
ServerListener::Listen()
{
  return (listen(this->sock_fd, MAXPENDING) >= 0);
}

ClientConnection*
ServerListener::Accept()
{
  int client_sock = accept(this->sock_fd, NULL, NULL);
  if (client_sock < 0) {
    return nullptr;
  } else {
    return this->connection_factory(client_sock);
  }
}
