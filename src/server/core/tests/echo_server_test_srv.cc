#include "..\netuv.h"
#include <iostream>

namespace echo {

  const int DEFAULT_PORT = 8575;
  const char * LOCAL_SOCKET = "0.0.0.0";

class ClientHandler {
  using uv::ClientSocket;
  using uv::Buffer;

 public:
  ClientHandler(ClientSocket* sock);
  virtual ~ClientHandler();

 private:
  void on_read(Buffer buf, bool success);
  void on_write(bool success);
  void shutdown();

  ClientSocket* sock;
};

}

int main() {
  using uv::EventLoop;
  using uv::ServerSocket;

  EventLoop main_loop;
  ServerSocket server(&main_loop, [](ClientSocket* sock) {
    new ClientHandler(sock);
  });
  bool s1 = server.Bind(echo::LOCAL_SOCKET, echo::DEFAULT_PORT);
  bool s2 = server.Listen();

  if (!s1 || !s2) {
    std::cerr << "Failed to initialize server.\n";
    return 1;
  }

  return main_loop.Run();
}

echo::ClientHandler(ClientSocket* sock) {
  this->sock = sock;
  this->sock->SetOnRead([=](Buffer buf, bool success) {
    this->on_read(std::move(buf), success);
  });
  this->sock->SetOnWrite([=](bool success) {
    this->on_write(success);
  });
}

void
echo::ClientHandler::on_read(Buffer buf, bool success) {
  if (success) {
    this->sock->StartWrite(std::move(buf));
  } else {
    this->shutdown();
  }
}

void
echo::ClientHandler::on_write(bool success) {
  // todo: log
}

void
echo::ClientHandler::shutdown() {
  delete this;
}

echo::ClientHandler::~ClientHandler() {
  if (this->sock) {
    delete this->sock;
  }
}
