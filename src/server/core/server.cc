#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>
#include "netuv.h"

const int DEFAULT_PORT = 8575;

class EchoClientHandler {
 public:
  EchoClientHandler(uv::ClientSocket* sock)
    : sock(sock) {
      sock->OnRead([=](uv::Buffer buf, bool success) {
        this->on_read(std::move(buf), success);
      });

      sock->OnWrite([=](bool success) {
        this->on_write(success);
      });

      sock->OnClose([=]() {
        delete this;
      });

      sock->StartRead();
    }

  virtual ~EchoClientHandler() {
    if (this->sock) {
      delete this->sock;
    }
  }
 private:

  void on_read(uv::Buffer buf, bool success) {
    if (!success) {
      this->sock->StartClose();
      return;
    }

    this->sock->StartWrite(std::move(buf));
  }

  void on_write(bool success) {
    if (!success) {
      // log error
    }
  }

  uv::ClientSocket* sock;
};

void bad_handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main() {
  signal(SIGSEGV, bad_handler);

  uv::EventLoop loop;
  uv::ServerSocket server(&loop, [](uv::ClientSocket* sock) {
    EchoClientHandler* handler = new EchoClientHandler(sock);
    (void)(handler);
  });

  server.Bind("0.0.0.0", DEFAULT_PORT);
  bool success = server.Listen();

  if (!success) {
    fprintf(stderr, "Listen error.\n");
    return 1;
  }

  return loop.Run();
}

