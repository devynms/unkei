#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>
#include "ClientHandler.h"
#include "netuv.h"

const int DEFAULT_PORT = 8575;

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
  using namespace server::core;

  signal(SIGSEGV, bad_handler);

  int id = 1;

  uv::EventLoop loop;
  uv::ServerSocket server(&loop, [&](uv::ClientSocket* sock) {
    ClientHandler* handler = new ClientHandler(&loop, sock, id);
    UNUSED(handler);
    id += 1;
  });

  server.Bind("0.0.0.0", DEFAULT_PORT);
  bool success = server.Listen();

  if (!success) {
    fprintf(stderr, "Listen error.\n");
    return 1;
  }

  return loop.Run();
}

