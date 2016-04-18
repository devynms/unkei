#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <execinfo.h>
#include <signal.h>

#include "ClientJobHandler.h"
#include "ClientFileHandler.h"
#include "netuv.h"

const int JOB_PORT = 43230;
const int FILE_PORT = 43081;

void bad_handler(int sig) {
  void *array[10];
  size_t size;

  size = backtrace(array, 10);
  fprintf(stderr, "Error: signal %d\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main() {
  using namespace server::core;

  signal(SIGSEGV, bad_handler);

  uv::EventLoop loop;

  uv::ServerSocket job_server(&loop, [&](uv::ClientSocket* sock) {
    ClientJobHandler* handler = new ClientJobHandler(&loop, sock);
    UNUSED(handler);
  });

  uv::ServerSocket file_server(&loop, [&](uv::ClientSocket* sock) {
    ClientFileHandler* handler = new ClientFileHandler(&loop, sock);
  });

  job_server.Bind("0.0.0.0", JOB_PORT);
  file_server.Bind("0.0.0.0", FILE_PORT);
  bool success;
  success = job_server.Listen();
  if (!success) {
    fprintf(stderr, "Error listening on job server.\n");
    return 1;
  }
  success = file_server.Listen();
  if (!success) {
    fprintf(stderr, "Error listening on file server.\n");
  }

}
