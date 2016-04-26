#include "ServerListener.h"
#include "ClientConnection.h"
#include "ClientHandler.h"
#include "FilesystemClientHandler.h"
#include "JobClientHandler.h"

#include <thread>
#include <iostream>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#define EVER ;;

static const int FILESYSTEM_LISTENER_PORT = 43230;
static const int JOB_LISTENER_PORT = 43231;

void Register(ClientHandler* handler);


void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main()
{
  signal(SIGSEGV, handler);

  ServerListener filesystem_listener(ClientConnection::GetFactory());
  if (!filesystem_listener.Create()) {
    std::cerr << "Failed to create filesystem listener." << std::endl;
    exit(0);
  }
  if (!filesystem_listener.Bind(FILESYSTEM_LISTENER_PORT)) {
    std::cerr << "Failed to bind filesystem listener." << std::endl;
    exit(0);
  }
  if (!filesystem_listener.Listen()) {
    std::cerr << "Failed to listen to filesystem listener." << std::endl;
    exit(0);
  }

  ServerListener job_listener(ClientConnection::GetFactory());
  if (!job_listener.Create()) {
    std::cerr << "Failed to bind job listener." << std::endl;
    exit(0);
  }
  if (!job_listener.Bind(JOB_LISTENER_PORT)) {
    std::cerr << "Failed to bind job listener." << std::endl;
    exit(0);
  }
  if (!job_listener.Listen()) {
    std::cerr << "Failed to listen to job listener." << std::endl;
    exit(0);
  }

  std::thread filesystem_listen_thread([&]() {
    for (EVER) {
      auto connection = filesystem_listener.Accept();
      // std::cout << "accept" << std::endl;
      auto handler = new FilesystemClientHandler(connection);
      Register(handler);
    }
  });

  std::thread job_listen_thread([&]() {
    for (EVER) {
      auto connection = job_listener.Accept();
      auto handler = new JobClientHandler(connection);
      Register(handler);
    }
  });

  filesystem_listen_thread.join();
  job_listen_thread.join();
}

void
Register(ClientHandler* handler)
{
  std::thread t([=](){
    handler->handle();
    handler->close();
  });
  t.detach();
}
