#ifndef _SERVER_CORE_JOB_CLIENT_HANDLER_H_
#define _SERVER_CORE_JOB_CLIENT_HANDLER_H_

#include "ClientHandler.h"
#include "ClientConnection.h"

class JobClientHandler : public ClientHandler
{
 public:
  JobClientHandler(ClientConnection* conn);
  virtual ~JobClientHandler();

  JobClientHandler(const JobClientHandler&) = delete;
  JobClientHandler(JobClientHandler&&) = delete;

  JobClientHandler&
  operator=(const JobClientHandler& other) = delete;

  virtual void handle();
  virtual void close();

 private:
  ClientConnection* connection;
};

#endif
