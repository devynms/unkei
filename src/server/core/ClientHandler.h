#ifndef _SERVER_CORE_CLIENT_HANDLER_H_
#define _SERVER_CORE_CLIENT_HANDLER_H_

class ClientHandler {
 public:
  virtual void handle() = 0;
  virtual void close() = 0;
};

#endif
