#ifndef _SERVER_CORE_CLIENT_CONNECTION_H_
#define _SERVER_CORE_CLIENT_CONNECTION_H_

#include <stdint.h>
#include <functional>
#include <string>
#include <vector>

class ClientConnection;

using ConnectionFactory = std::function<ClientConnection*(int)>;

class ClientConnection {
 public:
  static ConnectionFactory GetFactory();

  ClientConnection(int sock_fd);
  ClientConnection(const ClientConnection&) = delete;
  ClientConnection(ClientConnection&&) = delete;
  virtual ~ClientConnection();

  ClientConnection& operator=(const ClientConnection&) = delete;

  void SendAll(std::vector<uint8_t>&);
  void SendAll(std::string&);
  void Send(std::vector<uint8_t>&, size_t bytes, size_t offset=0);
  size_t Receive(std::vector<uint8_t>&, size_t bytes, size_t offset=0);
  size_t ReceiveAll(std::vector<uint8_t>&);

  int16_t ReceiveInt16();
  int32_t ReceiveInt32();
  int64_t ReceiveInt64();

  void SendInt16(int16_t);
  void SendInt32(int32_t);
  void SendInt64(int64_t);

 protected:
  virtual size_t Receive(uint8_t* buf, size_t bytes, size_t offset=0);
  virtual void Send(const uint8_t* buf, size_t bytes, size_t offset=0);

 private:
  int sock_fd;
};

// ifndef _SERVER_CORE_CLIENT_CONNECTION_H_
#endif
