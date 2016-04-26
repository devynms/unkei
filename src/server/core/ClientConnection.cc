#include "ClientConnection.h"
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

#if defined(__linux__)
#  include <endian.h>
#elif defined(__FreeBSD__) || defined(__NetBSD__)
#  include <sys/endian.h>
#elif defined(__OpenBSD__)
#  include <sys/types.h>
#  define be16toh(x) betoh16(x)
#  define be32toh(x) betoh32(x)
#  define be64toh(x) betoh64(x)
#endif

ConnectionFactory
ClientConnection::GetFactory()
{
  return [](int sock_fd){ return new ClientConnection(sock_fd); };
}

ClientConnection::ClientConnection(int sock_fd)
  : sock_fd(sock_fd)
{

}

ClientConnection::~ClientConnection()
{
  if (sock_fd >= 0) {
    close(sock_fd);
  }
}

void
ClientConnection::Send(std::vector<uint8_t>& buf, size_t bytes, size_t offset)
{
  if (bytes > buf.size() - offset) {
    // TODO: handle error
    bytes = buf.size() - offset;
  }

  this->Send(buf.data(), bytes, offset);
}

size_t
ClientConnection::Receive(std::vector<uint8_t>& buf, size_t bytes, size_t offset)
{
  buf.resize(bytes + offset);
  return this->Receive(buf.data(), bytes, offset);
}

void
ClientConnection::SendAll(std::vector<uint8_t>& buf)
{
  this->Send(buf, buf.size());
}

size_t
ClientConnection::ReceiveAll(std::vector<uint8_t>& buf)
{
  return this->Receive(buf, buf.size());
}

int16_t
ClientConnection::ReceiveInt16()
{
  int16_t data;
  uint8_t* buf = (uint8_t*)(&data);
  size_t bytes = sizeof(data);
  this->Receive(buf, bytes);
  return ntohs(data);
}

int32_t
ClientConnection::ReceiveInt32()
{
  int32_t data;
  uint8_t* buf = (uint8_t*)(&data);
  size_t bytes = sizeof(data);
  this->Receive(buf, bytes);
  return ntohl(data);
}

int64_t
ClientConnection::ReceiveInt64()
{
  int16_t data;
  uint8_t* buf = (uint8_t*)(&data);
  size_t bytes = sizeof(data);
  this->Receive(buf, bytes);
  return be64toh(data);
}

void
ClientConnection::Send(const uint8_t* buf, size_t bytes, size_t offset)
{
  if (write(this->sock_fd, buf + offset, bytes) < 0) {
    // TODO: handle error
  }
}

size_t
ClientConnection::Receive(uint8_t* buf, size_t bytes, size_t offset)
{
  int res = read(this->sock_fd, buf + offset, bytes);
  if (res < 0) {
    // TODO: handle error
  }
  return res;
}

void
ClientConnection::SendAll(std::string& str)
{
  std::vector<uint8_t> vec(std::begin(str), std::end(str));
  this->SendAll(vec);
}

void
ClientConnection::SendInt32(int32_t data)
{
  int32_t ndata = htonl(data);
  this->Send((const uint8_t*)&ndata, sizeof(ndata));
}

/*
  TODO:

  void SendInt16(int16_t);
  void SendInt32(int32_t);
  void SendInt64(int64_t);
  void SendAll(std::string&);
*/
