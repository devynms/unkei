#include <string>
#include <sstream>
#include <ios>
#include <cstring>

#include "ClientFileHandler.h"
#include "SpookyHash.h"

server::core::ClientFileHandler(
  uv::EventLoop* parent_loop,
  uv::ClientSocket* socket)
    : connection(socket),
      meta_file(parent_loop),
      requested_file(parent_loop)
{
  meta_file_size = 0;
  requested_file_size = 0;
  connection_closed = false;
  meta_file_closed = false;
  requested_file_closed = false;
  filename_len = -1;
  username_len = -1;
  password_len = -1;

  this->connection->OnRead([=](uv::Buffer buf, bool success) {
    this->on_socket_read(std::move(buf), success);
  });

  this->connection->OnWrite([=](bool success) {
    this->on_socket_write(sucecss);
  });

  this->connection->OnClose([=]() {
    // TODO
  });

  this->requested_file.OnRead([=](int read) {

  });

  this->requested_file.OnOpen([=](bool success) {

  });

  this->requested_file.OnClose([=](bool success) {

  });

  this->requested_file.OnWrite([=](int written) {

  });

  this->meta_file.OnRead([=](int read) {
    this->on_meta_read(read);
  });

  this->meta_file.OnOpen([=](bool success) {
    this->on_meta_open(success);
  });

  this->meta_file.OnClose([=](bool success) {
    this->on_meta_close(success);
  });

  this->meta_file.OnWrite([=](int written) {
    // LOGIC ERROR
  });
}

void
server::core::ClientFileHandler::Start()
{
  this->connection->StartRead();
}

void
server::core::ClientFileHandler::on_socket_read(uv::Buffer buf, bool success)
{
  this->reader.Write(buf.data(), buf.length());

  if (this->username_len == -1) {
    if (!this->reader.ReadInt(&this->username_len)) return;
  }

  if (this->password_len == -1) {
    if (!this->reader.ReadInt(&this->password_len)) return;
  }

  if (this->filename_len == -1) {
    if (!this->reader.ReadInt(&this->filename_len)) return;
  }

  if (username.empty()) {
    if (this->reader.AvailableReadSpace() < this->username_len) return;
    char* buf = new char[this->username_len];
    this->reader.ReadSomeBytes(buf, this->username_len);
    this->username = std::string(buf, this->username_len);
    delete buf;
  }

  if (password.empty()) {
    if (this->reader.AvailableReadSpace() < this->password_len) return;
    char* buf = new char[this->password_len];
    this->reader.ReadSomeBytes(buf, this->password_len);
    this->password = std::string(buf, this->password_len);
    delete buf;
  }

  if (filename.empty()) {
    if (this->reader.AvailableReadSpace() < this->filename_len) return;
    char* buf = new char[this->filename_len];
    this->reader.ReadSomeBytes(buf, this->filename_len);
    this->filename = std::string(buf, this->filename_len);
    delete buf;
  }

  this->StartResponse();
}

std::string
server::core::ClientFileHandler::GetRealPath() const
{
  using std::string;
  using std::stringstream;
  using std::memcpy;

  string path;
  path.reserve(this->filename_len + this->username_len + 1);
  path = this->username + "/" + this->filename;
  uint64 hash1, hash2;
  SpookyHash.Hash128(path.data(), path.size(), &hash1, &hash2);
  std::stringstream real_path;
  unit16 buf[4];
  memcpy(buf, &hash1, 8);
  real_path << std::hex << buf[0] << "/" << buf[1] << buf[2] << buf[3] << hash2;
  return real_path.str();
}

void
server::core::ClientFileHandler::StartResponse()
{
  using std::string;

  string path = this->GetRealPath();
  string meta_path = path + ".meta";
  meta_file.SetPath(meta_path);
  meta_file.StartOpen(O_RDONLY, S_IRUSR);
}

void
server::core::ClientFileHandler::on_meta_open(bool success)
{
  if (success == false) {
    this->NotifyFailure();
  }
  this->meta_file.GetSize([=](uint64_t size) {
    this->meta_file_size = size;
  });
}

void
server::core::ClientFileHandler::on_meta_close(bool success)
{

}

void
server::core::ClientFileHandler::on_meta_read(int read)
{

}
