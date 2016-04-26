#include "FilesystemClientHandler.h"
#include "CloudFilesystemView.h"
#include "json.hpp"

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

FilesystemClientHandler::FilesystemClientHandler(ClientConnection* connection)
    : connection(connection)
{}

FilesystemClientHandler::~FilesystemClientHandler()
{
  delete this->connection;
}

void
FilesystemClientHandler::handle()
{
  using std::vector;
  using std::cout;
  using std::endl;

  int16_t uname_len = this->connection->ReceiveInt16();
  int16_t pwd_len = this->connection->ReceiveInt16();
  int16_t cmd_id = this->connection->ReceiveInt16();
  int16_t cmd_len = this->connection->ReceiveInt16();
  vector<uint8_t> uname_data;
  vector<uint8_t> pwd_data;
  vector<uint8_t> cmd_data;
  this->connection->Receive(uname_data, uname_len);
  this->connection->Receive(pwd_data, pwd_len);
  this->connection->Receive(cmd_data, cmd_len);

  CloudFilesystemView view;
  std::string uname(std::begin(uname_data), std::end(uname_data));
  if(!view.UserExists(uname)) {
    return;
  }

  if (cmd_id == 1 && cmd_len == 8) {
    // cmd_data is 64 bit timestamp
    this->UserInfoRequestResponse(uname_data, cmd_data);
  } else if (cmd_id == 2) {
    // cmd_data is resource name, cmd_len bytes
    this->ResourceRequestResponse(uname_data, cmd_data);
  } else if (cmd_id == 3) {
    // cmd_data is resource name, cmd_len bytes
    this->DeleteResourceRequestResponse(uname_data, cmd_data);
  }
}

void
FilesystemClientHandler::ResourceRequestResponse(
  const std::vector<uint8_t>& uname_data,
  const std::vector<uint8_t>& cmd_data)
{
  using namespace std;
  using json = nlohmann::json;
  int i = 0;

  CloudFilesystemView view;
  auto uname = string(begin(uname_data), end(uname_data));
  auto resource = string(begin(cmd_data), end(cmd_data));
  if (!view.ResourceExists(uname, resource)) {
    this->connection->SendInt32(2); // code: no such resource
    this->connection->SendInt32(0);
    this->connection->SendInt32(0);
    return;
  }

  fstream resourceMetaFile(view.LookupResourceMetaFile(uname, resource), fstream::in);
  json metaJ;
  resourceMetaFile >> metaJ;
  resourceMetaFile.close();

  if (!metaJ["available"].is_null() && !metaJ["available"].get<bool>()) {
    string meta_data = metaJ.dump();
    this->connection->SendInt32(3); // code: resource incomplete
    this->connection->SendInt32(meta_data.length()); // length of meta file
    this->connection->SendInt32(0); // length of resource file
    this->connection->SendAll(meta_data);
  } else {
    string meta_data = metaJ.dump();
    fstream resource_file(view.LookupResourceFile(uname, resource), fstream::in);
    std::vector<uint8_t> resource;
    int byte;
    while ((byte = resource_file.get()) != EOF) {
      resource.push_back(byte);
    }
    resource_file.close();
    this->connection->SendInt32(0); // code: OK
    this->connection->SendInt32(meta_data.length());
    this->connection->SendInt32(resource.size());
    this->connection->SendAll(meta_data);
    this->connection->SendAll(resource);
  }
}

static int64_t
Vector8ToInt64(const std::vector<uint8_t> vec)
{
  uint64_t result = 0;
  for (int i = 0; i < 8; i++) {
    result |= (((uint64_t)0xffU << (i*8)) & (((uint64_t)vec[i])<< (i*8)));
  }
  return result;
}

void
FilesystemClientHandler::UserInfoRequestResponse(
  const std::vector<uint8_t>& uname_data,
  const std::vector<uint8_t>& cmd_data)
{
  using namespace std;
  using json = nlohmann::json;

  CloudFilesystemView view;
  auto uname = string(begin(uname_data), end(uname_data));
  fstream info_file(view.LookupUserInfoFile(uname), fstream::in);
  json userJ;
  info_file >> userJ;
  info_file.close();
  int64_t server_timestamp = userJ["timestamp"].get<int64_t>();
  int64_t client_timestamp = Vector8ToInt64(cmd_data);
  if (server_timestamp <= client_timestamp) {
    this->connection->SendInt32(0); // code: OK
    this->connection->SendInt32(0); // no data necessary
  } else {
    string contents = userJ.dump();
    this->connection->SendInt32(0);
    this->connection->SendInt32(contents.length());
    this->connection->SendAll(contents);
  }
}

void
FilesystemClientHandler::DeleteResourceRequestResponse(
  const std::vector<uint8_t>& uname_data,
  const std::vector<uint8_t>& cmd_data)
{
  // TODO
}

void
FilesystemClientHandler::close()
{
  delete this;
}
