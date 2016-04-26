#include "JobClientHandler.h"
#include "CloudFilesystemView.h"
#include "json.hpp"

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

JobClientHandler::JobClientHandler(ClientConnection* conn)
  : connection(conn)
{ }

JobClientHandler::~JobClientHandler()
{
  delete this->connection;
}

void
JobClientHandler::handle()
{
  using namespace std;
  using json = nlohmann::json;

  CloudFilesystemView view;
  json infoJ;
  string info_path = view.LookupUserInfoFile("user");
  fstream info(info_path, fstream::in);
  info >> infoJ;
  info.close();
  int count = infoJ["data"].size();
  //cout << count << endl;
  stringstream rname_stream;
  rname_stream << "resource_" << count;
  string rname = rname_stream.str();
  //cout << rname << endl;
  infoJ["data"].push_back(rname);
  //cout << infoJ["data"] << endl;

  view.Prepare("user", rname);
  vector<uint8_t> bytes;
  vector<uint8_t> buf;
  size_t read = 2048;
  while (read == 2048) {
    read = this->connection->Receive(buf, 2048);
    for (int i = 0; i < read; i++) {
      bytes.push_back(buf[i]);
    }
    buf.clear();
  }
  fstream resource(view.LookupResourceFile("user", rname), fstream::out);
  resource.write(bytes.data(), bytes.size());
  resource.close();

  view.Prepare("user", rname, true);
  fstream meta(view.LookupResourceMetaFile("user", rname), fstream::out);
  json fileMetaData = {
    {"resource_name", rname},
    {"timestamp", 1LL},
    {"size", bytes.size()},
    {"filetype", "stl"},
  };
  meta << fileMetaData.dump();
  meta.close();

  fstream info2(info_path, fstream::out);
  info2 << infoJ.dump();
  info2.close();
}

void
JobClientHandler::close()
{

}
