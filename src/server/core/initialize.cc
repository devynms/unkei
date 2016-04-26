#include "SpookyHash.h"
#include "CloudFilesystemView.cc"
#include "json.hpp"

#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using json = nlohmann::json;

void
PushFile(string uname, string name, string data)
{
  json fileMetaData = {
    {"resource_name", name},
    {"timestamp", 1LL},
    {"size", data.length()},
    {"filetype", "txt"},
    {"available", "true"}
  };
  string path = resource_path(uname, name);
  string mpath = path + ".META";
  CloudFilesystemView view;
  view.Prepare(path);
  view.Prepare(mpath);
  fstream file(path, fstream::out);
  file << data;
  file.close();
  fstream mfile(mpath, fstream::out);
  mfile << fileMetaData.dump();
  mfile.close();
}

int main()
{
  string uname = "user";
  string rpath = "info/" + resource_path(uname, "info");
  CloudFilesystemView view;
  view.Prepare(rpath);
  json infoJ = {
    {"username", "user"},
    {"data", {"hello", "world"}},
    {"timestamp", 1}
  };
  cout << rpath << endl;
  cout << infoJ << endl;
  fstream file(rpath, fstream::out);
  file << infoJ.dump();
  file.close();

  PushFile("user", "hello", "HELLO world");
  PushFile("user", "world", "hello WORLD");
}
