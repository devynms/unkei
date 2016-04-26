#include "SpookyHash.h"
#include "CloudFilesystemView.cc"
#include "json.hpp"

#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using json = nlohmann::json;

int main()
{
  string uname = "user";
  string rpath = "info/" + resource_path(uname, "info");
  CloudFilesystemView view;
  view.Prepare(rpath);
  json infoJ = {
    {"username", "user"},
    {"data", json::array()},
    {"timestamp", 1}
  };
  cout << rpath << endl;
  cout << infoJ << endl;
  fstream file(rpath, fstream::out);
  file << infoJ.dump();
  file.close();
}
