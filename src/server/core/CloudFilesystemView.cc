#include "CloudFilesystemView.h"
#include "SpookyHash.h"
#include "json.hpp"

#include <string>
#include <sstream>
#include <sys/stat.h>
#include <stdlib.h>
#include <iostream>

static bool
file_exists(const std::string& filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}

static std::string
resource_path(std::string username, std::string resource_name)
{
  std::string data = username + ":" + resource_name;
  union {
    uint64_t u64;
    uint16_t u16[sizeof(uint64_t)/sizeof(uint16_t)];
  } hash1;
  uint64_t hash2;
  hash1.u64 = 0;
  hash2 = 17;
  SpookyHash::Hash128(
    (void*) data.data(), data.length(), &hash1.u64, &hash2);
  std::stringstream path;
  path << std::hex << hash1.u16[0] << "/" << hash1.u16[1] << hash1.u16[2]
       << hash1.u16[3] << hash2;
  return path.str();
}

bool
CloudFilesystemView::UserExists(std::string username)
{
  // TODO
  return true;
}

bool
CloudFilesystemView::ResourceExists(
  std::string username,
  std::string resource_name)
{
  auto rpath = resource_path(username, resource_name);
  return file_exists(rpath);
}

std::string
CloudFilesystemView::LookupResourceFile(
  std::string username,
  std::string resource_name)
{
  std::string rpath = resource_path(username, resource_name);
  return rpath;
}

std::string
CloudFilesystemView::LookupResourceMetaFile(
  std::string username,
  std::string resource_name)
{
  std::string rpath = resource_path(username, resource_name) + ".META";
  return rpath;
}

std::string
CloudFilesystemView::LookupUserInfoFile(std::string username)
{
  std::string rpath = "info/" + resource_path(username, "info");
  return rpath;
}

void
CloudFilesystemView::Prepare(std::string path)
{
  using std::string;
  string cmd = string("mkdir -p ") + path.substr(0, path.find_last_of('/'));
  system(cmd.c_str());
}

void
CloudFilesystemView::Prepare(std::string username, std::string resource_name, bool meta)
{
  using std::string;
  string resource = resource_path(username, resource_name);
  if (meta) {
    resource += ".META";
  }
  this->Prepare(resource);
}
