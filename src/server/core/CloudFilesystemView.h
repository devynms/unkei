#ifndef _SERVER_CORE_CLOUD_FILESYSTEM_VIEW_H_
#define _SERVER_CORE_CLOUD_FILESYSTEM_VIEW_H_

#include <fstream>
#include <string>

class CloudFilesystemView
{
 public:
  bool
  UserExists(std::string username);

  bool
  ResourceExists(std::string username, std::string resource_name);

  std::fstream&&
  LookupResourceFile(std::string username, std::string resource_name);

  std::fstream&&
  LookupResourceMetaFile(std::string username, std::string resource_name);

  std::fstream&&
  LookupUserInfoFile(std::string username);

  void
  Prepare(std::string path);
};

#endif
