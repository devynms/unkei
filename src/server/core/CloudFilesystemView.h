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

  std::string
  LookupResourceFile(std::string username, std::string resource_name);

  std::string
  LookupResourceMetaFile(std::string username, std::string resource_name);

  std::string
  LookupUserInfoFile(std::string username);

  void
  Prepare(std::string path);

  void
  Prepare(std::string username, std::string resource_name, bool meta=false);
};

#endif
