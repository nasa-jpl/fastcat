#include "fastcat/fastcat.h"

int main(int argc, char* argv[])
{
  MSG("MSG");
  MSG_DEBUG("MSG_DEBUG");
  WARNING("WARNING");
  ERROR("ERROR");
  SUCCESS("SUCCESS");

  if (argc != 2) {
    ERROR("Pass input yaml file as first argument");
    return 0;
  }

  std::string filepath(argv[1]);
  MSG("loading Yaml from %s", filepath.c_str());
  YAML::Node node = YAML::LoadFile(filepath);

  fastcat::Manager manager;
  manager.ConfigFromYaml(node);

  SUCCESS("File successfully parsed!");
  return 1;
}
