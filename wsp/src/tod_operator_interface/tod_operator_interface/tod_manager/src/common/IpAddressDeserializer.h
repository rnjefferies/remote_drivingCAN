// Copyright 2021 Feiler
#include <string>
#include <vector>
#include <tod_core/YamlLoader.h>

namespace IpAddressDeserializer {

std::vector<std::string> load(const std::string& filePath, const std::string& key)  {
    YamlLoader loader;
    loader.load_from_path(filePath);
    return loader.get_param<std::vector<std::string>>(key);
}

}
