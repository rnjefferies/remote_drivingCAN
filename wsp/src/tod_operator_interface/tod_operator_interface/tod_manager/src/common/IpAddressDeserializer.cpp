// Copyright 2021 Feiler

#include "IpAddressDeserializer.h"

IpAddressDeserializer::IpAddressDeserializer(const std::string& pathToInitialIpAddressFile,
const std::string& searchedKey) :
_pathToInitialIpAddressFile(pathToInitialIpAddressFile),
_searchedKey(searchedKey) {
}

void IpAddressDeserializer::execute() {
    catchAndPrintErrorsWhenGettingNodeAtSearchedKey();
    pushIpAddressesIntoVector();
}

void IpAddressDeserializer::catchAndPrintErrorsWhenGettingNodeAtSearchedKey() {
    if ( !std::ifstream(_pathToInitialIpAddressFile) ) {
        printf("No correct _pathToInitialIpAddressFile provided (%s)\n",
        _pathToInitialIpAddressFile.c_str());
        return;
    }
    YAML::Node data = YAML::LoadFile(_pathToInitialIpAddressFile);
    if ( !data.IsMap() ) {
        printf("Yaml wrongly formatted (%s)\n",
        _pathToInitialIpAddressFile.c_str());
        return;
    }
    if ( !data[_searchedKey].IsDefined() ) {
        printf("Could not found key: %s\n",
        _searchedKey.c_str());
        return;
    }
    _node = data[_searchedKey];
    if ( !_node.IsSequence() ) {
        printf("No Sequence provided under key %s in %s\n",
        _searchedKey.c_str(), _pathToInitialIpAddressFile.c_str());
        return;
    }
}

void IpAddressDeserializer::pushIpAddressesIntoVector() {
    for ( int position = 0; position < _node.size(); ++position ) {
        std::string ipAddress = _node[position].as<std::string>().c_str();
        _ipAddresses.push_back(ipAddress);
    }
}

std::vector<std::string> IpAddressDeserializer::getIpAddresses() {
    return _ipAddresses;
}
