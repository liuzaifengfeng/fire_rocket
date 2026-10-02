#pragma once
#include "FS.h"
class MockLittleFS {
public:
    bool formatted=false;
    size_t capacity=0x270000;
    std::map<std::string,std::shared_ptr<std::vector<uint8_t>>> files;
    bool begin(bool) { return formatted; }
    void end() {}
    bool format() { files.clear(); formatted=true; return true; }
    File open(const char* name,const char* mode) {
        if(!formatted) return File();
        bool write=std::string(mode)=="w";
        if(write) files[name]=std::make_shared<std::vector<uint8_t>>();
        auto it=files.find(name);
        return it==files.end()?File():File(it->second,write);
    }
    bool remove(const char* name) { return files.erase(name)>0; }
    size_t totalBytes() const { return capacity; }
    size_t usedBytes() const { size_t n=0; for(const auto& file:files) n+=file.second->size(); return n; }
};
extern MockLittleFS LittleFS;
