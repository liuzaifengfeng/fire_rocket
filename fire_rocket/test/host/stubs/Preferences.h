#pragma once
#include "FS.h"
class Preferences {
public:
    std::map<std::string,std::vector<uint8_t>> blobs;
    bool begin(const char*,bool) { return true; }
    size_t getBytesLength(const char* key) const { auto it=blobs.find(key); return it==blobs.end()?0:it->second.size(); }
    size_t getBytes(const char* key,void* ptr,size_t n) { auto& b=blobs[key]; n=std::min(n,b.size()); if(n) memcpy(ptr,b.data(),n); return n; }
    size_t putBytes(const char* key,const void* ptr,size_t n) {
        const auto* p=static_cast<const uint8_t*>(ptr); blobs[key]=std::vector<uint8_t>(p,p+n); return n;
    }
};
