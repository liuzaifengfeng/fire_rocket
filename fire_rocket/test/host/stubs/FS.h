#pragma once
#include "Arduino.h"
#include <memory>
#include <vector>
#include <map>
#include <limits>
extern size_t mockWriteLimit;
class File {
public:
    File() {}
    File(std::shared_ptr<std::vector<uint8_t>> data,bool writable):data_(data),writable_(writable) {}
    explicit operator bool() const { return bool(data_); }
    size_t size() const { return data_?data_->size():0; }
    size_t available() const { return data_&&at_<data_->size()?data_->size()-at_:0; }
    void close() { data_.reset(); at_=0; }
    void flush() {}
    size_t write(const uint8_t* src,size_t n) {
        if(!data_||!writable_) return 0;
        n=std::min(n,mockWriteLimit);
        if(data_->size()<at_+n) data_->resize(at_+n);
        memcpy(data_->data()+at_,src,n); at_+=n; return n;
    }
    size_t read(uint8_t* dst,size_t n) {
        n=std::min(n,available());
        if(n) memcpy(dst,data_->data()+at_,n);
        at_+=n; return n;
    }
    size_t readBytes(char* dst,size_t n) { return read(reinterpret_cast<uint8_t*>(dst),n); }
    size_t printf(const char* fmt,...) {
        char b[2048]; va_list args; va_start(args,fmt); int n=vsnprintf(b,sizeof(b),fmt,args); va_end(args);
        return write(reinterpret_cast<uint8_t*>(b),size_t(std::max(0,n)));
    }
private:
    std::shared_ptr<std::vector<uint8_t>> data_;
    bool writable_=false;
    size_t at_=0;
};
