#include "ccuv.h"

uv::EventLoop::EventLoop() {
  uv_loop_init(&this->loop);
}

uv::EventLoop::~EventLoop() {
  uv_loop_close(&this->loop);
}

uv_loop_t*
uv::EventLoop::data() {
  return &this->loop;
}

int
uv::EventLoop::Run() {
  return uv_run(&this->loop, UV_RUN_DEFAULT);
}

uv::Buffer::Buffer(char * buf, ssize_t len) {
  if (len > 0) {
    this->buf.reserve(len);
    for (int i = 0; i < len; i++) {
      this->buf.push_back(*buf);
      ++buf;
    }
  }
}

uv::Buffer::Buffer(std::vector<byte>&& buf)
    : buf(std::move(buf))
{}

uv::Buffer::Buffer(int reserve)
    : buf(reserve)
{}

const char *
uv::Buffer::data() const {
  return (const char*) this->buf.data();
}

char*
uv::Buffer::data() {
  return (char*) this->buf.data();
}

ssize_t
uv::Buffer::length() const {
  return this->buf.size();
}

void
uv::Buffer::write(const byte* bytes, int nbytes) {
  for (int i = 0; i < nbytes; i++) {
    buf[i] = bytes[i];
  }
}
