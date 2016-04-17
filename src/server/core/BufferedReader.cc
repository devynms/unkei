#include "BufferedReader.h"

#include <algorithm>
#include <cstring>

server::core::impl::BufferFrame::BufferFrame()
  : length(0), read(0)
{}

const std::array<server::core::byte,
                 server::core::impl::BufferFrame::FRAME_SIZE>&
server::core::impl::BufferFrame::Data() const {
  return data;
}

int
server::core::impl::BufferFrame::AvailableReadSpace() const {
  return this->length - this->read;
}

int
server::core::impl::BufferFrame::AvailableWriteSpace() const {
  return FRAME_SIZE - this->length;
}

bool
server::core::impl::BufferFrame::IsFull() const {
  return this->length == FRAME_SIZE;
}

bool
server::core::impl::BufferFrame::IsRead() const {
  return this->read == this->length;
}

int
server::core::BufferedReader::AvailableReadSpace() const {
  using namespace impl;

  int total = 0;
  for (const BufferFrame& frame : buffer) {
    total += frame.AvailableReadSpace();
  }
  return total;
}

bool
server::core::BufferedReader::ReadLsbInt(int& out) {
  std::array<byte, 4> buf;
  if (!this->ReadAllBytes(buf.begin(), 4)) {
    return false;
  }
  unsigned result = 0;
  result |= buf[0];
  result |= (buf[1] << 8);
  result |= (buf[2] << 16);
  result |= (buf[3] << 24);
  out = *((int*)&result);
  return true;
}

bool
server::core::BufferedReader::ReadMsbInt(int& out) {
  std::array<byte, 4> buf;
  if(!this->ReadAllBytes(buf.begin(), 4)) {
    return false;
  }
  unsigned result = 0;
  result |= (buf[0] << 24);
  result |= (buf[1] << 16);
  result |= (buf[2] << 8);
  result |= (buf[3]);
  out = *((int*)&result);
  return true;
}

bool
server::core::BufferedReader::ReadInt(int& out) {
  std::array<byte, 4> buf;
  if(!this->ReadAllBytes(buf.begin(), 4)) {
    return false;
  }
  std::memcpy(&out, buf.data(), 4);
  return true;
}

