#ifndef _SERVER_CORE_BUFFERED_READER_H_
#define _SERVER_CORE_BUFFERED_READER_H_

#include <array>
#include <vector>
#include <list>
#include <iterator>
#include <utility>
#include <mutex>

#include "ccuv.h"

namespace server {
namespace core {

using byte = unsigned char;

namespace impl {

class BufferFrame {
 public:
  static const int FRAME_SIZE = 1024;

  BufferFrame();
  virtual ~BufferFrame() = default;
  BufferFrame(const BufferFrame&) = delete;
  BufferFrame(BufferFrame&&) = delete;
  BufferFrame& operator=(const BufferFrame&) = delete;

  const std::array<byte, FRAME_SIZE>& Data() const;
  int AvailableReadSpace() const;
  int AvailableWriteSpace() const;
  bool IsFull() const;
  bool IsRead() const;

  template <typename Iter>
  bool Read(Iter& dst, int nbytes);

  template <typename Iter>
  bool Write(Iter& start, int nbytes);
 private:
  std::array<byte, FRAME_SIZE> data;
  int length;
  int read;
};

}

class BufferedReader {
 public:
  BufferedReader() = default;
  virtual ~BufferedReader() = default;
  BufferedReader(const BufferedReader&) = delete;
  BufferedReader(BufferedReader&&) = delete;
  BufferedReader& operator=(BufferedReader&) = delete;

  bool ReadInt(int& out);
  bool ReadMsbInt(int& out);
  bool ReadLsbInt(int& out);

  template <typename Iter>
  void Write(Iter start, int nbytes);

  template <typename Iter>
  int ReadAllBytes(Iter dst, int nbytes);

  template <typename Iter>
  int ReadSomeBytes(Iter dst, int nbytes);

  template <typename Iter>
  int ReadAvailableBytes(Iter dst);

  int AvailableReadSpace() const;

 private:
  template <typename Iter>
  void ReadBytes(Iter dst, int nbytes);

  std::list<impl::BufferFrame> buffer;
  std::mutex mtx;
};

}
}

#include "BufferedReader.hh"

#endif // ifndef _SERVER_CORE_BUFFERED_READER_H_
