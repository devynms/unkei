template <typename Iter>
void
server::core::BufferedReader::ReadBytes(Iter dst, int nbytes) {
  mtx.lock();
  auto pframe = buffer.begin();
  while (nbytes > 0) {
    int to_read = std::min(nbytes, pframe->AvailableReadSpace());
    pframe->Read(dst, to_read);
    nbytes -= to_read;
  }
  mtx.unlock();
}

template <typename Iter>
int
server::core::BufferedReader::ReadAllBytes(Iter dst, int nbytes) {
  // Read bytes if possible and return number of bytes read
  if (this->AvailableReadSpace() < nbytes) {
    return 0;
  } else {
    this->ReadBytes(dst, nbytes);
    return nbytes;
  }
}

template <typename Iter>
int
server::core::BufferedReader::ReadSomeBytes(Iter dst, int nbytes) {
  int to_read = std::min(this->AvailableReadSpace(), nbytes);
  this->ReadBytes(dst, to_read);
  return to_read;
}

template <typename Iter>
int
server::core::BufferedReader::ReadAvailableBytes(Iter dst) {
  int available_bytes = this->AvailableReadSpace();
  this->ReadBytes(dst, available_bytes);
  return available_bytes;
}

template <typename Iter>
bool
server::core::impl::BufferFrame::Read(Iter& dst, int nbytes) {
  if (nbytes > this->AvailableReadSpace()) {
    return false;
  }

  for (int i = this->read; i < nbytes + this->read; i++) {
    *dst = this->data[i];
    ++dst;
  }
  this->read += nbytes;
  return true;
}

template <typename Iter>
bool
server::core::impl::BufferFrame::Write(Iter& dst, int nbytes) {
  if (nbytes > this->AvailableWriteSpace()) {
    return false;
  }

  for (int i = this->length; i < nbytes + this->length; i++) {
    this->data[i] = *dst;
    ++dst;
  }
  this->length += nbytes;
  return true;
}

template <typename Iter>
void
server::core::BufferedReader::Write(Iter iter, int nbytes) {
  using namespace impl;

  mtx.lock();

  if (buffer.empty()) {
    buffer.emplace_back();
  }

  while (nbytes > 0) {
    BufferFrame* frame = &buffer.back();
    if (frame->AvailableWriteSpace() == 0) {
      buffer.emplace_back();
      frame = &buffer.back();
    }

    int to_write = std::min(nbytes, frame->AvailableWriteSpace());
    frame->Write(iter, to_write);
    nbytes -= to_write;
  }

  mtx.unlock();
}
