#include "fileuv.h"

uv::File::File(EventLoop* loop, std::string path)
    : path(path) {
  this->loop = loop;
}

uv::File::File(EventLoop* loop) {
  this->loop = loop;
}

void
uv::File::OnRead(std::function<void(int read)> read_cb) {
  this->on_read_done = read_cb;
}

void
uv::File::OnOpen(std::function<void(bool)> open_cb) {
  this->on_open_done = open_cb;
}

void
uv::File::OnClose(std::function<void(bool)> close_cb) {
  this->on_close_done = close_cb;
}

void
uv::File::OnWrite(std::function<void(int written)> write_cb) {
  this->on_write_done = write_cb;
}

extern "C"
void uv_fileuv_file_on_open(uv_fs_t* request) {
  uv::File* file = (uv::File*) request->data;
  file->OpenDone(request->result);
  uv_fs_req_cleanup(request);
  delete request;
}

void
uv::File::StartOpen(int flags, int mode) {
  uv_fs_t* request = new uv_fs_t;
  request->data = this;
  uv_fs_open(this->loop->data(), request, path.data(), flags, mode,
    uv_fileuv_file_on_open);
}

void
uv::File::OpenDone(int status) {
  if (status >= 0) {
    this->handle = status;
  }
  this->on_open_done((status >= 0));
}

extern "C"
void uv_fileuv_file_on_write(uv_fs_t* request) {
  uv::File* file = (uv::File*) request->data;
  file->WriteDone(request->result);
  uv_fs_req_cleanup(request);
  delete request;
}

void
uv::File::StartWrite(const Buffer& buf, int offset) {
  uv_fs_t* request = new uv_fs_t;
  request->data = this;
  uv_buf_t wbuf = uv_buf_init((char*)buf.data(), buf.length());
  uv_fs_write(this->loop->data(), request, this->handle, &wbuf, 1, offset,
    uv_fileuv_file_on_write);
}

void
uv::File::WriteDone(int status) {
  this->on_write_done(status);
}

extern "C"
void uv_fileuv_file_on_read(uv_fs_t* request) {
  uv::File* file = (uv::File*) request->data;
  file->ReadDone(request->result);
  uv_fs_req_cleanup(request);
  delete request;
}

void
uv::File::StartRead(Buffer& buf, int offset) {
  uv_fs_t* request = new uv_fs_t;
  request->data = this;
  uv_buf_t rbuf = uv_buf_init((char*)buf.data(), buf.length());
  uv_fs_read(this->loop->data(), request, this->handle, &rbuf, 1, offset,
    uv_fileuv_file_on_read);
}

void
uv::File::ReadDone(int status) {
  this->on_read_done(status);
}

extern "C"
void uv_fileuv_file_on_close(uv_fs_t* request) {
  uv::File* file = (uv::File*) request->data;
  file->CloseDone(request->result);
  uv_fs_req_cleanup(request);
  delete request;
}

void
uv::File::StartClose() {
  uv_fs_t* request = new uv_fs_t;
  request->data = this;
  uv_fs_close(this->loop->data(), request, this->handle, uv_fileuv_file_on_close);
}

void
uv::File::CloseDone(int status) {
  this->on_close_done((status >= 0));
}

void
uv::File::SetPath(std::string path) {
  // TODO: handle the case in which we try and do things w/o a path
  this->path = path;
}
