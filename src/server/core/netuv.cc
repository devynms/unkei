#include "netuv.h"
#include <stdexcept>

#define UNUSED(x) (void)(x)

const int DEFAULT_BACKLOG = 128;

uv::EventLoop::EventLoop() {
  uv_loop_init(&this->loop);
}

uv::EventLoop::~EventLoop() {
  uv_loop_close(&this->loop);
}

int
uv::EventLoop::Run() {
  return uv_run(&this->loop, UV_RUN_DEFAULT);
}

uv::ServerSocket::ServerSocket(EventLoop* loop,
      std::function<void(ClientSocket*)> cb)
    : on_new_connection_cb(cb) {
  uv_tcp_init(&loop->loop, &this->tcp_connection);
  this->tcp_connection.data = (void*) this;
  this->loop = loop;
}

bool
uv::ServerSocket::Bind(const char * ip, int port) {
  struct sockaddr_in addr;
  uv_ip4_addr(ip, port, &addr);
  int result = uv_tcp_bind(&this->tcp_connection, (const sockaddr*)&addr, 0);

  if (result >= 0) {
    return true;
  } else if (result == UV_EADDRINUSE) {
    return false;
  } else {
    throw std::runtime_error("uv_tcp_bind: unexpected error");
  }
}

extern "C" void uv_server_on_new_connection(uv_stream_t* server, int status) {
  uv::ServerSocket * srv = (uv::ServerSocket*) server->data;
  srv->OnNewConnection(status);
}

bool
uv::ServerSocket::Listen() {
  int r = uv_listen((uv_stream_t*) &this->tcp_connection,
                    DEFAULT_BACKLOG,
                    uv_server_on_new_connection);
  if (r) {
    return false;
  } else {
    return true;
  }
}

void
uv::ServerSocket::OnNewConnection(int status) {
  if (status < 0) {
    // log error
    return;
  }

  ClientSocket* client = new ClientSocket(this->loop);
  uv_tcp_t* client_conn = &client->tcp_connection;
  if (uv_accept((uv_stream_t*) &this->tcp_connection,
                (uv_stream_t*) client_conn) == 0) {
    this->on_new_connection_cb(client);
  } else {
    delete client;
  }
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

const char *
uv::Buffer::data() {
  return (const char*) this->buf.data();
}

ssize_t
uv::Buffer::length() const {
  return this->buf.size();
}

uv::ClientSocket::ClientSocket(uv::EventLoop * loop) {
  uv_tcp_init(&loop->loop, &this->tcp_connection);
  this->tcp_connection.data = (void*) this;
}

uv::ClientSocket::~ClientSocket() {
  uv_close((uv_handle_t*) &this->tcp_connection, NULL);
}

void
uv::ClientSocket::SetOnRead(std::function<void(Buffer, bool)> read_cb) {
  this->on_read_done = read_cb;
}

void
uv::ClientSocket::SetOnWrite(std::function<void(bool)> write_cb) {
  this->on_write_done = write_cb;
}

extern "C" void alloc_buffer(
    uv_handle_t *handle,
    size_t suggested_size,
    uv_buf_t *buf) {
  UNUSED(handle);
  buf->base = new char[suggested_size];
  buf->len = suggested_size;
}

extern "C" void uv_client_on_read_done(
    uv_stream_t *client,
    ssize_t nread,
    const uv_buf_t *buf) {
  uv::ClientSocket* my_client = (uv::ClientSocket*) client->data;
  my_client->OnReadDone(buf, nread);
}

void
uv::ClientSocket::StartRead() {
  uv_read_start((uv_stream_t*)&this->tcp_connection,
                alloc_buffer,
                uv_client_on_read_done);
}

void
uv::ClientSocket::OnReadDone(const uv_buf_t* buf, ssize_t nread) {
  bool success = true;
  if (nread < 0) {
    if (nread != UV_EOF) {
      // log error
    }
    success = false;
  }

  this->on_read_done(Buffer(buf->base, nread), success);

  if (buf->base) {
    delete[] buf->base;
  }
}

extern "C" void uv_client_on_write_done(uv_write_t *req, int status) {
  uv::ClientSocket* client = (uv::ClientSocket*) req->data;
  client->OnWriteDone(status);
  free(req);
}

void
uv::ClientSocket::StartWrite(Buffer buf) {
  uv_write_t* req = (uv_write_t*) malloc(sizeof(uv_write_t));
  req->data = (void*) this;
  uv_buf_t wrbuf = uv_buf_init((char *)buf.data(), buf.length());
  uv_write(req, (uv_stream_t*)&this->tcp_connection, &wrbuf, 1,
            uv_client_on_write_done);
}

void
uv::ClientSocket::OnWriteDone(int status) {
  bool success = true;
  if (status < 0) {
    success = false;
  }
  this->on_write_done(success);
}
