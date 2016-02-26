#include "ClientHandler.h"

server::core::ClientHandler::ClientHandler(
  uv::EventLoop* loop,
  uv::ClientSocket* sock,
  int id)
    : connection(sock),
      connection_closed(false),
      output_file(loop),
      file_closed(false),
      sm(this)
{
  this->connection->OnRead([=](uv::Buffer buf, bool success) {
    this->on_socket_read(std::move(buf), success);
  });

  this->connection->OnWrite([=](bool success) {
    this->on_socket_write(success);
  });

  this->connection->OnClose([=]() {
    if (this->file_closed) {
      this->destroy();
    } else {
      this->connection_closed = true;
    }
  });

  this->output_file.OnRead([=](int read) {
    UNUSED(read);
    // TODO: fatal logic error
  });

  this->output_file.OnOpen([=](bool status) {
    if (status == false) {
      this->Close();
    }
  });

  this->output_file.OnClose([=](bool status) {
    UNUSED(status);
    // TODO: use status
    if (this->connection_closed) {
      this->destroy();
    } else {
      this->file_closed = true;
    }
  });

  this->output_file.OnWrite([=](int written) {
      this->on_file_write(written);
  });

  this->output_file.SetPath("File_" + id);
  this->output_file.StartOpen(O_WRONLY | O_CREAT, S_IRWXU);
}

server::core::ClientHandler::~ClientHandler() {
  delete connection;
}

void
server::core::ClientHandler::on_socket_read(uv::Buffer buf, bool success) {
  using namespace server::core::impl;

  if (!success) {
    // TODO: log
    this->Close();
  }

  this->sm.AcceptBytes(buf);
}

void
server::core::ClientHandler::on_socket_write(bool success) {
  if (!success) {
    // TODO: log
    this->Close();
  }
}

void
server::core::ClientHandler::on_file_write(int bytes) {
  this->sm.ConfirmWrite(bytes);
  if(this->sm.Done()) {
    this->Close();
  }
}

void
server::core::ClientHandler::Close() {
  this->connection->StartClose();
  this->output_file.StartClose();
}

  // INITIAL STATE: expected_file_size = -1
  //  no bytes have been read, must determine size of payload
  // READING: expected_file_size != -1, bytes_written < expected_file_size
  //  in the process of writing to the file
  // DONE: expected_file_size != -1, bytes_written == expected_file_size
  //  time to clean up connection and file
  // ERROR: {}
  //  clean up connection and delete file

server::core::impl::ClientHandlerStateMachine::ClientHandlerStateMachine(
  ClientHandler* parent)
    : parent(parent),
      error_occured(false),
      file_opened(false),
      expected_file_size(-1),
      bytes_read(0),
      bytes_written(0),
      bytes_confirmed(0)
{}

server::core::impl::ClientHandlerState
server::core::impl::ClientHandlerStateMachine::CurrentState() const {
  if (error_occured) {
    return ClientHandlerState::ERROR;

  } else if (expected_file_size == -1) {
    return ClientHandlerState::INITIAL;

  } else if (file_opened == false) {
    return ClientHandlerState::WAITING;

  } else if (bytes_written < expected_file_size) {
    return ClientHandlerState::READING;

  } else if (bytes_written == expected_file_size &&
             bytes_confirmed != expected_file_size) {
    return ClientHandlerState::WRITING;

  } else if (bytes_written == expected_file_size &&
             bytes_confirmed == expected_file_size) {
    return ClientHandlerState::DONE;

  } else {
    return ClientHandlerState::INVALID;
  }
}

bool
server::core::impl::ClientHandlerStateMachine::Done() const {
  ClientHandlerState current = this->CurrentState();
  if (current == ClientHandlerState::DONE ||
      current == ClientHandlerState::ERROR) {
    return true;
  } else {
    return false;
  }
}

void
server::core::impl::ClientHandlerStateMachine::AcceptBytes(
    const uv::Buffer& buf) {
  const byte* data = buf.data();
  this->reader.Write(data, buf.length());
  this->UpdateStateMachine();
}

void
server::core::impl::ClientHandlerStateMachine::UpdateStateMachine() {
  ClientHandlerState current = this->CurrentState();
  switch(current) {

    case ClientHandlerState::INITIAL: {
      int read_int;
      if (reader.ReadInt(read_int)) {
        this->expected_file_size = read_int;
      } else {
        break;
      }
    }

    case ClientHandlerState::WAITING: {
      // do nothing... we need to wait for the file to be open
      break;
    }

    case ClientHandlerState::READING: {
      int bytes_available = reader.AvailableReadSpace();
      std::vector<server::core::byte> buffer(bytes_available);
      reader.ReadAvailableBytes(buffer.begin());
      this->parent->PostBytes(uv::Buffer(std::move(buffer)));
      this->bytes_read += bytes_available;
      break;
    }

    case ClientHandlerState::WRITING: {
      // do nothing... we need to wait for the bytes to be confirmed
      break;
    }

    case ClientHandlerState::DONE:
      this->error_occured = true;
      // if this function is called again, it will register as an error
      break;

    case ClientHandlerState::ERROR:
      this->parent->Close();
      // TODO: log error
      break;

    case ClientHandlerState::INVALID:
      this->parent->Close();
      // TODO: logic error
      break;
  }
}

void
server::core::impl::ClientHandlerStateMachine::ConfirmWrite(int bytes) {
  this->bytes_confirmed += bytes;
  this->UpdateStateMachine();
}

void
server::core::impl::ClientHandlerStateMachine::FileOpened() {
  this->file_opened = true;
  this->UpdateStateMachine();
}

void
server::core::ClientHandler::destroy() {
  delete this;
}

void
server::core::ClientHandler::PostBytes(const uv::Buffer& buf) {
  this->output_file.StartWrite(buf, -1);
}
