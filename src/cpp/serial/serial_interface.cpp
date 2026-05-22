#include "serial/serial_interface.hpp"

namespace omuraisu {
namespace serial {
namespace {

uint16_t clamp_len(uint16_t len) {
  return len > SERIAL_MESSAGE_MAX_LEN ? SERIAL_MESSAGE_MAX_LEN : len;
}

void copy_data(uint8_t* dst, const uint8_t* src, uint16_t len) {
  uint16_t bounded_len = clamp_len(len);
  for (uint16_t i = 0; i < SERIAL_MESSAGE_MAX_LEN; ++i) {
    dst[i] = i < bounded_len ? src[i] : 0U;
  }
}

}  // namespace

SerialMessage::SerialMessage() noexcept : ::SerialMessage{{0}, 0} {}

SerialMessage::SerialMessage(const uint8_t* raw_data,
                             uint16_t len_value) noexcept
    : ::SerialMessage{{0}, clamp_len(len_value)} {
  if (raw_data != nullptr) {
    copy_data(data, raw_data, len);
  }
}

SerialMessage::SerialMessage(const ::SerialMessage& other) noexcept
    : ::SerialMessage{{0}, clamp_len(other.len)} {
  copy_data(data, other.data, len);
}

SerialMessage::SerialMessage(const SerialMessage& other) noexcept
    : ::SerialMessage{{0}, clamp_len(other.len)} {
  copy_data(data, other.data, len);
}

SerialMessage& SerialMessage::operator=(const SerialMessage& other) noexcept {
  if (this == &other) {
    return *this;
  }
  len = clamp_len(other.len);
  copy_data(data, other.data, len);
  return *this;
}

CSerialPortAdapter::CSerialPortAdapter(::SerialPort* port) noexcept
    : port_(port) {}

bool CSerialPortAdapter::open() {
  if (port_ == nullptr) {
    return false;
  }
  return serial_port_open(port_);
}

void CSerialPortAdapter::close() {
  if (port_ == nullptr) {
    return;
  }
  serial_port_close(port_);
}

bool CSerialPortAdapter::write(const SerialMessage& msg) {
  if (port_ == nullptr) {
    return false;
  }
  return serial_port_write(port_, static_cast<const ::SerialMessage*>(&msg));
}

bool CSerialPortAdapter::read(SerialMessage& msg) {
  if (port_ == nullptr) {
    return false;
  }
  if (!serial_port_read(port_, static_cast<::SerialMessage*>(&msg))) {
    return false;
  }
  return true;
}

void CSerialPortAdapter::start_read() {
  if (port_ == nullptr) {
    return;
  }
  serial_port_start_read(port_);
}

void CSerialPortAdapter::stop_read() {
  if (port_ == nullptr) {
    return;
  }
  serial_port_stop_read(port_);
}

void CSerialPortAdapter::set_rx_callback(SerialRxCallback callback,
                                         void* user_arg) {
  if (port_ == nullptr) {
    return;
  }
  serial_port_set_rx_callback(port_, callback, user_arg);
}

CppSerialPortBridge::CppSerialPortBridge(ISerialPort& port) noexcept
    : port_(&port), c_port_{} {
  c_port_.open = &CppSerialPortBridge::open_thunk;
  c_port_.close = &CppSerialPortBridge::close_thunk;
  c_port_.write = &CppSerialPortBridge::write_thunk;
  c_port_.read = &CppSerialPortBridge::read_thunk;
  c_port_.start_read = &CppSerialPortBridge::start_read_thunk;
  c_port_.stop_read = &CppSerialPortBridge::stop_read_thunk;
  c_port_.set_rx_callback = &CppSerialPortBridge::set_rx_callback_thunk;
  c_port_.destroy = &CppSerialPortBridge::destroy_thunk;
  c_port_.impl = this;
}

::SerialPort* CppSerialPortBridge::c_port() noexcept { return &c_port_; }

const ::SerialPort* CppSerialPortBridge::c_port() const noexcept {
  return &c_port_;
}

bool CppSerialPortBridge::open_thunk(void* self) {
  if (self == nullptr) {
    return false;
  }
  CppSerialPortBridge* bridge = static_cast<CppSerialPortBridge*>(self);
  if (bridge->port_ == nullptr) {
    return false;
  }
  return bridge->port_->open();
}

void CppSerialPortBridge::close_thunk(void* self) {
  if (self == nullptr) {
    return;
  }
  CppSerialPortBridge* bridge = static_cast<CppSerialPortBridge*>(self);
  if (bridge->port_ == nullptr) {
    return;
  }
  bridge->port_->close();
}

bool CppSerialPortBridge::write_thunk(void* self, const ::SerialMessage* msg) {
  if (self == nullptr || msg == nullptr) {
    return false;
  }
  CppSerialPortBridge* bridge = static_cast<CppSerialPortBridge*>(self);
  if (bridge->port_ == nullptr) {
    return false;
  }
  const SerialMessage& cpp_msg = *static_cast<const SerialMessage*>(msg);
  return bridge->port_->write(cpp_msg);
}

bool CppSerialPortBridge::read_thunk(void* self, ::SerialMessage* msg) {
  if (self == nullptr || msg == nullptr) {
    return false;
  }
  CppSerialPortBridge* bridge = static_cast<CppSerialPortBridge*>(self);
  if (bridge->port_ == nullptr) {
    return false;
  }
  SerialMessage& cpp_msg = *static_cast<SerialMessage*>(msg);
  return bridge->port_->read(cpp_msg);
}

void CppSerialPortBridge::start_read_thunk(void* self) {
  if (self == nullptr) {
    return;
  }
  CppSerialPortBridge* bridge = static_cast<CppSerialPortBridge*>(self);
  if (bridge->port_ == nullptr) {
    return;
  }
  bridge->port_->start_read();
}

void CppSerialPortBridge::stop_read_thunk(void* self) {
  if (self == nullptr) {
    return;
  }
  CppSerialPortBridge* bridge = static_cast<CppSerialPortBridge*>(self);
  if (bridge->port_ == nullptr) {
    return;
  }
  bridge->port_->stop_read();
}

void CppSerialPortBridge::set_rx_callback_thunk(void* self,
                                                ::SerialRxCallback callback,
                                                void* user_arg) {
  if (self == nullptr) {
    return;
  }
  CppSerialPortBridge* bridge = static_cast<CppSerialPortBridge*>(self);
  if (bridge->port_ == nullptr) {
    return;
  }
  bridge->port_->set_rx_callback(callback, user_arg);
}

void CppSerialPortBridge::destroy_thunk(void* self) { (void)self; }

}  // namespace serial
}  // namespace omuraisu
