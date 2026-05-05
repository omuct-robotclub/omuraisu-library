#include "serial/serial_mbed.hpp"

// このファイルはmbed環境でのみ使用可能
#if defined(__has_include)
#if __has_include("mbed.h")
#include "mbed.h"

namespace omuraisu {
namespace serial {

MbedSerialPort::MbedSerialPort(PinName tx, PinName rx, int baudrate)
    : serial_(new BufferedSerial(tx, rx, baudrate)),
      owned_(true),
      rx_callback_(nullptr),
      rx_callback_user_arg_(nullptr),
      rx_head_(0),
      rx_tail_(0),
      rx_count_(0) {
  serial_->set_blocking(false);
}

MbedSerialPort::MbedSerialPort(BufferedSerial& serial)
    : serial_(&serial),
      owned_(false),
      rx_callback_(nullptr),
      rx_callback_user_arg_(nullptr),
      rx_head_(0),
      rx_tail_(0),
      rx_count_(0) {
  serial_->set_blocking(false);
}

MbedSerialPort::~MbedSerialPort() {
  stop_read();
  if (owned_) {
    delete serial_;
  }
}

bool MbedSerialPort::open() { return true; }

void MbedSerialPort::close() { stop_read(); }

bool MbedSerialPort::write(const SerialMessage& msg) {
  if (serial_ == nullptr || msg.len == 0) {
    return false;
  }
  ssize_t written = serial_->write(msg.data, msg.len);
  return written == msg.len;
}

bool MbedSerialPort::read(SerialMessage& msg) {
  if (serial_ == nullptr) {
    return false;
  }

  msg.len = 0;
  if (rx_available() > 0) {
    uint16_t count = 0;
    while (count < SERIAL_MESSAGE_MAX_LEN) {
      uint8_t value = 0;
      if (!rx_pop(&value)) {
        break;
      }
      msg.data[count++] = value;
    }
    msg.len = count;
    return count > 0;
  }

  if (!serial_->readable()) {
    return false;
  }

  uint8_t buffer[SERIAL_MESSAGE_MAX_LEN];
  ssize_t read_bytes = serial_->read(buffer, sizeof(buffer));
  if (read_bytes <= 0) {
    return false;
  }

  msg.len = static_cast<uint16_t>(read_bytes);
  for (uint16_t i = 0; i < msg.len; ++i) {
    msg.data[i] = buffer[i];
  }
  return true;
}

void MbedSerialPort::start_read() {
  if (serial_ == nullptr) {
    return;
  }
  serial_->sigio(mbed::callback(this, &MbedSerialPort::on_rx_irq));
}

void MbedSerialPort::stop_read() {
  if (serial_ == nullptr) {
    return;
  }
  serial_->sigio(nullptr);
}

void MbedSerialPort::set_rx_callback(SerialRxCallback callback,
                                     void* user_arg) {
  rx_callback_ = callback;
  rx_callback_user_arg_ = user_arg;
}

BufferedSerial& MbedSerialPort::get_serial() { return *serial_; }

void MbedSerialPort::on_rx_irq() {
  if (serial_ == nullptr) {
    return;
  }

  uint8_t buffer[SERIAL_MESSAGE_MAX_LEN];
  while (serial_->readable()) {
    ssize_t read_bytes = serial_->read(buffer, sizeof(buffer));
    if (read_bytes <= 0) {
      break;
    }

    uint16_t len = static_cast<uint16_t>(read_bytes);
    for (uint16_t i = 0; i < len; ++i) {
      rx_push(buffer[i]);
    }

    if (rx_callback_ != nullptr) {
      SerialMessage msg(buffer, len);
      rx_callback_(&msg, rx_callback_user_arg_);
    }
  }
}

bool MbedSerialPort::rx_push(uint8_t value) {
  if (rx_count_ >= OMURAISU_SERIAL_MBED_RX_BUFFER_SIZE) {
    return false;
  }

  rx_buffer_[rx_head_] = value;
  rx_head_ = (uint16_t)((rx_head_ + 1U) % OMURAISU_SERIAL_MBED_RX_BUFFER_SIZE);
  rx_count_++;
  return true;
}

bool MbedSerialPort::rx_pop(uint8_t* value) {
  if (rx_count_ == 0U) {
    return false;
  }

  *value = rx_buffer_[rx_tail_];
  rx_tail_ = (uint16_t)((rx_tail_ + 1U) % OMURAISU_SERIAL_MBED_RX_BUFFER_SIZE);
  rx_count_--;
  return true;
}

uint16_t MbedSerialPort::rx_available() const { return rx_count_; }

}  // namespace serial
}  // namespace omuraisu

#endif
#endif  // __has_include check
