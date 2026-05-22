#include "serial/serial_boost.hpp"

// このファイルはBoost.Asioが利用可能な環境でのみ使用可能
#if defined(__has_include)
#if __has_include(<boost/asio.hpp>)
#include <boost/asio.hpp>

namespace omuraisu {
namespace serial {

BoostSerialPort::BoostSerialPort(boost::asio::io_context& io,
                                 const std::string& device, uint32_t baudrate)
    : io_(io),
      serial_(io),
      device_(device),
      baudrate_(baudrate),
      reading_(false),
      rx_callback_(nullptr),
      rx_callback_user_arg_(nullptr),
      rx_head_(0),
      rx_tail_(0),
      rx_count_(0) {}

BoostSerialPort::~BoostSerialPort() { close(); }

bool BoostSerialPort::open() {
  if (serial_.is_open()) {
    return true;
  }

  boost::system::error_code ec;
  serial_.open(device_, ec);
  if (ec) {
    return false;
  }

  serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_), ec);
  if (ec) {
    serial_.close();
    return false;
  }
  serial_.set_option(boost::asio::serial_port_base::character_size(8), ec);
  if (ec) {
    serial_.close();
    return false;
  }
  serial_.set_option(boost::asio::serial_port_base::parity(
                         boost::asio::serial_port_base::parity::none),
                     ec);
  if (ec) {
    serial_.close();
    return false;
  }
  serial_.set_option(boost::asio::serial_port_base::stop_bits(
                         boost::asio::serial_port_base::stop_bits::one),
                     ec);
  if (ec) {
    serial_.close();
    return false;
  }
  serial_.set_option(boost::asio::serial_port_base::flow_control(
                         boost::asio::serial_port_base::flow_control::none),
                     ec);
  if (ec) {
    serial_.close();
    return false;
  }

  return true;
}

void BoostSerialPort::close() {
  stop_read();
  if (serial_.is_open()) {
    boost::system::error_code ec;
    serial_.close(ec);
  }
}

bool BoostSerialPort::write(const SerialMessage& msg) {
  if (!serial_.is_open() || msg.len == 0) {
    return false;
  }

  boost::system::error_code ec;
  std::size_t written =
      boost::asio::write(serial_, boost::asio::buffer(msg.data, msg.len), ec);
  return !ec && written == msg.len;
}

bool BoostSerialPort::read(SerialMessage& msg) {
  msg.len = 0;
  if (rx_available() == 0) {
    return false;
  }

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

void BoostSerialPort::start_read() {
  if (reading_ || !serial_.is_open()) {
    return;
  }
  reading_ = true;
  async_read_some();
}

void BoostSerialPort::stop_read() {
  if (!reading_) {
    return;
  }
  reading_ = false;
  if (serial_.is_open()) {
    boost::system::error_code ec;
    serial_.cancel(ec);
  }
}

void BoostSerialPort::set_rx_callback(SerialRxCallback callback,
                                      void* user_arg) {
  rx_callback_ = callback;
  rx_callback_user_arg_ = user_arg;
}

void BoostSerialPort::async_read_some() {
  if (!reading_ || !serial_.is_open()) {
    return;
  }

  serial_.async_read_some(
      boost::asio::buffer(temp_buffer_, sizeof(temp_buffer_)),
      [this](const boost::system::error_code& ec, std::size_t bytes) {
        on_read(ec, bytes);
      });
}

void BoostSerialPort::on_read(const boost::system::error_code& ec,
                              std::size_t bytes) {
  if (ec || !reading_) {
    return;
  }

  uint16_t len = static_cast<uint16_t>(bytes);
  for (uint16_t i = 0; i < len; ++i) {
    rx_push(temp_buffer_[i]);
  }

  if (rx_callback_ != nullptr && len > 0) {
    SerialMessage msg(temp_buffer_, len);
    rx_callback_(&msg, rx_callback_user_arg_);
  }

  async_read_some();
}

bool BoostSerialPort::rx_push(uint8_t value) {
  if (rx_count_ >= OMURAISU_SERIAL_BOOST_RX_BUFFER_SIZE) {
    return false;
  }

  rx_buffer_[rx_head_] = value;
  rx_head_ = (uint16_t)((rx_head_ + 1U) % OMURAISU_SERIAL_BOOST_RX_BUFFER_SIZE);
  rx_count_++;
  return true;
}

bool BoostSerialPort::rx_pop(uint8_t* value) {
  if (rx_count_ == 0U) {
    return false;
  }

  *value = rx_buffer_[rx_tail_];
  rx_tail_ = (uint16_t)((rx_tail_ + 1U) % OMURAISU_SERIAL_BOOST_RX_BUFFER_SIZE);
  rx_count_--;
  return true;
}

uint16_t BoostSerialPort::rx_available() const { return rx_count_; }

}  // namespace serial
}  // namespace omuraisu

#endif
#endif  // __has_include check
