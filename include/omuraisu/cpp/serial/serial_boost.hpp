#ifndef OMURAISU_CPP_SERIAL_SERIAL_BOOST_HPP_
#define OMURAISU_CPP_SERIAL_SERIAL_BOOST_HPP_

#include "serial/serial_interface.hpp"

// このファイルはBoost.Asioが利用可能な環境でのみ使用可能
#if defined(__has_include)
#if __has_include(<boost/asio.hpp>)
#include <boost/asio.hpp>
#include <string>

namespace omuraisu {
namespace serial {

#ifndef OMURAISU_SERIAL_BOOST_RX_BUFFER_SIZE
#define OMURAISU_SERIAL_BOOST_RX_BUFFER_SIZE 512
#endif

class BoostSerialPort : public ISerialPort {
 public:
  BoostSerialPort(boost::asio::io_context& io, const std::string& device,
                  uint32_t baudrate);
  ~BoostSerialPort() override;

  BoostSerialPort(const BoostSerialPort&) = delete;
  BoostSerialPort& operator=(const BoostSerialPort&) = delete;

  bool open() override;
  void close() override;
  bool write(const SerialMessage& msg) override;
  bool read(SerialMessage& msg) override;
  void start_read() override;
  void stop_read() override;
  void set_rx_callback(SerialRxCallback callback, void* user_arg) override;

 private:
  void async_read_some();
  void on_read(const boost::system::error_code& ec, std::size_t bytes);
  bool rx_push(uint8_t value);
  bool rx_pop(uint8_t* value);
  uint16_t rx_available() const;

  boost::asio::io_context& io_;
  boost::asio::serial_port serial_;
  std::string device_;
  uint32_t baudrate_;

  bool reading_;

  SerialRxCallback rx_callback_;
  void* rx_callback_user_arg_;

  uint8_t rx_buffer_[OMURAISU_SERIAL_BOOST_RX_BUFFER_SIZE];
  uint16_t rx_head_;
  uint16_t rx_tail_;
  uint16_t rx_count_;

  uint8_t temp_buffer_[SERIAL_MESSAGE_MAX_LEN];
};

}  // namespace serial
}  // namespace omuraisu

#endif
#endif  // __has_include check

#endif  // OMURAISU_CPP_SERIAL_SERIAL_BOOST_HPP_
