#ifndef OMURAISU_CPP_SERIAL_SERIAL_MBED_HPP_
#define OMURAISU_CPP_SERIAL_SERIAL_MBED_HPP_

#include "serial/serial_interface.hpp"

// このファイルはmbed環境でのみ使用可能
#if defined(__has_include)
#if __has_include("mbed.h")
#include "mbed.h"

namespace omuraisu {
namespace serial {

#ifndef OMURAISU_SERIAL_MBED_RX_BUFFER_SIZE
#define OMURAISU_SERIAL_MBED_RX_BUFFER_SIZE 256
#endif

class MbedSerialPort : public ISerialPort {
 public:
  MbedSerialPort(PinName tx, PinName rx, int baudrate = 115200);
  explicit MbedSerialPort(BufferedSerial& serial);
  ~MbedSerialPort() override;

  MbedSerialPort(const MbedSerialPort&) = delete;
  MbedSerialPort& operator=(const MbedSerialPort&) = delete;

  bool open() override;
  void close() override;
  bool write(const SerialMessage& msg) override;
  bool read(SerialMessage& msg) override;
  void start_read() override;
  void stop_read() override;
  void set_rx_callback(SerialRxCallback callback, void* user_arg) override;

  BufferedSerial& get_serial();

 private:
  void on_rx_irq();
  bool rx_push(uint8_t value);
  bool rx_pop(uint8_t* value);
  uint16_t rx_available() const;

  BufferedSerial* serial_;
  bool owned_;

  SerialRxCallback rx_callback_;
  void* rx_callback_user_arg_;

  uint8_t rx_buffer_[OMURAISU_SERIAL_MBED_RX_BUFFER_SIZE];
  uint16_t rx_head_;
  uint16_t rx_tail_;
  uint16_t rx_count_;
};

}  // namespace serial
}  // namespace omuraisu

#endif
#endif  // __has_include check

#endif  // OMURAISU_CPP_SERIAL_SERIAL_MBED_HPP_
