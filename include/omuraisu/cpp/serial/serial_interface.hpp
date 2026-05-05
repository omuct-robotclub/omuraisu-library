#ifndef OMURAISU_CPP_SERIAL_SERIAL_INTERFACE_HPP_
#define OMURAISU_CPP_SERIAL_SERIAL_INTERFACE_HPP_

#include <cstdint>

#include "serial/serial_interface.h"

namespace omuraisu {
namespace serial {

struct SerialMessage : public ::SerialMessage {
  SerialMessage() noexcept;
  SerialMessage(const uint8_t* data, uint16_t len) noexcept;
  explicit SerialMessage(const ::SerialMessage& other) noexcept;
  SerialMessage(const SerialMessage& other) noexcept;

  SerialMessage& operator=(const SerialMessage& other) noexcept;
};

using SerialRxCallback = ::SerialRxCallback;

class ISerialPort {
 public:
  virtual ~ISerialPort() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool write(const SerialMessage& msg) = 0;
  virtual bool read(SerialMessage& msg) = 0;
  virtual void start_read() {}
  virtual void stop_read() {}
  virtual void set_rx_callback(SerialRxCallback callback, void* user_arg) = 0;
};

class CSerialPortAdapter : public ISerialPort {
 public:
  explicit CSerialPortAdapter(::SerialPort* port) noexcept;

  bool open() override;
  void close() override;
  bool write(const SerialMessage& msg) override;
  bool read(SerialMessage& msg) override;
  void start_read() override;
  void stop_read() override;
  void set_rx_callback(SerialRxCallback callback, void* user_arg) override;

 private:
  ::SerialPort* port_;
};

class CppSerialPortBridge {
 public:
  explicit CppSerialPortBridge(ISerialPort& port) noexcept;

  CppSerialPortBridge(const CppSerialPortBridge&) = delete;
  CppSerialPortBridge& operator=(const CppSerialPortBridge&) = delete;

  ::SerialPort* c_port() noexcept;
  const ::SerialPort* c_port() const noexcept;

 private:
  static bool open_thunk(void* self);
  static void close_thunk(void* self);
  static bool write_thunk(void* self, const ::SerialMessage* msg);
  static bool read_thunk(void* self, ::SerialMessage* msg);
  static void start_read_thunk(void* self);
  static void stop_read_thunk(void* self);
  static void set_rx_callback_thunk(void* self, ::SerialRxCallback callback,
                                    void* user_arg);
  static void destroy_thunk(void* self);

  ISerialPort* port_;
  ::SerialPort c_port_;
};

}  // namespace serial
}  // namespace omuraisu

#endif  // OMURAISU_CPP_SERIAL_SERIAL_INTERFACE_HPP_
