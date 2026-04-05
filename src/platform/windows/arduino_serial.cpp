/**
 * @file src/platform/windows/arduino_serial.cpp
 * @brief Arduino Leonardo serial communication implementation.
 */
#include "arduino_serial.h"

#include <SetupAPI.h>
#include <devguid.h>
#include <initguid.h>

#include <sstream>
#include <string>

#include "src/logging.h"

// Link SetupAPI
#pragma comment(lib, "setupapi.lib")

namespace arduino {

  // VID:PID pairs to detect Arduino Leonardo
  static constexpr struct { uint16_t vid; uint16_t pid; } KNOWN_DEVICES[] = {
    {0x2341, 0x8036},  // Arduino Leonardo
    {0x2341, 0x8037},  // Arduino Micro
    {0x2341, 0x0036},  // Arduino Leonardo bootloader
  };

  Serial::~Serial() {
    disconnect();
  }

  bool Serial::connect() {
    if (_connected) return true;

    auto port = auto_detect_port();
    if (port.empty()) {
      BOOST_LOG(warning) << "Arduino not found on any COM port";
      return false;
    }

    // Open COM port
    std::string path = "\\\\.\\" + port;
    _handle = CreateFileA(
      path.c_str(),
      GENERIC_READ | GENERIC_WRITE,
      0,
      nullptr,
      OPEN_EXISTING,
      0,
      nullptr
    );

    if (_handle == INVALID_HANDLE_VALUE) {
      BOOST_LOG(error) << "Failed to open " << port << ": " << GetLastError();
      return false;
    }

    // Configure serial port
    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(_handle, &dcb)) {
      CloseHandle(_handle);
      _handle = INVALID_HANDLE_VALUE;
      return false;
    }

    dcb.BaudRate = 115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;

    if (!SetCommState(_handle, &dcb)) {
      CloseHandle(_handle);
      _handle = INVALID_HANDLE_VALUE;
      return false;
    }

    // Set timeouts
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts(_handle, &timeouts);

    // Wait for Arduino reset (DTR toggle)
    Sleep(2000);

    // Drain any pending data
    PurgeComm(_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    _connected = true;
    BOOST_LOG(info) << "Arduino connected on " << port;
    return true;
  }

  void Serial::disconnect() {
    if (_connected) {
      release_all();
      _connected = false;
    }
    if (_handle != INVALID_HANDLE_VALUE) {
      CloseHandle(_handle);
      _handle = INVALID_HANDLE_VALUE;
    }
  }

  void Serial::mouse_move_relative(int dx, int dy) {
    send("MM," + std::to_string(dx) + "," + std::to_string(dy));
  }

  void Serial::mouse_move_absolute(int x, int y, int screen_w, int screen_h) {
    send("MA," + std::to_string(x) + "," + std::to_string(y) + "," +
         std::to_string(screen_w) + "," + std::to_string(screen_h));
  }

  void Serial::mouse_button(int button, bool release) {
    // Moonlight: 1=left, 2=middle, 3=right, 4=X1, 5=X2
    const char *btn;
    switch (button) {
      case 1: btn = "L"; break;
      case 2: btn = "M"; break;
      case 3: btn = "R"; break;
      default: return;  // X1/X2 not supported by basic Arduino Mouse.h
    }
    send(std::string(release ? "MU," : "MD,") + btn);
  }

  void Serial::mouse_scroll(int delta) {
    // Moonlight sends in WHEEL_DELTA units (120), Arduino expects -5 to 5
    int wheel = delta / 120;
    if (wheel == 0) wheel = (delta > 0) ? 1 : -1;
    if (wheel > 5) wheel = 5;
    if (wheel < -5) wheel = -5;
    send("MW," + std::to_string(wheel));
  }

  void Serial::key_press(uint16_t keycode, bool release) {
    send(std::string(release ? "KU," : "KD,") + std::to_string(keycode));
  }

  void Serial::key_string(const std::string &text) {
    send("KS," + text);
  }

  void Serial::release_all() {
    send("KA");
  }

  void Serial::send(const std::string &cmd) {
    if (!_connected || _handle == INVALID_HANDLE_VALUE) return;

    std::lock_guard<std::mutex> lock(_lock);

    // Drain any pending input
    DWORD bytes_available = 0;
    COMSTAT stat;
    DWORD errors;
    if (ClearCommError(_handle, &errors, &stat)) {
      if (stat.cbInQue > 0) {
        std::vector<char> drain(stat.cbInQue);
        DWORD read;
        ReadFile(_handle, drain.data(), stat.cbInQue, &read, nullptr);
      }
    }

    std::string data = cmd + "\n";
    DWORD written;
    if (!WriteFile(_handle, data.c_str(), (DWORD)data.size(), &written, nullptr)) {
      BOOST_LOG(warning) << "Arduino serial write failed: " << GetLastError();
      _connected = false;
    }
  }

  std::string Serial::auto_detect_port() {
    // Enumerate COM ports via SetupAPI
    HDEVINFO devInfo = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (devInfo == INVALID_HANDLE_VALUE) return "";

    SP_DEVINFO_DATA devData = {};
    devData.cbSize = sizeof(devData);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(devInfo, i, &devData); i++) {
      // Get hardware ID to check VID:PID
      char hwId[256] = {};
      if (!SetupDiGetDeviceRegistryPropertyA(devInfo, &devData, SPDRP_HARDWAREID, nullptr,
                                              (BYTE *)hwId, sizeof(hwId), nullptr)) {
        continue;
      }

      std::string hwIdStr(hwId);
      bool found = false;
      for (const auto &dev : KNOWN_DEVICES) {
        char match[64];
        snprintf(match, sizeof(match), "VID_%04X&PID_%04X", dev.vid, dev.pid);
        if (hwIdStr.find(match) != std::string::npos) {
          found = true;
          break;
        }
      }
      if (!found) continue;

      // Get COM port name from registry
      HKEY key = SetupDiOpenDevRegKey(devInfo, &devData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
      if (key == INVALID_HANDLE_VALUE) continue;

      char portName[32] = {};
      DWORD portNameSize = sizeof(portName);
      DWORD type;
      if (RegQueryValueExA(key, "PortName", nullptr, &type, (BYTE *)portName, &portNameSize) == ERROR_SUCCESS) {
        RegCloseKey(key);
        SetupDiDestroyDeviceInfoList(devInfo);
        BOOST_LOG(info) << "Arduino Leonardo detected: " << portName;
        return std::string(portName);
      }
      RegCloseKey(key);
    }

    SetupDiDestroyDeviceInfoList(devInfo);
    return "";
  }

  // Global singleton
  static Serial _instance;

  Serial &instance() {
    return _instance;
  }

  bool is_enabled() {
    return _instance.is_connected();
  }

}  // namespace arduino
