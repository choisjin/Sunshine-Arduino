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

  // Windows scancode → Arduino Keyboard.h keycode mapping
  // Scancodes from Sunshine's VK_TO_SCANCODE_MAP (US English layout)
  // Arduino keycodes: ASCII for printable, 0x80+ for special keys
  static uint16_t scancode_to_arduino(uint16_t scan, bool is_scancode) {
    if (!is_scancode) {
      // VK code path — convert common VK codes to Arduino keycodes
      switch (scan) {
        case 0x08: return 0xB2;  // VK_BACK → Backspace
        case 0x09: return 0xB3;  // VK_TAB → Tab
        case 0x0D: return 0xB0;  // VK_RETURN → Enter
        case 0x10: return 0x81;  // VK_SHIFT → ShiftLeft
        case 0x11: return 0x80;  // VK_CONTROL → CtrlLeft
        case 0x12: return 0x82;  // VK_MENU → AltLeft
        case 0x14: return 0xC1;  // VK_CAPITAL → CapsLock
        case 0x1B: return 0xB1;  // VK_ESCAPE → Escape
        case 0x20: return 0x20;  // VK_SPACE → Space
        case 0x21: return 0xD3;  // VK_PRIOR → PageUp
        case 0x22: return 0xD6;  // VK_NEXT → PageDown
        case 0x23: return 0xD5;  // VK_END → End
        case 0x24: return 0xD2;  // VK_HOME → Home
        case 0x25: return 0xD8;  // VK_LEFT → ArrowLeft
        case 0x26: return 0xDA;  // VK_UP → ArrowUp
        case 0x27: return 0xD7;  // VK_RIGHT → ArrowRight
        case 0x28: return 0xD9;  // VK_DOWN → ArrowDown
        case 0x2D: return 0xD1;  // VK_INSERT → Insert
        case 0x2E: return 0xD4;  // VK_DELETE → Delete
        case 0x5B: return 0x83;  // VK_LWIN → MetaLeft
        case 0x5C: return 0x87;  // VK_RWIN → MetaRight
        case 0x70: return 0xC2;  // VK_F1
        case 0x71: return 0xC3;  // VK_F2
        case 0x72: return 0xC4;  // VK_F3
        case 0x73: return 0xC5;  // VK_F4
        case 0x74: return 0xC6;  // VK_F5
        case 0x75: return 0xC7;  // VK_F6
        case 0x76: return 0xC8;  // VK_F7
        case 0x77: return 0xC9;  // VK_F8
        case 0x78: return 0xCA;  // VK_F9
        case 0x79: return 0xCB;  // VK_F10
        case 0x7A: return 0xCC;  // VK_F11
        case 0x7B: return 0xCD;  // VK_F12
        case 0xA0: return 0x81;  // VK_LSHIFT
        case 0xA1: return 0x85;  // VK_RSHIFT
        case 0xA2: return 0x80;  // VK_LCONTROL
        case 0xA3: return 0x84;  // VK_RCONTROL
        case 0xA4: return 0x82;  // VK_LMENU (Alt)
        case 0xA5: return 0x86;  // VK_RMENU (AltGr)
        default:
          // 0-9, A-Z: VK codes match ASCII
          if (scan >= 0x30 && scan <= 0x39) return scan;        // Digits
          if (scan >= 0x41 && scan <= 0x5A) return scan + 0x20; // Letters → lowercase
          return scan;
      }
    }

    // Scancode path — convert PS/2 scancode to Arduino keycode
    switch (scan) {
      // Letters (scancodes 16-50 → ASCII lowercase)
      case 16: return 'q'; case 17: return 'w'; case 18: return 'e';
      case 19: return 'r'; case 20: return 't'; case 21: return 'y';
      case 22: return 'u'; case 23: return 'i'; case 24: return 'o';
      case 25: return 'p'; case 30: return 'a'; case 31: return 's';
      case 32: return 'd'; case 33: return 'f'; case 34: return 'g';
      case 35: return 'h'; case 36: return 'j'; case 37: return 'k';
      case 38: return 'l'; case 44: return 'z'; case 45: return 'x';
      case 46: return 'c'; case 47: return 'v'; case 48: return 'b';
      case 49: return 'n'; case 50: return 'm';

      // Number row
      case 2: return '1'; case 3: return '2'; case 4: return '3';
      case 5: return '4'; case 6: return '5'; case 7: return '6';
      case 8: return '7'; case 9: return '8'; case 10: return '9';
      case 11: return '0';

      // Symbols
      case 12: return '-';  // Minus
      case 13: return '=';  // Equal
      case 26: return '[';  // BracketLeft
      case 27: return ']';  // BracketRight
      case 39: return ';';  // Semicolon
      case 40: return '\''; // Quote
      case 41: return '`';  // Backquote
      case 43: return '\\'; // Backslash
      case 51: return ',';  // Comma
      case 52: return '.';  // Period
      case 53: return '/';  // Slash
      case 57: return ' ';  // Space

      // Special keys
      case 1:  return 0xB1;  // Escape
      case 14: return 0xB2;  // Backspace
      case 15: return 0xB3;  // Tab
      case 28: return 0xB0;  // Enter
      case 58: return 0xC1;  // CapsLock

      // Modifiers
      case 42: return 0x81;  // LShift
      case 54: return 0x85;  // RShift
      case 29: return 0x80;  // LCtrl (also RCtrl with extended flag)
      case 56: return 0x82;  // LAlt (also RAlt with extended flag)

      // Navigation (extended keys)
      case 71: return 0xD2;  // Home
      case 72: return 0xDA;  // Up
      case 73: return 0xD3;  // PageUp
      case 75: return 0xD8;  // Left
      case 77: return 0xD7;  // Right
      case 79: return 0xD5;  // End
      case 80: return 0xD9;  // Down
      case 81: return 0xD6;  // PageDown
      case 82: return 0xD1;  // Insert
      case 83: return 0xD4;  // Delete

      // Function keys
      case 59: return 0xC2;  // F1
      case 60: return 0xC3;  // F2
      case 61: return 0xC4;  // F3
      case 62: return 0xC5;  // F4
      case 63: return 0xC6;  // F5
      case 64: return 0xC7;  // F6
      case 65: return 0xC8;  // F7
      case 66: return 0xC9;  // F8
      case 67: return 0xCA;  // F9
      case 68: return 0xCB;  // F10
      case 87: return 0xCC;  // F11
      case 88: return 0xCD;  // F12

      // Numpad scancodes overlap with navigation (71=Home/Num7, 72=Up/Num8, etc.)
      // Extended key flag distinguishes them, but Arduino Keyboard.h doesn't support numpad.
      // Navigation keys take priority (already mapped above).
      case 74: return 0xDE;  // NumpadSubtract (unique scancode)
      case 78: return 0xDB;  // NumpadAdd (unique scancode)
      case 55: return 0xDD;  // NumpadMultiply
      // case 53 = NumpadDivide shares scancode with '/' (already mapped above)

      default: return scan;  // Pass through unknown
    }
  }

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

  void Serial::key_press(uint16_t keycode, bool release, bool is_scancode) {
    uint16_t arduino_code = scancode_to_arduino(keycode, is_scancode);
    send(std::string(release ? "KU," : "KD,") + std::to_string(arduino_code));
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
