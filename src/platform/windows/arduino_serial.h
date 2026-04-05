/**
 * @file src/platform/windows/arduino_serial.h
 * @brief Arduino Leonardo serial communication for HID input injection.
 *
 * Replaces SendInput() with Arduino USB HID for anti-cheat safe input.
 * Protocol: text commands over serial at 115200 baud.
 */
#pragma once

#include <Windows.h>
#include <string>
#include <mutex>

namespace arduino {

  class Serial {
  public:
    Serial() = default;
    ~Serial();

    /**
     * @brief Auto-detect and connect to Arduino Leonardo.
     * @return true if connected successfully.
     */
    bool connect();

    /**
     * @brief Disconnect from Arduino.
     */
    void disconnect();

    /**
     * @brief Check if connected.
     */
    bool is_connected() const { return _connected; }

    // Mouse commands
    void mouse_move_relative(int dx, int dy);
    void mouse_move_absolute(int x, int y, int screen_w, int screen_h);
    void mouse_button(int button, bool release);
    void mouse_scroll(int delta);

    // Keyboard commands
    void key_press(uint16_t keycode, bool release, bool is_scancode = true);
    void key_string(const std::string &text);
    void release_all();

  private:
    void send(const std::string &cmd);
    std::string auto_detect_port();

    HANDLE _handle = INVALID_HANDLE_VALUE;
    bool _connected = false;
    std::mutex _lock;
  };

  /**
   * @brief Get the global Arduino serial instance.
   */
  Serial &instance();

  /**
   * @brief Check if Arduino input mode is enabled.
   */
  bool is_enabled();

}  // namespace arduino
