#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/select/select.h"
#include "toshiba_climate_mode.h"
#include "esphome/core/component.h"

namespace esphome {
namespace toshiba_suzumi {

static const char *const TAG = "ToshibaClimateUart";
static const uint8_t MAX_TEMP = 30;
static const uint8_t MIN_TEMP_STANDARD = 17;
static const uint8_t SPECIAL_TEMP_OFFSET = 16;
static const uint8_t SPECIAL_MODE_EIGHT_DEG_MIN_TEMP = 5;
static const uint8_t SPECIAL_MODE_EIGHT_DEG_MAX_TEMP = 13;
static const uint8_t SPECIAL_MODE_EIGHT_DEG_DEF_TEMP = 8;
static const uint8_t NORMAL_MODE_DEF_TEMP = 20;

static const std::vector<uint8_t> HANDSHAKE[6] = {
    {2, 255, 255, 0, 0, 0, 0, 2},       {2, 255, 255, 1, 0, 0, 1, 2, 254}, {2, 0, 0, 0, 0, 0, 2, 2, 2, 250},
    {2, 0, 1, 129, 1, 0, 2, 0, 0, 123}, {2, 0, 1, 2, 0, 0, 2, 0, 0, 254},  {2, 0, 2, 0, 0, 0, 0, 254},
};

static const std::vector<uint8_t> AFTER_HANDSHAKE[2] = {
    {2, 0, 2, 1, 0, 0, 2, 0, 0, 251},
    {2, 0, 2, 2, 0, 0, 2, 0, 0, 250},
};

struct ToshibaCommand {
  ToshibaCommandType cmd;
  std::vector<uint8_t> payload;
  int delay;
};

class ToshibaClimateUart : public PollingComponent, public climate::Climate, public uart::UARTDevice {
 public:
  void set_external_sensor(esphome::sensor::Sensor *sensor) { external_sensor_ = sensor; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void update() override;
  void scan();
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_outdoor_temp_sensor(sensor::Sensor *outdoor_temp_sensor) { outdoor_temp_sensor_ = outdoor_temp_sensor; }
  void set_pwr_select(select::Select *pws_select) { pwr_select_ = pws_select; }
  void set_horizontal_swing(bool enabled) { horizontal_swing_ = enabled; }
  void disable_wifi_led(bool disabled) { wifi_led_disabled_ = disabled; }
  void set_special_mode_select(select::Select *special_mode_select) { special_mode_select_ = special_mode_select; }
  void set_min_temp(uint8_t min_temp) { min_temp_ = min_temp; }
  void set_fan_speed_delay(uint32_t delay) { fan_speed_delay_ = delay; }  // <-- FAN SPEED DELAY SETTER

 protected:
  esphome::sensor::Sensor *external_sensor_{nullptr};
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  // --- FAN SPEED DELAY MEMBERS ---
  uint32_t fan_speed_delay_ = 30;     // Delay in seconds, default 30
  uint32_t reached_temp_time_ = 0;    // Timestamp when target temp reached

 private:
  std::vector<uint8_t> rx_message_;
  std::vector<ToshibaCommand> command_queue_;
  uint32_t last_command_timestamp_ = 0;
  uint32_t last_rx_char_timestamp_ = 0;
  STATE power_state_ = STATE::OFF;
  optional<SPECIAL_MODE> special_mode_ = SPECIAL_MODE::STANDARD;
  select::Select *pwr_select_ = nullptr;
  sensor::Sensor *outdoor_temp_sensor_ = nullptr;
  bool horizontal_swing_ = false;
  uint8_t min_temp_ = 17;
  bool wifi_led_disabled_ = false;
  select::Select *special_mode_select_ = nullptr;

  void enqueue_command_(const ToshibaCommand &command);
  void send_to_uart(const ToshibaCommand command);
  void start_handshake();
  void parseResponse(std::vector<uint8_t> rawData);
  void requestData(ToshibaCommandType cmd);
  void process_command_queue_();
  void sendCmd(ToshibaCommandType cmd, uint8_t value);
  void getInitData();
  void handle_rx_byte_(uint8_t c);
  bool validate_message_();
  void on_set_pwr_level(const std::string &value);
  void on_set_special_mode(const std::string &value);

  friend class ToshibaPwrModeSelect;
  friend class ToshibaSpecialModeSelect;
};

class ToshibaPwrModeSelect : public select::Select, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(const std::string &value) override;
};

class ToshibaSpecialModeSelect : public select::Select, public esphome::Parented<ToshibaClimateUart> {
 protected:
  virtual void control(const std::string &value) override;
};

}  // namespace toshiba_suzumi
}  // namespace esphome
