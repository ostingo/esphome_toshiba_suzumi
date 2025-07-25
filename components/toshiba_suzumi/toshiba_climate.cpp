#include "toshiba_climate.h"
#include "toshiba_climate_mode.h"
#include "esphome/core/log.h"

namespace esphome {
namespace toshiba_suzumi {

using namespace esphome::climate;

// ... (rest of the code is unchanged)

void ToshibaClimateUart::control(const climate::ClimateCall &call) {
  // ... (existing mode and temp control logic remains unchanged)

  if (call.get_fan_mode().has_value()) {
    auto fan_mode = *call.get_fan_mode();
    if (fan_mode == CLIMATE_FAN_AUTO) {
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_AUTO));
      this->requestData(ToshibaCommandType::FAN);
    } else if (fan_mode == CLIMATE_FAN_QUIET) {
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_QUIET));
      this->requestData(ToshibaCommandType::FAN);
    } else if (fan_mode == CLIMATE_FAN_LOW) {
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_LOW));
      this->requestData(ToshibaCommandType::FAN);
    } else if (fan_mode == CLIMATE_FAN_MEDIUM) {
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_MEDIUM));
      this->requestData(ToshibaCommandType::FAN);
    } else if (fan_mode == CLIMATE_FAN_HIGH) {
      this->set_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_HIGH));
      this->requestData(ToshibaCommandType::FAN);
    }
  }

  if (call.get_custom_fan_mode().has_value()) {
    auto fan_mode = *call.get_custom_fan_mode();
    auto payload = StringToFanLevel(fan_mode);
    if (payload.has_value()) {
      this->set_custom_fan_mode_(fan_mode);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(payload.value()));
      this->requestData(ToshibaCommandType::FAN);
    }
  }

  // ... (existing swing_mode and publish_state)
}

void ToshibaClimateUart::update() {
  
  ESP_LOGD(TAG, "Update: cur=%.2f, tgt=%.2f, fan=%d, mode=%d, time=%u",
           this->current_temperature,
           this->target_temperature,
           this->fan_mode,
           this->mode,
           this->reached_temp_time_);
  this->requestData(ToshibaCommandType::ROOM_TEMP);
  if (outdoor_temp_sensor_ != nullptr) {
    this->requestData(ToshibaCommandType::OUTDOOR_TEMP);
  }

  // --- Enhanced Fan speed adjustment logic ---
  constexpr float LOW_FAN_THRESHOLD = 0.75;    // Temperature difference for LOW fan
  constexpr float MED_FAN_THRESHOLD = 1.25;    // Temperature difference for MEDIUM fan
  constexpr float HIGH_FAN_THRESHOLD = 1.75;   // Temperature difference for HIGH fan
  
  // Only act if device is ON, not OFF
  if (this->power_state_ == STATE::ON) {
    // Ensure we have valid temperatures
    if (!isnan(this->current_temperature) && !isnan(this->target_temperature)) {
      float temp_diff = fabs(this->current_temperature - this->target_temperature);
      bool is_heat_or_cool_mode = (this->mode == climate::CLIMATE_MODE_HEAT || 
                                   this->mode == climate::CLIMATE_MODE_COOL);
      
      ESP_LOGD(TAG, "Fan logic: temp_diff=%.2f, is_heat_or_cool=%d, current_fan=%d", 
               temp_diff, is_heat_or_cool_mode, this->fan_mode);

      // 1. HIGH fan when temperature difference is HIGH (> 1.0°C), especially in HEAT or COOL mode

  if (temp_diff > HIGH_FAN_THRESHOLD) {
    if (this->fan_mode != CLIMATE_FAN_HIGH && (is_heat_or_cool_mode || this->fan_mode != CLIMATE_FAN_AUTO)) {
      this->set_fan_mode_(CLIMATE_FAN_HIGH);
      this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_HIGH));
      this->requestData(ToshibaCommandType::FAN);
      this->reached_temp_time_ = 0;
    }
  } else if (temp_diff > LOW_FAN_THRESHOLD && temp_diff <= MED_FAN_THRESHOLD && this->fan_mode == CLIMATE_FAN_HIGH) {
    this->set_fan_mode_(CLIMATE_FAN_MEDIUM);
    this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_MEDIUM));
    this->requestData(ToshibaCommandType::FAN);
    this->reached_temp_time_ = 0;
  } else if (temp_diff <= LOW_FAN_THRESHOLD && this->fan_mode != CLIMATE_FAN_LOW) {
    if (is_heat_or_cool_mode && temp_diff > (LOW_FAN_THRESHOLD * 0.8)) {
      this->reached_temp_time_ = 0;
    } else {
      if (this->reached_temp_time_ == 0) {
        this->reached_temp_time_ = millis();
      } else if (millis() - this->reached_temp_time_ > this->fan_speed_delay_ * 1000) {
        this->set_fan_mode_(CLIMATE_FAN_LOW);
        this->sendCmd(ToshibaCommandType::FAN, static_cast<uint8_t>(FAN::FAN_LOW));
        this->requestData(ToshibaCommandType::FAN);
      }
    }
  }

  // ... (rest unchanged)
}

}  // namespace toshiba_suzumi
}  // namespace esphome

