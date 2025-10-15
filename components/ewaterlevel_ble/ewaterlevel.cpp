#include "ewaterlevel.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32

namespace esphome {
namespace ewaterlevel_ble {

static const char *const TAG = "e-waterlevel_ble";

void EWaterLevel::dump_config() {
  ESP_LOGCONFIG(TAG, "E-Waterlevel-BLE");
  ESP_LOGCONFIG(TAG, "  Address: %s", format_ble_address_pretty(this->address_).c_str());
  ESP_LOGCONFIG(TAG, "  Min value: %.3f", this->min_value_);
  ESP_LOGCONFIG(TAG, "  Max value: %.3f", this->max_value_);
  if (!std::isnan(this->length_) && this->length_ > 0.0f) {
    ESP_LOGCONFIG(TAG, "  Length: %.1fcm", this->length_);
  }
  LOG_SENSOR("  ", "Time", this->time_);
  LOG_SENSOR("  ", "Value", this->value_);
  LOG_SENSOR("  ", "Height", this->height_);
  LOG_SENSOR("  ", "Level", this->level_);
  LOG_SENSOR("  ", "Battery Level", this->battery_level_);
  LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_);
}

/**
 * Parse all incoming BLE payloads to see if it is a matching BLE advertisement.
 * Currently this supports the following products:
 *
 * - E-Trailer E-Waterlevel
 *
 * The following points are used to identify a sensor:
 *
 * - Bluetooth data frame size
 * - Bluetooth data frame header
 */
bool EWaterLevel::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  if (this->address_ != 0) {
    if (device.address_uint64() != this->address_) {
      return false;
    }
    ESP_LOGI(TAG, "Found BLE device: %s (Name: %s)", device.address_str().c_str(), device.get_name().c_str());
  }

   auto mfg_datas = device.get_manufacturer_datas();
   if (mfg_datas.empty()) {
     return false;
   }
   auto mfg_data = mfg_datas[0];

  // UUID als native Bytes holen (z.B. 2 oder 16 Bytes, je nach UUID-Typ)
  auto uuid_bytes = mfg_data.uuid.get_native(); // gibt uint16_t oder std::array<uint8_t, 16> zurück

  std::vector<uint8_t> payload;

  // UUID-Bytes anhängen
  if constexpr (std::is_same<decltype(uuid_bytes), uint16_t>::value) {
    payload.push_back(static_cast<uint8_t>(uuid_bytes & 0xFF));
    payload.push_back(static_cast<uint8_t>((uuid_bytes >> 8) & 0xFF));
  } else {
    payload.insert(payload.end(), uuid_bytes.begin(), uuid_bytes.end());
  }

  // Daten anhängen
  payload.insert(payload.end(), mfg_data.data.begin(), mfg_data.data.end());

  // Zugriff auf das zusammengefügte Payload
  const uint8_t *payload = payload.data();
  size_t len = payload.size();
  ESP_LOGI(TAG, "Manufacturer data size: %u (expected: %u)", len, sizeof(ewaterlevel_data));

  if (len == sizeof(ewaterlevel_data)) {
    const ewaterlevel_data *data = reinterpret_cast<const ewaterlevel_data *>(payload);
    if (!data->validate_header()) {
      ESP_LOGI(TAG, "Header validation failed!");
      return false;
    }

    if (this->address_ == 0) {
      ESP_LOGI(TAG, "E-Waterlevel SENSOR FOUND: %s", device.address_str().c_str());
    }

    ESP_LOGI(TAG, "[%s] Sensor data: %s", device.address_str().c_str(),
             format_hex_pretty(payload, len).c_str());
    ESP_LOGI(TAG, "[%s] HW: V%u.%u SW: V%u.%u, ShortPin: %.1fcm, LongPin: %.1fcm", device.address_str().c_str(),
             data->version_hw_high, data->version_hw_low, data->version_sw_high, data->version_sw_low,
             data->read_short_pin_length(), data->read_long_pin_length());
    ESP_LOGI(TAG, "[%s] State_A: %s, State_B: %s, State_C: %s", device.address_str().c_str(),
             format_hex(&data->state_a, 1).c_str(), format_hex(&data->state_b, 1).c_str(),
             format_hex(&data->state_c, 1).c_str());
    ESP_LOGI(TAG, "[%s] Time: %.2f, Bat: %.3fV, Value: %.3f", device.address_str().c_str(), data->read_counter(),
             data->read_battery_voltage(), data->read_value());
    ESP_LOGI(TAG, "[%s] Waterlevel: %.1fcm, Percentage: %.1f%%", device.address_str().c_str(),
             this->water_height_in_cm_(data),
             clamp_percentage(this->water_height_in_cm_(data) / this->pin_length_(data) * 100.0f));

    if (this->time_ != nullptr) {
      this->time_->publish_state(data->read_counter());
    }

    if (this->value_ != nullptr) {
      this->value_->publish_state(data->read_value());
    }

    if (this->height_ != nullptr) {
      this->height_->publish_state(this->water_height_in_cm_(data));
    }

    if (this->level_ != nullptr) {
      const auto height_in_cm = this->water_height_in_cm_(data);
      const auto pin_length = this->pin_length_(data);
      this->level_->publish_state(clamp_percentage(height_in_cm / pin_length * 100.0f));
    }

    if (this->battery_voltage_ != nullptr) {
      this->battery_voltage_->publish_state(data->read_battery_voltage());
    }

    if (this->battery_level_ != nullptr) {
      const auto battery_volt = data->read_battery_voltage();
      if (std::isnan(battery_volt)) {
        this->battery_level_->publish_state(NAN);
      } else {
        float percent = (battery_volt - 2.2f) / 0.65f * 100.0f;
        this->battery_level_->publish_state(clamp_percentage(percent));
      }
    }

    return true;
  }
  return false;
}

float EWaterLevel::water_height_in_cm_(const ewaterlevel_data *data) {
  if (!data->validate_state_a()) {
    return 0.0f;
  }

  const auto value = data->read_value();
  const auto pin_length = this->pin_length_(data);
  const auto scaling_factor = (this->max_value_ - this->min_value_) / (pin_length - this->min_length_);
  const auto height = (value - this->min_value_) / scaling_factor + this->min_length_;

  if (height < 0.0f) {
    return 0.0f;
  } else if (height > pin_length) {
    return pin_length;
  } else {
    return height;
  }
}

}  // namespace ewaterlevel_ble
}  // namespace esphome

#endif
