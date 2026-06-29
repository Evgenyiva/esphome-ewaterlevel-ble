#include "ewaterlevel.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32

namespace esphome {
namespace ewaterlevel_ble {

static const char *const TAG = "ewaterlevel_ble";

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
    ESP_LOGV(TAG, "Found BLE device: %s (Name: %s)", device.address_str().c_str(), device.get_name().c_str());
  }

  auto service_datas = device.get_service_datas();
  for (const auto &service_data : service_datas) {
    ESP_LOGV(TAG, "Service UUID: %s", service_data.uuid.to_string().c_str());
    ESP_LOGV(TAG, "Service Data: %s", format_hex_pretty(service_data.data.data(), service_data.data.size()).c_str());
  }

   auto mfg_datas = device.get_manufacturer_datas();
   if (mfg_datas.empty()) {
     return false;
   }
   auto mfg_data = mfg_datas[0];
   // Optional robustness: pre-filter by company id, e.g.
   //   if (mfg_data.uuid.get_uuid() == 0x5457) { ... }
   // ('WT' = 0x5457). Not enabled because the byte-encoding/byte-order of the
   // company id as exposed here is unverified.
   // TODO: confirm uuid byte-order with hardware before enabling.

  // Access the assembled payload
  const uint8_t *payload = mfg_data.data.data();
  size_t len = mfg_data.data.size();
  ESP_LOGV(TAG, "Manufacturer data size: %u (expected: %u)", len, sizeof(ewaterlevel_data));

  if (len == sizeof(ewaterlevel_data)) {
    const ewaterlevel_data *data = reinterpret_cast<const ewaterlevel_data *>(payload);
    ESP_LOGV(TAG, "[%s] Sensor data: %s", device.address_str().c_str(),
             format_hex_pretty(payload, len).c_str());

    if (!data->validate_header()) {
      ESP_LOGI(TAG, "Header validation failed!");
      return false;
    }

    if (this->address_ == 0) {
      ESP_LOGI(TAG, "E-Waterlevel SENSOR FOUND: %s", device.address_str().c_str());
    }

    // Compute the derived values once and reuse them for both logging and publishing.
    // water_height_in_cm_ and pin_length_ used to be recomputed up to 4x per advert.
    const float height_in_cm = this->water_height_in_cm_(data);
    const float pin_length = this->pin_length_(data);
    const float level_percent = clamp_percentage(height_in_cm / pin_length * 100.0f);

    // Throttle the matched-device calibration INFO lines to at most once per 5s.
    // Find-mode (address_ == 0) keeps logging every advert as-is (temporary setup).
    // Only logging is throttled here — publishing below stays per-advert.
    bool should_log = true;
    if (this->address_ != 0) {
      const uint32_t now = millis();
      if (now - this->last_log_ms_ >= 5000) {
        this->last_log_ms_ = now;
      } else {
        should_log = false;
      }
    }

    if (should_log) {
      ESP_LOGI(TAG, "[%s] HW: V%u.%u SW: V%u.%u, ShortPin: %.1fcm, LongPin: %.1fcm", device.address_str().c_str(),
               data->version_hw_high, data->version_hw_low, data->version_sw_high, data->version_sw_low,
               data->read_short_pin_length(), data->read_long_pin_length());
      ESP_LOGI(TAG, "[%s] Time: %.2f, Bat: %.3fV, Value: %.3f", device.address_str().c_str(), data->read_counter(),
               data->read_battery_voltage(), data->read_value());
      ESP_LOGI(TAG, "[%s] Waterlevel: %.1fcm, Percentage: %.1f%%", device.address_str().c_str(),
               height_in_cm, level_percent);
    }

    if (this->time_ != nullptr) {
      this->time_->publish_state(data->read_counter());
    }

    if (this->value_ != nullptr) {
      this->value_->publish_state(data->read_value());
    }

    if (this->height_ != nullptr) {
      this->height_->publish_state(height_in_cm);
    }

    if (this->level_ != nullptr) {
      this->level_->publish_state(level_percent);
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

  // Guard against degenerate calibration / pin geometry. The config schema already
  // enforces max_value > min_value, but pin_length (which may come from the device
  // at runtime) could equal min_length_ or be NaN, yielding a zero/non-finite
  // scaling_factor. Return 0 (not NaN) to keep the HA graph continuous.
  if (scaling_factor == 0.0f || !std::isfinite(scaling_factor)) {
    return 0.0f;
  }

  const auto height = (value - this->min_value_) / scaling_factor + this->min_length_;

  if (!std::isfinite(height)) {
    return 0.0f;
  }

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
