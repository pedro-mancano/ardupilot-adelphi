#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AC_AdelphiLinker/AdelphiLinker.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <vector>
#include <cmath>
#include <AP_Param/AP_Param.h>

#include <stdio.h>

constexpr float RAD_TO_DEGf = 180.0f / M_PI;

const double EARTH_RADIUS = 6371000.0;

struct Vector2D
{
  double x;
  double y;

  // Vector magnitude
  double magnitude() const
  {
    return sqrt(x * x + y * y);
  }

  // Normalize the vector
  Vector2D normalized() const
  {
    double mag = magnitude();
    if (mag <= 1.0e-12) {
      return {0.0, 0.0};
    }
    return {x / mag, y / mag};
  }

  // Operator overloading for vector arithmetic
  Vector2D operator+(const Vector2D &other) const
  {
    return {x + other.x, y + other.y};
  }

  Vector2D operator-(const Vector2D &other) const
  {
    return {x - other.x, y - other.y};
  }

  Vector2D operator*(double scalar) const
  {
    return {x * scalar, y * scalar};
  }

  Vector2D operator/(double scalar) const
  {
    return {x / scalar, y / scalar};
  }
};

enum class PlanadorInterfaceFields : uint32_t
{
  YES = 0x696969,
  NO = 0x006900,
};

struct __attribute__((packed)) PlanadorInterfacePacket
{
  uint16_t header = 0xAA55;
  uint8_t id = 0x69;
  PlanadorInterfaceFields pilot_called_release = PlanadorInterfaceFields::NO;
  PlanadorInterfaceFields ardupilot_release_confirmation = PlanadorInterfaceFields::NO;
  uint32_t ardupilot_millis = 0;
  uint8_t flight_mode = 0;          // Mode number (0=MANUAL, 2=STABILIZE, 5=FBWA, 10=AUTO)
  uint8_t adelphi_status = 0;       // 0=UNINIT, 1=ATTACHED, 2=DEPLOYED, 3=LANDED
  uint8_t is_armed = 0;             // 1=ARMED, 0=DISARMED
  uint8_t gps_fix = 0;              // GPS Status enum
  uint8_t satellites = 0;           // Sat count
  int32_t altitude_cm = 0;          // Relative altitude in cm
  uint16_t battery_voltage_mv = 0;  // Battery voltage in millivolts
  uint32_t checksum = 0;
};

static_assert(sizeof(PlanadorInterfacePacket) == 30, "PlanadorInterfacePacket size is not 30 bytes");


Vector2D latLonToCartesian(double lat, double lon, double lat_ref, double lon_ref);
Vector2D cartesianToLatLon(double x, double y, double lat_ref, double lon_ref);
Vector2D headingToVector(double heading);
std::vector<Vector2D> calculate(double target_lat, double target_lon, double current_lat, double current_lon, double initial_heading, double speed, double steer_factor, double max_bank, double max_dist, int max_iterations = 1000);
Vector2D findApproachPoint(const Vector2D &target_point, const std::vector<Vector2D> &points, double approach_distance_from_target);

class Adelphi
{
private:
  bool emergency_active = false;
  uint8_t emergency_msg_count = 0;

  bool hasLanded = false;
  
  int waiting_gps_fix = 0;
  bool has_fixed_once = false;

  Location home;
  float home_alt;

  bool prepared_to_release = false;
  long prepared_to_release_time = 0;

  int should_write_to_release_esp32 = 0;

  // UART communication with Release ESP32
  AP_HAL::UARTDriver *release_esp32_uart = nullptr;
  uint32_t release_esp32_last_read_t = 0;
  PlanadorInterfacePacket release_esp32_data;
  PlanadorInterfacePacket release_esp32_data_temp = {};

public:
  static const struct AP_Param::GroupInfo var_info[];

  AP_Float glide_slope; // ADELPHI_GLIDES: Rampa de planeio final (L/D no solo)
  AP_Float r_factor;    // ADELPHI_RFACTOR: % da altitude de soltura reservada para rampa final
  AP_Int16 flight_id;   // ADELPHI_FLIGHTID: Identificador unico da sessao de voo para o DAS (0 a 32767)

  Adelphi();
  ~Adelphi();
  void init();
  void update();

  void trigger_emergency();
  void on_emergency_actived();

  // uart communication with release esp32
  bool probe_release_bus();
  void release_esp32_timer();
  bool release_esp32_read();

  // Hooks
  void on_land();
};

uint8_t calcChecksum(uint8_t *buffer, uint8_t len);
