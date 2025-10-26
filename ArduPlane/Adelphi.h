#pragma once

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AC_AdelphiLinker/AdelphiLinker.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <vector>

#include <stdio.h>

#define ADELPHI_LOG_FILE_NAME "Adelphi"
#define ADELPHI_LOG_FILE_EXT ".csv"

constexpr float RAD_TO_DEGf = 180.0f / M_PI;

const float EARTH_RADIUS = 6371000.0;

struct Vector2D
{
  float x;
  float y;

  // Vector magnitude
  float magnitude() const
  {
    return safe_sqrt(x * x + y * y);
  }

  // Normalize the vector
  Vector2D normalized() const
  {
    float mag = magnitude();
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

  Vector2D operator*(float scalar) const
  {
    return {x * scalar, y * scalar};
  }

  Vector2D operator/(float scalar) const
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
  uint8_t id = 0;
  PlanadorInterfaceFields pilot_called_release = PlanadorInterfaceFields::NO;
  PlanadorInterfaceFields ardupilot_release_confirmation = PlanadorInterfaceFields::NO;
  uint32_t checksum;
};

static_assert(sizeof(PlanadorInterfacePacket) == 13, "PlanadorInterfacePacket size is not 13 bytes");

enum class EmergencyInterfaceFields : uint32_t
{
  YES = 0x696969,
  NO = 0x006900,
};

struct __attribute__((packed)) EmergencyInterfacePacket
{
  uint8_t id = 0;
  EmergencyInterfaceFields command = EmergencyInterfaceFields::NO;
  uint32_t checksum;
};

static_assert(sizeof(EmergencyInterfacePacket) == 9, "EmergencyInterfacePacket size is not 9 bytes");


Vector2D latLonToCartesian(float lat, float lon, float lat_ref, float lon_ref);
Vector2D cartesianToLatLon(float x, float y, float lat_ref, float lon_ref);
Vector2D headingToVector(float heading);
std::vector<Vector2D> calculate(float target_lat, float target_lon, float current_lat, float current_lon, float initial_heading, float speed, float steer_factor, float max_bank, float max_dist, int max_iterations = 1000);
Vector2D findApproachPoint(const Vector2D &target_point, const std::vector<Vector2D> &points, float approach_distance_from_target);

class Adelphi
{
private:
  // Filesystem
  int file;
  ByteBuffer writebuf{0};
  int _write_offset = 0;
  int dropped_count = 0;
  HAL_Semaphore sem;
  bool hasLanded = false;

  // Mission
  const char *header = "Tempo\tXGPS\tYGPS\tZGPS\tELEV\tAIL\tRUD\tTHETA\tPHI\tPSI\tStatus\tAOA\tAOS\n";

  int waiting_gps_fix = 0;
  bool has_fixed_once = false;

  double base_time = -1;

  Location home;
  float home_alt;

  bool has_armed = false;
  bool prepared_to_release = false;
  long prepared_to_release_time = 0;

  int should_write_to_release_esp32 = 0;

  // I2C communication with Release ESP32
  AP_HAL::OwnPtr<AP_HAL::I2CDevice> release_esp32_device;
  uint32_t release_esp32_last_read_t = 0;
  PlanadorInterfacePacket release_esp32_data = {};
  PlanadorInterfacePacket release_esp32_data_temp = {};

  // I2C communication with Emergency ESP32
  AP_HAL::OwnPtr<AP_HAL::I2CDevice> emergency_esp32_device;
  uint32_t emergency_esp32_last_read_t = 0;
  EmergencyInterfacePacket emergency_esp32_data = {};
  EmergencyInterfacePacket emergency_esp32_data_temp = {};

public:
  Adelphi();
  ~Adelphi();
  void init();
  void update();

  // fs functions
  void io_thread();
  void io_timer();
  int log_count();
  void writeBlock(const uint8_t *pBuffer, uint16_t size);

  // i2c communication with release esp32
  bool probe_release_bus(uint8_t bus, uint8_t address);
  void release_esp32_timer();
  bool release_esp32_read();

  // i2c communication with emergency esp32
  bool probe_emergency_bus(uint8_t bus, uint8_t address);
  void emergency_esp32_timer();
  bool emergency_esp32_read();

  // Hooks
  void on_land();
};

uint8_t calcChecksum(uint8_t *buffer, uint8_t len);
