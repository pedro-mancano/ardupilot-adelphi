#pragma once

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AC_AdelphiLinker/AdelphiLinker.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

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

struct DataFromPlanador
{
  uint8_t id = 0;
  uint32_t in_release_condition = 0;
  uint8_t should_alert_plane_of_release = 0;
  uint8_t ardupilot_release_confirmation = 0;
  uint8_t data[24] = {0};
  uint8_t checksum;
};

static_assert(sizeof(DataFromPlanador) == 36, "DataToPlanador size is not 128 bytes");

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

  // Mission
  const char *header = "Tempo\tXGPS\tYGPS\tZGPS\tELEV\tAIL\tRUD\tTHETA\tPHI\tPSI\tStatus\tAOA\tAOS\n";

  int waiting_gps_fix = 0;
  bool has_fixed_once = false;

  double base_time = -1;

  Location home;
  float home_alt;

  bool has_armed = false;
  bool prepared = false;
  long prepared_time = 0;
  bool should_write_to_esp32 = false;

  // ESP32
  AP_HAL::OwnPtr<AP_HAL::I2CDevice> esp32_device;
  uint32_t esp32_last_read_t = 0;
  DataFromPlanador esp32_data = {};
  DataFromPlanador esp32_data_temp = {};

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

  // i2c communication with esp32
  bool probe_bus(uint8_t bus, uint8_t address);
  void esp32_timer();
  bool esp32_read();

  // Hooks
  void on_land();
};

uint8_t calcChecksum(uint8_t *buffer, uint8_t len);
