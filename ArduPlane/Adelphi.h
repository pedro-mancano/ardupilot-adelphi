#pragma once

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>

#define ADELPHI_LOG_FILE_NAME "Adelphi"
#define ADELPHI_LOG_FILE_EXT ".csv"

enum class STATUS : uint32_t
{
  UNINITIALIZED = 0,
  ATTACHED = 1,
  DEPLOYED = 2,
  LANDED = 3,
};

constexpr float RAD_TO_DEGf = 180.0f / M_PI;

class Adelphi
{
private:
  int file;
  ByteBuffer writebuf{0};
  int _write_offset = 0;
  HAL_Semaphore sem;
  int dropped_count = 0;

  const char *header = "Tempo\tXGPS\tYGPS\tZGPS\tELEV\tAIL\tRUD\tTHETA\tPHI\tPSI\tStatus\tAOA\tAOS\n";
  bool has_fixed_once = false;
  double base_time = -1;
  Location home;
  STATUS status = STATUS::UNINITIALIZED;
  float home_alt;
  int waiting_gps_fix = 0;

public:
  Adelphi();
  ~Adelphi();
  void init();
  void update();
  void io_thread();
  void io_timer();
  void writeBlock(const uint8_t *pBuffer, uint16_t size);
  int log_count();
};