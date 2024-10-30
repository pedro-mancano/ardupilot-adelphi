#pragma once

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AC_AdelphiLinker/AdelphiLinker.h>
#include <AP_HAL/I2CDevice.h>

#define ADELPHI_LOG_FILE_NAME "Adelphi"
#define ADELPHI_LOG_FILE_EXT ".csv"

constexpr float RAD_TO_DEGf = 180.0f / M_PI;

struct DataFromPlanador
{
  uint8_t id;
  uint8_t should_prepare;
  uint8_t status;
  uint8_t ardupilot_response;
  uint8_t data[24] = {0};

  // Remove this later - ADELPHI_REMOVE
  uint32_t channel_aileron;
  uint32_t channel_rudder;
  uint32_t channel_elevator;
  uint8_t checksum;
};

static_assert(sizeof(DataFromPlanador) == 44, "DataToPlanador size is not 128 bytes");

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

  STATUS status = STATUS::UNINITIALIZED;
  bool has_armed = false;
  bool prepared = false;
  bool should_write_to_esp32 = false;

  // ESP32
  AP_HAL::OwnPtr<AP_HAL::I2CDevice> esp32_device;
  uint32_t esp32_last_read_t = 0;
  DataFromPlanador esp32_data = {};

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

  // rc out
  void rcout_esp32_timer();

  // Hooks
  void on_land();
};

uint8_t calcChecksum(uint8_t *buffer, uint8_t len);
