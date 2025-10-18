#pragma once

#ifndef AP_ADELPHILINKER_ENABLED
#define AP_ADELPHILINKER_ENABLED 1
#endif

#ifdef AP_ADELPHILINKER_ENABLED

// #include <AP_Filesystem/AP_Filesystem.h>
// #include <AP_AHRS/AP_AHRS.h>
// #include <AP_HAL/AP_HAL.h>
// #include <GCS_MAVLink/GCS.h>
// #include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>

#define ADELPHI_CUSTOM_PLANE

enum class STATUS : uint32_t
{
  UNINITIALIZED = 0,
  ATTACHED = 1,
  DEPLOYED = 2,
  LANDED = 3,
};

class AdelphiLinker
{

public:
  static const struct AP_Param::GroupInfo var_info[];

  AdelphiLinker();

  static AdelphiLinker *get_singleton(void) { return _singleton; }

  void set_status(STATUS stat) { this->status = stat; }
  STATUS get_status() { return this->status; }
  const char *status_to_string(STATUS status);
  void init();

  AP_Int8 show_status;

private:
  CLASS_NO_COPY(AdelphiLinker);

  static AdelphiLinker *_singleton;
  STATUS status = STATUS::UNINITIALIZED;
};

namespace AP
{
  AdelphiLinker &adelphi();
};

#endif