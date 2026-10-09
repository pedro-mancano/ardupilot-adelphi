#pragma once

#ifndef AP_ADELPHILINKER_ENABLED
#define AP_ADELPHILINKER_ENABLED 1
#endif

#ifdef AP_ADELPHILINKER_ENABLED

#include <AP_HAL/AP_HAL.h>

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
  AdelphiLinker();

  static AdelphiLinker *get_singleton(void) { return _singleton; }

  void set_status(STATUS stat) { this->status = stat; }
  STATUS get_status() { return this->status; }
  const char *status_to_string(STATUS status);
  void init();

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