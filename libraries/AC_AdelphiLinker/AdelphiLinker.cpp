#include "AdelphiLinker.h"

AdelphiLinker *AdelphiLinker::_singleton;

AdelphiLinker::AdelphiLinker()
{
  if (_singleton != nullptr)
  {
    AP_HAL::panic("AdelphiLinker must be singleton");
  }

  _singleton = this;
}

void AdelphiLinker::init()
{
  // do nothing
}

const char *AdelphiLinker::status_to_string(STATUS stat)
{
  switch (stat)
  {
  case STATUS::UNINITIALIZED:
    return "UNINITIALIZED";
  case STATUS::ATTACHED:
    return "ATTACHED";
  case STATUS::DEPLOYED:
    return "DEPLOYED";
  case STATUS::LANDED:
    return "LANDED";
  default:
    return "UNKNOWN";
  }
}

namespace AP
{
  AdelphiLinker &adelphi()
  {
    return *AdelphiLinker::get_singleton();
  }
}