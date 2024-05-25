#include "AdelphiLinker.h"

AdelphiLinker *AdelphiLinker::_singleton;

AdelphiLinker::AdelphiLinker()
{
  AP_Param::setup_object_defaults(this, var_info);

  if (_singleton != nullptr)
  {
    AP_HAL::panic("AP_GPS must be singleton");
  }

  _singleton = this;
}

void AdelphiLinker::init()
{
  // do nothing
}

const AP_Param::GroupInfo AdelphiLinker::var_info[] = {

    // @Param: _UGA
    // @DisplayName: Adelphi UGA
    // @Description: Test parameter for Adelphi
    // @Range: 1 5
    // @Increment: 1
    AP_GROUPINFO("UGA", 1, AdelphiLinker, show_status, 1),

    AP_GROUPEND};

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