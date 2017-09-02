#include "Copter.h"

#include "version.h"

const AP_FWVersion Copter::fwver = {
  major: FW_MAJOR,
  minor: FW_MINOR,
  patch: FW_PATCH,
  fw_type: FW_TYPE,
#ifndef GIT_VERSION
  fw_string: THISFIRMWARE
#else
  fw_string: THISFIRMWARE " (" GIT_VERSION ")"
#endif
};

