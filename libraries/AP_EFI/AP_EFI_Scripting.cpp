#include "AP_EFI_Scripting.h"

#if AP_EFI_SCRIPTING_ENABLED

// Called from frontend to update with the readings received by handler
void AP_EFI_Scripting::update()
{
    // Nothing to do here
}

// handle EFI message from scripting
bool AP_EFI_Scripting::handle_scripting(const EFI_State &efi_state)
{
    internal_state = efi_state;
    copy_to_frontend();
    return true;
}

#endif // AP_EFI_SCRIPTING_ENABLED
