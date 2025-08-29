/* sbgCommonLib headers */
#include <sbgCommon.h>

/* Local headers */
#include "sbgInterface.h"

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

void sbgInterfaceZeroInit(SbgInterface *pInterface)
{
    assert(pInterface);

    memset(pInterface, 0x00, sizeof(*pInterface));
}
