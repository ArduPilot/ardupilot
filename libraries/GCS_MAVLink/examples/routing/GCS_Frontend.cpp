#include "GCS_Frontend.h"

class Parameters
{
};

Parameters g;

DataFlash_Class dataflash{"Example DataFlash"};
GCS_Frontend_RoutingExample _frontend{dataflash, g};

GCS_Frontend& GCS_Frontend_Static::get_Frontend() {
    return _frontend;
}
