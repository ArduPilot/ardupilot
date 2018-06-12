#include "AP_OccupancyGrid.h"

void AP_OccupancyGrid::init()
{
    cells = (AP_OccupancyGrid_Cell**)malloc(sizeof(class AP_OccupancyGrid_Cell) * sizex * sizey);
}
