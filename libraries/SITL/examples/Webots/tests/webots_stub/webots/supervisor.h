#ifndef WEBOTS_STUB_SUPERVISOR_H
#define WEBOTS_STUB_SUPERVISOR_H
#include "types.h"
WbNodeRef wb_supervisor_node_get_root(void);
WbFieldRef wb_supervisor_node_get_field(WbNodeRef node, const char *name);
int wb_supervisor_node_get_type(WbNodeRef node);
int wb_supervisor_field_get_count(WbFieldRef field);
WbNodeRef wb_supervisor_field_get_mf_node(WbFieldRef field, int index);
const double *wb_supervisor_field_get_sf_vec3f(WbFieldRef field);
const double *wb_supervisor_field_get_sf_rotation(WbFieldRef field);
void wb_supervisor_field_set_sf_vec3f(WbFieldRef field, const double values[3]);
void wb_supervisor_field_set_sf_rotation(WbFieldRef field, const double values[4]);
WbNodeRef wb_supervisor_node_get_self(void);
void wb_supervisor_node_reset_physics(WbNodeRef node);
#define WB_NODE_ROBOT 40
#define WB_NODE_WORLD_INFO 41
#endif
