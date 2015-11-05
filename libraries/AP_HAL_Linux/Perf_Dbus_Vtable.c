#ifdef HAVE_SYSTEMD

#include <systemd/sd-bus.h>

extern int _dbus_list_perf_callback(sd_bus_message *m, void *data, sd_bus_error *ret_error);
extern int _dbus_get_perf_callback(sd_bus_message *m, void *data, sd_bus_error *ret_error);

const sd_bus_vtable _perf_vtable[] = {
    SD_BUS_VTABLE_START(0),
    SD_BUS_METHOD("List", NULL, "a(ss)", _dbus_list_perf_callback, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("Get", "as", "a(ssa{st})", _dbus_get_perf_callback, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_VTABLE_END
};

#endif
