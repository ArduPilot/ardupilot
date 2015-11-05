#ifdef HAVE_LIBSYSTEMD

#include <systemd/sd-bus.h>

extern int dbus_list_perf_trampoline(sd_bus_message *m, void *data, sd_bus_error *ret_error);
extern int dbus_get_perf_trampoline(sd_bus_message *m, void *data, sd_bus_error *ret_error);

const sd_bus_vtable _perf_vtable[] = {
    SD_BUS_VTABLE_START(0),
    SD_BUS_METHOD("List", NULL, "a(ss)", dbus_list_perf_trampoline, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("Get", "as", "a(ssa{st})", dbus_get_perf_trampoline, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_VTABLE_END
};

#endif
