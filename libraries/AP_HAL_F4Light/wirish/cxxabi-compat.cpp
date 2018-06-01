/* We compile with nodefaultlibs, so we need to provide an error
 * handler for an empty pure virtual function */

extern "C" void __cxa_pure_virtual(void) {

    extern void __error(uint32_t num, uint32_t pc, uint32_t lr, uint32_t flag);

    __error(11, 0, 0, 0);
}
