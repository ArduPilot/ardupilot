#pragma once

/*
  class for an IPV4 address as a parameter
 */
class AP_Networking_IPV4
{
public:
    AP_Networking_IPV4(const char *default_addr);
    AP_Int16 addr[4];

    // return address as a uint32_t
    uint32_t get_uint32(void) const;

    // set address from a uint32_t
    void set_uint32(uint32_t addr);

    // set default address from a uint32
    void set_default_uint32(uint32_t addr);

    static const struct AP_Param::GroupInfo var_info[];
};

/*
  class for an ethernet MAC address as a parameter
 */
class AP_Networking_MAC
{
public:
    AP_Networking_MAC(const char *default_addr);
    AP_Int16 addr[6];
    void get_address(uint8_t addr[6]) const;
    void set_default_address_byte(uint8_t idx, uint8_t b);
    static const struct AP_Param::GroupInfo var_info[];
};

