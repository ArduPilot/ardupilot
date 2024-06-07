#pragma once

#include "Util.h"

namespace Linux {

enum class LINUX_BOARD_TYPE: int {
	RPI_ZERO_1=0,
    RPI_2_3_ZERO2=1,
	RPI_4=2,
    RPI_5=3,
    ALLWINNWER_H616=100,
    UNKNOWN_BOARD=999
};

class UtilRPI : public Util {
public:
    UtilRPI();

    static UtilRPI *from(AP_HAL::Util *util) {
        return static_cast<UtilRPI*>(util);
    }

    /* return the Raspberry Pi version */
    LINUX_BOARD_TYPE detect_linux_board_type() const;

protected:
    // Called in the constructor once
    void _get_board_type_using_peripheral_base();

private:
    LINUX_BOARD_TYPE _linux_board_version = LINUX_BOARD_TYPE::UNKNOWN_BOARD;
};

}
