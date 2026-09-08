board_runner_args(jlink "--device=MIMXRT1176xxxxx_M7" "--speed=4000" "--reset-after-load")
board_runner_args(pyocd "--target=mimxrt1170_cm7" "--frequency=4000000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
