#include <AP_gbenchmark.h>
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP

#include <AP_HAL_Linux/VideoIn.h>

static void BM_Crop8bpp(benchmark::State& state)
{
    uint8_t *buffer, *new_buffer;
    uint32_t width = 640;
    uint32_t height = 480;
    uint32_t left = width / 2 - state.range_x() / 2;
    uint32_t top = height / 2 - state.range_y() / 2;

    buffer = (uint8_t *)malloc(width * height);
    if (!buffer) {
        fprintf(stderr, "error: couldn't malloc buffer\n");
        return;
    }

    new_buffer = (uint8_t *)malloc(state.range_x() * state.range_y());
    if (!new_buffer) {
        fprintf(stderr, "error: couldn't malloc new_buffer\n");
        free(buffer);
        return;
    }

    while (state.KeepRunning()) {
        Linux::VideoIn::crop_8bpp(buffer, new_buffer, width,
            left, state.range_x(), top, state.range_y());
    }

    free(buffer);
    free(new_buffer);
}

BENCHMARK(BM_Crop8bpp)->ArgPair(64, 64)->ArgPair(240, 240)->ArgPair(640, 480);

static void BM_YuyvToGrey(benchmark::State& state)
{
    uint8_t *buffer, *new_buffer;

    buffer = (uint8_t *)malloc(state.range_x());
    if (!buffer) {
        fprintf(stderr, "error: couldn't malloc buffer\n");
        return;
    }

    new_buffer = (uint8_t *)malloc(state.range_x() / 2);
    if (!new_buffer) {
        fprintf(stderr, "error: couldn't malloc new_buffer\n");
        free(buffer);
        return;
    }

    while (state.KeepRunning()) {
        Linux::VideoIn::yuyv_to_grey(buffer, state.range_x(), new_buffer);
    }

    free(buffer);
    free(new_buffer);
}

BENCHMARK(BM_YuyvToGrey)->Arg(64 * 64)->Arg(320 * 240)->Arg(640 * 480);
#endif

BENCHMARK_MAIN()
