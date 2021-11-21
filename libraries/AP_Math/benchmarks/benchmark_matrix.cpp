#include <AP_gbenchmark.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void BM_MatrixMultiplication(benchmark::State& state)
{
    Matrix3f m1(Vector3f(1.0f, 2.0f, 3.0f),
                Vector3f(4.0f, 5.0f, 6.0f),
                Vector3f(7.0f, 8.0f, 9.0f));
    Matrix3f m2(Vector3f(1.0f, 2.0f, 3.0f),
                Vector3f(4.0f, 5.0f, 6.0f),
                Vector3f(7.0f, 8.0f, 9.0f));

    while (state.KeepRunning()) {
        Matrix3f m3 = m1 * m2;
        gbenchmark_escape(&m3);
    }
}

BENCHMARK(BM_MatrixMultiplication);

BENCHMARK_MAIN();
