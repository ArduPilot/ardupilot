#include "SafeRTL_test.h"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

SafeRTL_Path* p;

void setup();
void loop();
void reset();
bool check_path(const std::vector<Vector3f>&);

void setup()
{
    hal.console->printf("SafeRTL performance test\n");
    AP_BoardConfig{}.init();

    p = new SafeRTL_Path(false);
}

void loop()
{
    hal.scheduler->delay(5e3); // 5 seconds
    if (!hal.console->is_initialized()) {
        return;
    }
    uint32_t reference_time, run_time;
    bool correct;

    hal.console->printf("--------------------\n");

    // test append_if_far_enough()
    reset();
    correct = check_path(test_path_after_adding);
    hal.console->printf("append: %s\n", correct ? "success" : "fail");

    // test rdp()
    reference_time = AP_HAL::micros();
    for (int i = 0; i < 1000; i++) {
        p->rdp();
    }
    run_time = AP_HAL::micros() - reference_time;
    p->thorough_cleanup();
    correct = check_path(test_path_after_simplifying);
    hal.console->printf("rdp:    %s, %u usec\n", correct ? "success" : "fail", run_time);

    // test detect_loops()
    reset();
    reference_time = AP_HAL::micros();
    for (int i = 0; i < 1000; i++) {
        p->detect_loops();
    }
    run_time = AP_HAL::micros() - reference_time;
    p->thorough_cleanup();
    correct = check_path(test_path_after_pruning);
    hal.console->printf("prune:  %s, %u usec\n", correct ? "success" : "fail", run_time);

    // test both
    reset();
    reference_time = AP_HAL::micros();
    while (!(p->cleanup_ready())) {
        p->rdp();
        p->detect_loops();
    }
    run_time = AP_HAL::micros() - reference_time;
    p->thorough_cleanup();
    correct = check_path(test_path_complete);
    hal.console->printf("both:   %s, %u usec\n", correct ? "success" : "fail", run_time);
}

void reset()
{
    p->reset_path(Vector3f{0.0f, 0.0f, 0.0f});
    for (Vector3f v : test_path_before) {
        p->append_if_far_enough(v);
    }
}

bool check_path(const std::vector<Vector3f>& correct)
{
    for (uint32_t i = 0; i < correct.size(); i++) {
        if (!is_equal(p->get(i)[0],correct[i][0]) || !is_equal(p->get(i)[1],correct[i][1])|| !is_equal(p->get(i)[2],correct[i][2])) {
            return false;
        }
    }
    return true;
}

AP_HAL_MAIN();
