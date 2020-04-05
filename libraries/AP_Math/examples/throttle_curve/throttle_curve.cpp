//
// Generate plots for the AP_Math throttle curve code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SITL/SITL.h>

void test_throttle_curve(float alpha);
void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

void print_line(std::vector<double> &vec, ofstream &f);
float copter_throttle_curve(float thr_mid, float throttle_in);

// This code must be copied from Copter's Mode::get_pilot_desired_throttle() since
// there is no method analogous to AP_Math:throttle_curve in copter
float copter_throttle_curve(float thr_mid, float throttle_in)
{
    const float expo = constrain_float(-(thr_mid-0.5f)/0.375f, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

void print_line(vector<double> &vec, ofstream &f)
{
    for (double v : vec) {
        f << v << ",";
    }
    f << "\n";
}

void test_throttle_curve(float alpha)
{
    ofstream csv_out;

    const int ns = 101;
    const int nCurves = 4;
    float out_mid[nCurves] = {0.2, 0.35, 0.5, 0.7};
    vector<double> x(ns,0);
    vector<vector<double>> plane_curve(4, vector<double>(ns,0));
    vector<vector<double>> copter_curve(4, vector<double>(ns,0));
    
    for (uint32_t j=0; j<nCurves; j++) {
        for (uint32_t i=0; i<ns; i++) {
            x[i] = .01 * i;
            plane_curve[j].at(i) = throttle_curve(out_mid[j], alpha, x[i]);
            copter_curve[j].at(i) = copter_throttle_curve(out_mid[j], x[i]);
        }
    }

    // write plot data to csv file
    csv_out.open("throttle_curve_data.csv");
    // value of alpha
    csv_out << alpha << ",\n";
    // values for out_mid
    for (double v : out_mid) {
        csv_out << v << ",";
    }
    csv_out << "\n";
    // x axis
    print_line(x, csv_out);
    // throttle curves
    for (uint32_t j=0; j<nCurves; j++) {
        print_line(plane_curve[j], csv_out);
        print_line(copter_curve[j], csv_out);
    }
    csv_out.close();

    int status = system("python ./libraries/AP_Math/examples/throttle_curve/throttle_curve.py");
    status = system("unlink throttle_curve_data.csv");
    status = system("mv throttle_curves.png ./libraries/AP_Math/examples/throttle_curve");
    
    exit(status);
}
#else // not HAL_BOARD_SITL
void test_throttle_curve(float alpha) {}
#endif // HAL_BOARD_SITL

/*
 *  throttle_curve tests
 */
void setup(void)
{
    printf("generate throttle_curve plot data\n\n");
    printf("close display window to quit\n");
    
    test_throttle_curve(0.2);
}

void loop(void) {}

AP_HAL_MAIN();
