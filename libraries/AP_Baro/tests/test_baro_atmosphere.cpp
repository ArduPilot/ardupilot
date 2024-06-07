/*
  test atmospheric model
  to build, use:
    ./waf configure --board sitl --debug
    ./waf --target tests/test_baro_atmosphere
 */
#include <AP_gtest.h>

#include <AP_Baro/AP_Baro.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(AP_Baro, get_air_density_for_alt_amsl)
{
    float accuracy = 1E-6;

    EXPECT_NEAR(AP_Baro::get_air_density_for_alt_amsl(-4000), 1.7697266, accuracy);
    EXPECT_NEAR(AP_Baro::get_air_density_for_alt_amsl(6000), 0.66011156, accuracy);
    EXPECT_NEAR(AP_Baro::get_air_density_for_alt_amsl(17500), 131.5688E-3, accuracy);
    EXPECT_NEAR(AP_Baro::get_air_density_for_alt_amsl(25000), 40.08393E-3, accuracy);
    EXPECT_NEAR(AP_Baro::get_air_density_for_alt_amsl(40000), 3.99567824728293E-3, accuracy);
    EXPECT_NEAR(AP_Baro::get_air_density_for_alt_amsl(49000), 1.16276961471707E-3, accuracy);
    EXPECT_NEAR(AP_Baro::get_air_density_for_alt_amsl(65000), 163.210100967015E-6, accuracy);
    EXPECT_NEAR(AP_Baro::get_air_density_for_alt_amsl(78000), 25.2385188936996E-6, accuracy);
}

TEST(AP_Baro, get_altitude_from_pressure)
{
    AP_Baro baro;
    float accuracy = 1.5E-1;

    EXPECT_NEAR(baro.get_altitude_from_pressure(159598.2), -4000, accuracy);
    EXPECT_NEAR(baro.get_altitude_from_pressure(47217.6), 6000, accuracy);
    EXPECT_NEAR(baro.get_altitude_from_pressure(8182.3), 17500, accuracy);
    EXPECT_NEAR(baro.get_altitude_from_pressure(2549.2), 25000, accuracy);
    EXPECT_NEAR(baro.get_altitude_from_pressure(287.1441), 40000, accuracy);
    EXPECT_NEAR(baro.get_altitude_from_pressure(90.3365), 49000, accuracy);
    EXPECT_NEAR(baro.get_altitude_from_pressure(10.9297), 65000, accuracy);
    EXPECT_NEAR(baro.get_altitude_from_pressure(1.4674), 78000, accuracy);
}

TEST(AP_Baro, get_temperature_from_altitude)
{
    AP_Baro baro;

    EXPECT_FLOAT_EQ(baro.get_temperature_from_altitude(-4000), 314.166370821781);
    EXPECT_FLOAT_EQ(baro.get_temperature_from_altitude(6000), 249.186776458540);
    EXPECT_FLOAT_EQ(baro.get_temperature_from_altitude(17500), 216.650000000000);
    EXPECT_FLOAT_EQ(baro.get_temperature_from_altitude(25000), 221.552064726284);
    EXPECT_FLOAT_EQ(baro.get_temperature_from_altitude(40000), 250.349646102421);
    EXPECT_FLOAT_EQ(baro.get_temperature_from_altitude(49000), 270.650000000000);
    EXPECT_FLOAT_EQ(baro.get_temperature_from_altitude(65000), 233.292172386848);
    EXPECT_FLOAT_EQ(baro.get_temperature_from_altitude(78000), 202.540977853740);
}

TEST(AP_Baro, geopotential_alt_to_geometric)
{
    EXPECT_FLOAT_EQ(AP_Baro::geopotential_alt_to_geometric(-4002.51858796625), -4000);
    EXPECT_FLOAT_EQ(AP_Baro::geopotential_alt_to_geometric(5994.34208330151), 6000.0);
    EXPECT_FLOAT_EQ(AP_Baro::geopotential_alt_to_geometric(17451.9552525734), 17500);
    EXPECT_FLOAT_EQ(AP_Baro::geopotential_alt_to_geometric(24902.0647262842), 25000);
    EXPECT_FLOAT_EQ(AP_Baro::geopotential_alt_to_geometric(39749.8736080075), 40000);
    EXPECT_FLOAT_EQ(AP_Baro::geopotential_alt_to_geometric(48625.1814380981), 49000);
    EXPECT_FLOAT_EQ(AP_Baro::geopotential_alt_to_geometric(64342.0812904114), 65000);
    EXPECT_FLOAT_EQ(AP_Baro::geopotential_alt_to_geometric(77054.5110731299), 78000);
}

TEST(AP_Baro, geometric_alt_to_geopotential)
{
    EXPECT_FLOAT_EQ(AP_Baro::geometric_alt_to_geopotential(-4000), -4002.51858796625);
    EXPECT_FLOAT_EQ(AP_Baro::geometric_alt_to_geopotential(6000), 5994.34208330151);
    EXPECT_FLOAT_EQ(AP_Baro::geometric_alt_to_geopotential(17500), 17451.9552525734);
    EXPECT_FLOAT_EQ(AP_Baro::geometric_alt_to_geopotential(25000), 24902.0647262842);
    EXPECT_FLOAT_EQ(AP_Baro::geometric_alt_to_geopotential(40000), 39749.8736080075);
    EXPECT_FLOAT_EQ(AP_Baro::geometric_alt_to_geopotential(49000), 48625.1814380981);
    EXPECT_FLOAT_EQ(AP_Baro::geometric_alt_to_geopotential(65000), 64342.0812904114);
    EXPECT_FLOAT_EQ(AP_Baro::geometric_alt_to_geopotential(78000), 77054.5110731299);
}

TEST(AP_Baro, get_pressure_temperature_for_alt_amsl)
{
    float accuracy = 1E-6;

    // layer 0 -4000 m
    float pressure, temperature_K;
    AP_Baro::get_pressure_temperature_for_alt_amsl(-4000, pressure, temperature_K);
    // EXPECT_FLOAT_EQ(pressure, 159598.164433755);    // Matlab
    EXPECT_FLOAT_EQ(pressure, 159598.09375);
    EXPECT_FLOAT_EQ(temperature_K, 314.166370821781);

    // layer 0: 6000 m
    AP_Baro::get_pressure_temperature_for_alt_amsl(6000, pressure, temperature_K);
    EXPECT_FLOAT_EQ(pressure, 47217.6489854302);
    EXPECT_FLOAT_EQ(temperature_K, 249.186776458540);

    // layer 1: 17500 m
    AP_Baro::get_pressure_temperature_for_alt_amsl(17500, pressure, temperature_K);
    EXPECT_FLOAT_EQ(pressure, 8182.27857345022);
    EXPECT_FLOAT_EQ(temperature_K, 216.650000000000);

    // layer 2: 25000m
    AP_Baro::get_pressure_temperature_for_alt_amsl(25000, pressure, temperature_K);
    // EXPECT_FLOAT_EQ(pressure, 2549.22361148236);    // Matlab
    EXPECT_FLOAT_EQ(pressure, 2549.2251);
    EXPECT_FLOAT_EQ(temperature_K, 221.552064726284);

    // layer 3: 40000m
    AP_Baro::get_pressure_temperature_for_alt_amsl(40000, pressure, temperature_K);
    EXPECT_FLOAT_EQ(pressure, 287.144059695555);
    EXPECT_FLOAT_EQ(temperature_K, 250.349646102421);

    // layer 4: 49000m
    AP_Baro::get_pressure_temperature_for_alt_amsl(49000, pressure, temperature_K);
    EXPECT_FLOAT_EQ(pressure, 90.3365441635632);
    EXPECT_FLOAT_EQ(temperature_K, 270.650000000000);

    // layer 5: 65000m
    AP_Baro::get_pressure_temperature_for_alt_amsl(65000, pressure, temperature_K);
    // EXPECT_FLOAT_EQ(pressure, 10.9297197424037);    // Matlab
    EXPECT_FLOAT_EQ(pressure, 10.929715);
    EXPECT_FLOAT_EQ(temperature_K, 233.292172386848);

    // layer 6: 78000m
    AP_Baro::get_pressure_temperature_for_alt_amsl(78000, pressure, temperature_K);
    // EXPECT_FLOAT_EQ(pressure, 1.46736727632119);    // matlab
    EXPECT_NEAR(pressure, 1.4673687, accuracy);
    EXPECT_FLOAT_EQ(temperature_K, 202.540977853740);
}

AP_GTEST_MAIN()
