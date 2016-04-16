#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>

#define SQRT_2 1.4142135623730951f


TEST(ZeroTest, zero)
{
  #define TEST_IS_ZERO(_test, _result) { \
    bool bZero = is_zero(_test); \
    EXPECT_EQ(bZero, _result) << "is_zero(): floating point is_zero comparator failure"; \
  }
  
  TEST_IS_ZERO(0.1,      false);
  TEST_IS_ZERO(0.0001,   false);
  TEST_IS_ZERO(0.f,      true);
  TEST_IS_ZERO(FLT_MIN,  true);
  TEST_IS_ZERO(-FLT_MIN, true);
}

TEST(EqualTest, equal)
{
  #define TEST_IS_EQUAL(_test1, _test2, _result) { \
    bool bEqual = is_equal(_test1, _test2); \
    EXPECT_EQ(bEqual, _result) << "is_equal(): floating point is_equal comparator failure"; \
  }
  
  TEST_IS_EQUAL(0.1,     0.10001,           false);
  TEST_IS_EQUAL(0.1,     -0.1001,           false);
  TEST_IS_EQUAL(0.f,     0.0f,              true);
  TEST_IS_EQUAL(1.f,     1.f+FLT_EPSILON,   false);
}

TEST(SquareTest, square)
{
  float sq_1 = sq(1);
  float sq_2 = sq(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
  float sq_3 = sq(0);
  float sq_4 = sq(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
  float sq_5 = sq(2,2); 
  float sq_6 = sq(2,2,2,2);
  
  EXPECT_EQ(sq_1, 1.f);
  EXPECT_EQ(sq_2, 16.f);
  EXPECT_EQ(sq_3, 0.f);
  EXPECT_EQ(sq_4, 0.f);
  EXPECT_EQ(sq_5, 8.f);
  EXPECT_EQ(sq_6, 16.f);
}

TEST(NormTest, norm)
{
  float norm_1 = norm(1);
  float norm_2 = norm(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
  float norm_3 = norm(0);
  float norm_4 = norm(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
  float norm_5 = norm(3,4); // 5
  float norm_6 = norm(4,3,12); //13
  
  EXPECT_EQ(norm_1, 1.f);
  EXPECT_EQ(norm_2, 4.f);
  EXPECT_EQ(norm_3, 0.f);
  EXPECT_EQ(norm_4, 0.f);
  EXPECT_EQ(norm_5, 5.f);
  EXPECT_EQ(norm_6, 13.f);
}

TEST(ConstrainTest, constrain)
{  
  for(int i = 0; i < 1000; i++) {
      if(i < 250) {
          EXPECT_EQ(constrain_float(i, 250, 500), 250);
          EXPECT_EQ(constrain_int16(i, 250, 500), 250);
          EXPECT_EQ(constrain_int32(i, 250, 500), 250);
      }
      else if(i > 500) {
          EXPECT_EQ(constrain_float(i, 250, 500), 500);
          EXPECT_EQ(constrain_int16(i, 250, 500), 500);
          EXPECT_EQ(constrain_int32(i, 250, 500), 500);
      }
      else {
          EXPECT_EQ(constrain_float(i, 250, 500), i);        
          EXPECT_EQ(constrain_int16(i, 250, 500), i);
          EXPECT_EQ(constrain_int32(i, 250, 500), i);
      }
  }
  
  for(int i = 0; i <= 1000; i++) {
      int c = i - 1000;
      if(c < -250) {
          EXPECT_EQ(constrain_float(c, -250, -50), -250);
          EXPECT_EQ(constrain_int16(c, -250, -50), -250);
          EXPECT_EQ(constrain_int32(c, -250, -50), -250);
      }
      else if(c > -50) {
          EXPECT_EQ(constrain_float(c, -250, -50), -50);
          EXPECT_EQ(constrain_int16(c, -250, -50), -50);
          EXPECT_EQ(constrain_int32(c, -250, -50), -50);
      }
      else {
          EXPECT_EQ(constrain_float(c, -250, -50), c);        
          EXPECT_EQ(constrain_int16(c, -250, -50), c);
          EXPECT_EQ(constrain_int32(c, -250, -50), c);
      }
  }
  
  for(int i = 0; i <= 2000; i++) {
      int c = i - 1000;
      if(c < -250) {
          EXPECT_EQ(constrain_float(c, -250, 50), -250);
          EXPECT_EQ(constrain_int16(c, -250, 50), -250);
          EXPECT_EQ(constrain_int32(c, -250, 50), -250);
      }
      else if(c > 50) {
          EXPECT_EQ(constrain_float(c, -250, 50), 50);
          EXPECT_EQ(constrain_int16(c, -250, 50), 50);
          EXPECT_EQ(constrain_int32(c, -250, 50), 50);
      }
      else {
          EXPECT_EQ(constrain_float(c, -250, 50), c);        
          EXPECT_EQ(constrain_int16(c, -250, 50), c);
          EXPECT_EQ(constrain_int32(c, -250, 50), c);
      }
  }
}

TEST(Wrap180Test, angle180)
{
  // Full circle test
  for(int i = 0; i < 360; i++) {
      // smaller pole position
      if(i < 180) {
          EXPECT_EQ(wrap_180(i), i);
          EXPECT_EQ(wrap_180(-i), -i);
      }
      // hit pole position -180/+180 degree
      else if(i == 180) {
          EXPECT_EQ(wrap_180(i), i);
          EXPECT_EQ(wrap_180(-i), i);
      }
      // bigger pole position
      else {
          EXPECT_EQ(wrap_180(i), -(360-i));
          EXPECT_EQ(wrap_180(-i), (360-i));
      }
  }
  
  EXPECT_EQ(wrap_180(45.f),        45.f);
  EXPECT_EQ(wrap_180(90.f),        90.f);
  EXPECT_EQ(wrap_180(180.f),       180.f);
  EXPECT_EQ(wrap_180(180.1f),     -179.9f);
  EXPECT_EQ(wrap_180(270.f),      -90.f);
  EXPECT_EQ(wrap_180(360.f),       0.f);
  EXPECT_EQ(wrap_180(720.f),       0.f);
  EXPECT_EQ(wrap_180(3600.f),      0.f);
  EXPECT_EQ(wrap_180(7200.f),      0.f);
  EXPECT_EQ(wrap_180(-36000000.f), 0.f);
  
  EXPECT_EQ(wrap_180(-45.f),      -45.f);
  EXPECT_EQ(wrap_180(-90.f),      -90.f);
  EXPECT_EQ(wrap_180(-180.f),      180.f);
  EXPECT_EQ(wrap_180(-180.1f),     179.9f);
  EXPECT_EQ(wrap_180(-270.f),      90.f);
  EXPECT_EQ(wrap_180(-360.f),      0.f);
  EXPECT_EQ(wrap_180(-720.f),      0.f);
}

TEST(Wrap360Test, angle360)
{
  // Full circle test
  for(int i = 0; i <= 360; i++) {
      // hit pole position
      if(i == 0) {
          EXPECT_EQ(wrap_360(i), i);
          EXPECT_EQ(wrap_360(-i), i);
      }
      // between pole position
      else if(i < 360) {
          EXPECT_EQ(wrap_360(i), i);
          EXPECT_EQ(wrap_360(-i), 360-i);
      }
      // hit pole position
      else if(i == 360) {
          EXPECT_EQ(wrap_360(i), 0);
          EXPECT_EQ(wrap_360(-i), 0);
      }
  }
  
  EXPECT_EQ(wrap_360(45.f),        45.f);
  EXPECT_EQ(wrap_360(90.f),        90.f);
  EXPECT_EQ(wrap_360(180.f),       180.f);
  EXPECT_EQ(wrap_360(270.f),       270.f);
  EXPECT_EQ(wrap_360(360.f),       0.f);
  EXPECT_EQ(wrap_360(720.f),       0.f);
  EXPECT_EQ(wrap_360(3600.f),      0.f);
  EXPECT_EQ(wrap_360(7200.f),      0.f);
  EXPECT_EQ(wrap_360(-36000000.f), 0.f);
  
  EXPECT_EQ(wrap_360(-45.f),       315.f);
  EXPECT_EQ(wrap_360(-90.f),       270.f);
  EXPECT_EQ(wrap_360(-180.f),      180.f);
  EXPECT_EQ(wrap_360(-270.f),      90.f);
  EXPECT_EQ(wrap_360(-360.f),      0.f);
  EXPECT_EQ(wrap_360(-720.f),      0.f);
}
/*
TEST(WrapPiTest, pi_factor)
{  
  float wrap_1  =  wrap_PI(M_PI);
  float wrap_2  =  wrap_PI(M_2PI);
  float wrap_3  =  wrap_PI(M_PI_2);
  float wrap_4  =  wrap_PI(M_PI*10); // there is a loss in precision still
  
  const float accuracy = 1.0e-5;
  
  EXPECT_NEAR(wrap_1, M_PI, accuracy);
  EXPECT_NEAR(wrap_2, 0.f, accuracy);
  EXPECT_NEAR(wrap_3, M_PI_2, accuracy);
  EXPECT_NEAR(wrap_4, 0, accuracy);
}

TEST(Wrap2PiTest, pipi_factor)
{
  float wrap_1  = wrap_2PI(M_PI);
  float wrap_2  = wrap_2PI(M_2PI);
  float wrap_3  = wrap_2PI(M_PI_2);
  float wrap_4  = wrap_2PI(M_PI*10); // there is a loss in precision still
  
  float wrap_5  = wrap_2PI(0.f); // there is a loss in precision still
  
  float wrap_6  = wrap_2PI(-M_PI);
  float wrap_7  = wrap_2PI(-M_2PI);
  float wrap_8  = wrap_2PI(-M_PI_2);
  float wrap_9  = wrap_2PI(-M_PI*10); // there is a loss in precision still
  
  const float accuracy = 1.0e-5;
  
  EXPECT_NEAR(wrap_1, M_PI,         accuracy);
  EXPECT_NEAR(wrap_2, 0.f,          accuracy);
  EXPECT_NEAR(wrap_3, M_PI_2,       accuracy);  
  EXPECT_NEAR(wrap_4, M_2PI,        accuracy);
  
  EXPECT_NEAR(wrap_5, 0.f,          accuracy);

  EXPECT_NEAR(wrap_6, M_PI,         accuracy);
  EXPECT_NEAR(wrap_7, M_2PI,        accuracy);
  EXPECT_NEAR(wrap_8, M_2PI-M_PI_2, accuracy);
}
*/
TEST(VectorTest, Rotations)
{
    unsigned rotation_count = 0;

#define TEST_ROTATION(rotation, _x, _y, _z) { \
    const float accuracy = 1.0e-6; \
    Vector3f v(1, 1, 1); \
    v.rotate(rotation); \
    Vector3f expected(_x, _y, _z); \
    EXPECT_NEAR(expected.length(), v.length(), accuracy); \
    EXPECT_FLOAT_EQ(expected.x, v.x); \
    EXPECT_FLOAT_EQ(expected.y, v.y); \
    EXPECT_FLOAT_EQ(expected.z, v.z); \
    rotation_count++; \
}

    TEST_ROTATION(ROTATION_NONE, 1, 1, 1);
    TEST_ROTATION(ROTATION_YAW_45, 0, SQRT_2, 1);
    TEST_ROTATION(ROTATION_YAW_90, -1, 1, 1);
    TEST_ROTATION(ROTATION_YAW_135, -SQRT_2, 0, 1);
    TEST_ROTATION(ROTATION_YAW_180, -1, -1, 1);
    TEST_ROTATION(ROTATION_YAW_225, 0, -SQRT_2, 1);
    TEST_ROTATION(ROTATION_YAW_270, 1, -1, 1);
    TEST_ROTATION(ROTATION_YAW_315, SQRT_2, 0, 1);
    TEST_ROTATION(ROTATION_ROLL_180, 1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_45, SQRT_2, 0, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_90, 1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_135, 0, SQRT_2, -1);
    TEST_ROTATION(ROTATION_PITCH_180, -1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_225, -SQRT_2, 0, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_270, -1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_315, 0, -SQRT_2, -1);
    TEST_ROTATION(ROTATION_ROLL_90, 1, -1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_YAW_45, SQRT_2, 0, 1);
    TEST_ROTATION(ROTATION_ROLL_90_YAW_90, 1, 1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_YAW_135, 0, SQRT_2, 1);
    TEST_ROTATION(ROTATION_ROLL_270, 1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_270_YAW_45, 0, SQRT_2, -1);
    TEST_ROTATION(ROTATION_ROLL_270_YAW_90, -1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_270_YAW_135, -SQRT_2, 0, -1);
    TEST_ROTATION(ROTATION_PITCH_90, 1, 1, -1);
    TEST_ROTATION(ROTATION_PITCH_270, -1, 1, 1);
    TEST_ROTATION(ROTATION_PITCH_180_YAW_90, -1, -1, -1);
    TEST_ROTATION(ROTATION_PITCH_180_YAW_270, 1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_90, 1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_PITCH_90, -1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_270_PITCH_90, -1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_180, -1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_270_PITCH_180, -1, 1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_270, -1, -1, 1);
    TEST_ROTATION(ROTATION_ROLL_180_PITCH_270, 1, -1, 1);
    TEST_ROTATION(ROTATION_ROLL_270_PITCH_270, 1, 1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_180_YAW_90, 1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_90_YAW_270, -1, -1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_68_YAW_293, -0.4066309f, -1.5839677f, -0.5706992f);

    EXPECT_EQ(ROTATION_MAX, rotation_count) << "All rotations are expect to be tested";
}

AP_GTEST_MAIN()
