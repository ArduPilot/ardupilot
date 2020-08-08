//
// Unit tests for the AP_Math rotations code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void print_vector(Vector3f &v)
{
    hal.console->printf("[%.4f %.4f %.4f]\n",
                        (double)v.x,
                        (double)v.y,
                        (double)v.z);
}

// test rotation method accuracy
static void test_rotation_accuracy(void)
{
    Matrix3f attitude;
    Vector3f small_rotation;
    float roll, pitch, yaw;
    float rot_angle;

    hal.console->printf("\nRotation method accuracy:\n");

    // test roll
    for(int16_t i = 0; i < 90; i++ ) {

        // reset initial attitude
        attitude.from_euler(0.0f, 0.0f, 0.0f);

        // calculate small rotation vector
        rot_angle = ToRad(i);
        small_rotation = Vector3f(rot_angle, 0.0f, 0.0f);

        // apply small rotation
        attitude.rotate(small_rotation);

        // get resulting attitude's euler angles
        attitude.to_euler(&roll, &pitch, &yaw);

        // now try via from_axis_angle
        Matrix3f r2;
        r2.from_axis_angle(Vector3f(1.0f, 0.0f, 0.0f), rot_angle);
        attitude.from_euler(0.0f, 0.0f, 0.0f);
        attitude = r2 * attitude;

        float roll2, pitch2, yaw2;
        attitude.to_euler(&roll2, &pitch2, &yaw2);
        
        // display results
        hal.console->printf("actual angle: %d  angle1:%4.2f  angle2:%4.2f\n",
                            (int)i,
                            (double)ToDeg(roll),
                            (double)ToDeg(roll2));
    }

    // test pitch
    for(int16_t i = 0; i < 90; i++ ) {

        // reset initial attitude
        attitude.from_euler(0.0f, 0.0f, 0.0f);

        // calculate small rotation vector
        rot_angle = ToRad(i);
        small_rotation = Vector3f(0.0f ,rot_angle, 0.0f);

        // apply small rotation
        attitude.rotate(small_rotation);

        // get resulting attitude's euler angles
        attitude.to_euler(&roll, &pitch, &yaw);

        // now try via from_axis_angle
        Matrix3f r2;
        r2.from_axis_angle(Vector3f(0.0f ,1.0f, 0.0f), rot_angle);
        attitude.from_euler(0.0f, 0.0f, 0.0f);
        attitude = r2 * attitude;

        float roll2, pitch2, yaw2;
        attitude.to_euler(&roll2, &pitch2, &yaw2);
        
        // display results
        hal.console->printf("actual angle: %d  angle1:%4.2f  angle2:%4.2f\n",
                            (int)i,
                            (double)ToDeg(pitch),
                            (double)ToDeg(pitch2));
    }
    

    // test yaw
    for(int16_t i = 0; i < 90; i++ ) {

        // reset initial attitude
        attitude.from_euler(0.0f, 0.0f, 0.0f);

        // calculate small rotation vector
        rot_angle = ToRad(i);
        small_rotation = Vector3f(0.0f, 0.0f, rot_angle);

        // apply small rotation
        attitude.rotate(small_rotation);

        // get resulting attitude's euler angles
        attitude.to_euler(&roll, &pitch, &yaw);

        // now try via from_axis_angle
        Matrix3f r2;
        r2.from_axis_angle(Vector3f(0.0f, 0.0f, 1.0f), rot_angle);
        attitude.from_euler(0.0f, 0.0f, 0.0f);
        attitude = r2 * attitude;

        float roll2, pitch2, yaw2;
        attitude.to_euler(&roll2, &pitch2, &yaw2);
        
        // display results
        hal.console->printf("actual angle: %d  angle1:%4.2f  angle2:%4.2f\n",
                            (int)i,
                            (double)ToDeg(yaw),
                            (double)ToDeg(yaw2));
    }
}

static void test_euler(enum Rotation rotation, float roll, float pitch, float yaw)
{
    Vector3f v, v1, v2, diff;
    Matrix3f rotmat;
    const float accuracy = 1.0e-6f;

    v.x = 1;
    v.y = 2;
    v.z = 3;
    v1 = v;

    v1.rotate(rotation);
    
    rotmat.from_euler(radians(roll), radians(pitch), radians(yaw));
    v2 = v;
    v2 = rotmat * v2;

    diff = (v2 - v1);
    if (diff.length() > accuracy) {
        hal.console->printf("euler test %u failed : yaw:%d roll:%d pitch:%d\n",
        (unsigned)rotation,
        (int)yaw,
        (int)roll,
        (int)pitch);
        hal.console->printf("fast rotated: ");
        print_vector(v1);
        hal.console->printf("slow rotated: ");
        print_vector(v2);
        hal.console->printf("\n");
    }

    // quaternion rotation test
    const float q_accuracy = 1.0e-3f;
    Quaternion q, qe;
    q.from_rotation(rotation);
    qe.from_euler(radians(roll), radians(pitch), radians(yaw));
    float q_roll, q_pitch, q_yaw, qe_roll, qe_pitch, qe_yaw;
    q.to_euler(q_roll, q_pitch, q_yaw);
    qe.to_euler(qe_roll, qe_pitch, qe_yaw);
    const float roll_diff = fabsf(wrap_PI(q_roll - qe_roll));
    const float pitch_diff = fabsf(wrap_PI(q_pitch - qe_pitch));
    const float yaw_diff = fabsf(wrap_PI(q_yaw - qe_yaw));
    if ((roll_diff > q_accuracy) || (pitch_diff > q_accuracy) || (yaw_diff > q_accuracy)) {
        hal.console->printf("quaternion test %u failed : yaw:%f/%f roll:%f/%f pitch:%f/%f\n",
        (unsigned)rotation,
        (double)q_yaw,(double)qe_yaw,
        (double)q_roll,(double)qe_roll,
        (double)q_pitch,(double)qe_pitch);
    }
}

static void test_rotate_inverse(void)
{
    hal.console->printf("\nrotate inverse test(Vector (1,1,1)):\n");
    Vector3f vec(1.0f,1.0f,1.0f), cmp_vec(1.0f, 1.0f, 1.0f);
    for (enum Rotation r = ROTATION_NONE;
         r < ROTATION_MAX;
         r = (enum Rotation)((uint8_t)r+1)) {
        hal.console->printf("\nROTATION(%d) ", r);
        vec.rotate(r);
        print_vector(vec);

        hal.console->printf("INV_ROTATION(%d)", r);
        vec.rotate_inverse(r);
        print_vector(vec);
        if ((vec - cmp_vec).length() > 1e-5) {
            hal.console->printf("Rotation Test Failed!!! %.8f\n", (double)(vec - cmp_vec).length());
            return;
        }
    }
}
static void test_eulers(void)
{
    hal.console->printf("euler tests\n");
    test_euler(ROTATION_NONE,               0,   0,   0);
    test_euler(ROTATION_YAW_45,             0,   0,  45);
    test_euler(ROTATION_YAW_90,             0,   0,  90);
    test_euler(ROTATION_YAW_135,            0,   0, 135);
    test_euler(ROTATION_YAW_180,            0,   0, 180);
    test_euler(ROTATION_YAW_225,            0,   0, 225);
    test_euler(ROTATION_YAW_270,            0,   0, 270);
    test_euler(ROTATION_YAW_315,            0,   0, 315);
    test_euler(ROTATION_ROLL_180,         180,   0,   0);
    test_euler(ROTATION_ROLL_180_YAW_45,  180,   0,  45);
    test_euler(ROTATION_ROLL_180_YAW_90,  180,   0,  90);
    test_euler(ROTATION_ROLL_180_YAW_135, 180,   0, 135);
    test_euler(ROTATION_PITCH_180,          0, 180,   0);
    test_euler(ROTATION_ROLL_180_YAW_225, 180,   0, 225);
    test_euler(ROTATION_ROLL_180_YAW_270, 180,   0, 270);
    test_euler(ROTATION_ROLL_180_YAW_315, 180,   0, 315);
    test_euler(ROTATION_ROLL_90,           90,   0,   0);
    test_euler(ROTATION_ROLL_90_YAW_45,    90,   0,  45);
    test_euler(ROTATION_ROLL_90_YAW_90,    90,   0,  90);
    test_euler(ROTATION_ROLL_90_YAW_135,   90,   0, 135);
    test_euler(ROTATION_ROLL_270,         270,   0,   0);
    test_euler(ROTATION_ROLL_270_YAW_45,  270,   0,  45);
    test_euler(ROTATION_ROLL_270_YAW_90,  270,   0,  90);
    test_euler(ROTATION_ROLL_270_YAW_135, 270,   0, 135);
    test_euler(ROTATION_PITCH_90,           0,  90,   0);
    test_euler(ROTATION_PITCH_270,          0, 270,   0);
    test_euler(ROTATION_PITCH_180_YAW_90,   0, 180,  90);
    test_euler(ROTATION_PITCH_180_YAW_270,  0, 180, 270);
    test_euler(ROTATION_ROLL_90_PITCH_90,  90,  90,   0);
    test_euler(ROTATION_ROLL_180_PITCH_90,180,  90,   0);
    test_euler(ROTATION_ROLL_270_PITCH_90,270,  90,   0);
    test_euler(ROTATION_ROLL_90_PITCH_180, 90, 180,   0);
    test_euler(ROTATION_ROLL_270_PITCH_180,270,180,   0);
    test_euler(ROTATION_ROLL_90_PITCH_270, 90, 270,   0);
    test_euler(ROTATION_ROLL_180_PITCH_270,180,270,   0);
    test_euler(ROTATION_ROLL_270_PITCH_270,270,270,   0);
    test_euler(ROTATION_ROLL_90_PITCH_180_YAW_90, 90, 180,  90);
    test_euler(ROTATION_ROLL_90_YAW_270,   90,   0, 270);
    test_euler(ROTATION_ROLL_90_PITCH_68_YAW_293,90,68.8,293.3);
    test_euler(ROTATION_PITCH_7, 0, 7, 0);
}

static bool have_rotation(const Matrix3f &m)
{
    Matrix3f mt = m.transposed();
    for (enum Rotation r = ROTATION_NONE;
         r < ROTATION_MAX;
         r = (enum Rotation)((uint8_t)(r + 1))) {
        Vector3f v(1.0f, 2.0f, 3.0f);
        Vector3f v2 = v;
        v2.rotate(r);
        v2 = mt * v2;
        if ((v2 - v).length() < 0.01f) {
            return true;
        }
    }
    return false;
}

static void missing_rotations(void)
{
    hal.console->printf("testing for missing rotations\n");
    for (uint16_t yaw = 0; yaw < 360; yaw += 90)
        for (uint16_t pitch = 0; pitch < 360; pitch += 90)
            for (uint16_t roll = 0; roll < 360; roll += 90) {
                Matrix3f m;
                m.from_euler(ToRad(roll), ToRad(pitch), ToRad(yaw));
                if (!have_rotation(m)) {
                    hal.console->printf("Missing rotation (%u, %u, %u)\n", roll, pitch, yaw);
                }
            }
}

static void test_rotate_matrix(void)
{
    for (enum Rotation r = ROTATION_NONE;
         r < ROTATION_MAX;
         r = (enum Rotation)((uint8_t)r+1)) {
        //hal.console->printf("\nROTATION(%d)\n", r);
        Vector3f vec(1,2,3);
        Vector3f vec2 = vec;
        vec.rotate(r);
        Matrix3f m;
        m.from_rotation(r);
        vec2 = m * vec2;
        //print_vector(vec);
        //print_vector(vec2);
        if ((vec - vec2).length() > 1e-5) {
            hal.console->printf("Rotation Test Failed!!! %.8f\n", (double)(vec - vec2).length());
            return;
        }
    }
    hal.console->printf("test_rotate_matrix passed\n");
}


/*
 *  rotation tests
 */
void setup(void)
{
    hal.console->begin(115200);
    hal.console->printf("rotation unit tests\n\n");
    test_rotation_accuracy();
    test_eulers();
    missing_rotations();
    test_rotate_inverse();
    test_rotate_matrix();
    hal.console->printf("rotation unit tests done\n\n");
}

void loop(void) {}

AP_HAL_MAIN();
