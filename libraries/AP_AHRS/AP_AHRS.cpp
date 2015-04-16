/*
  APM_AHRS.cpp

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <AP_AHRS.h>
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS::var_info[] PROGMEM = {
	// index 0 and 1 are for old parameters that are no longer not used

    // @Param: GPS_GAIN
    // @DisplayName: AHRS GPS 增益
    // @Description: 这个参数控制如何使用GPS来保持正确的姿态。固定翼飞机不应该被设置为0，因为它会导致固定翼飞机在转弯时失去控制。固定翼飞机上请使用默认值1.0。
    // @Range: 0.0 1.0
    // @Increment: .01
    AP_GROUPINFO("GPS_GAIN",  2, AP_AHRS, gps_gain, 1.0f),

    // @Param: GPS_USE
    // @DisplayName: AHRS使用GPS导航
    // @Description: 这个参数控制是否使用AHRS和GPS导航。如果设置为0则不会使用GPS导航，只会使用AHRS。在正常的飞行中该值不应该被设置为0。
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("GPS_USE",  3, AP_AHRS, _gps_use, 1),

    // @Param: YAW_P
    // @DisplayName: Yaw P（偏航）
    // @Description: 这个参数控制电子罗盘和GPS上偏航信息的权重。较高的值意味着航向追踪偏航来源（来自GPS或电子罗盘）将更加频繁。
    // @Range: 0.1 0.4
    // @Increment: .01
    AP_GROUPINFO("YAW_P", 4,    AP_AHRS, _kp_yaw, 0.2f),

    // @Param: RP_P
    // @DisplayName: AHRS RP_P
    // @Description: 这个参数控制使用加速度计保持正确的姿态的速率。
    // @Range: 0.1 0.4
    // @Increment: .01
    AP_GROUPINFO("RP_P",  5,    AP_AHRS, _kp, 0.2f),

    // @Param: WIND_MAX
    // @DisplayName: 最大风速
    // @Description: 这个参数可以设置地面速度和飞行速度的最大允许的差值。这使得飞机可以应对空速传感器失效的状况。0值表示了在数据异常的状况下仍旧使用空速传感器的数据。
    // @Range: 0 127
    // @Units: m/s
    // @Increment: 1
    AP_GROUPINFO("WIND_MAX",  6,    AP_AHRS, _wind_max, 0.0f),

    // NOTE: 7 was BARO_USE

    // @Param: TRIM_X
    // @DisplayName: AHRS横滚配平
    // @Description: 补偿自动驾驶仪和车之间的横滚角差值。设置为正数使得车向右摇动。
    // @Units: 弧度
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: User

    // @Param: TRIM_Y
    // @DisplayName: AHRS俯仰配平
    // @Description: 补偿自动驾驶仪和车之间的俯仰角差值。设置为正数使得车前/后俯仰。
    // @Units: 弧度
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: User

    // @Param: TRIM_Z
    // @DisplayName: AHRS偏航配平
    // @Description: 未使用
    // @Units: 弧度
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TRIM", 8, AP_AHRS, _trim, 0),

    // @Param: ORIENTATION
    // @DisplayName: 主板的定位
    // @Description: 自动驾驶仪主板的方向相对于标准方向的朝向，取决于板的类型。改变该参数可以改变IMU和电子罗盘的读数，使得主板朝向可以与默认角度偏差90或45度。该设置将会在下一次重启后生效。改变此设置后你需要重新水平你的设备。
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270
    // @User: Advanced
    AP_GROUPINFO("ORIENTATION", 9, AP_AHRS, _board_orientation, 0),

    // @Param: COMP_BETA
    // @DisplayName: AHRS速度补偿滤波Beta系数
    // @Description: 这个参数控制在常数时间内使用交叉频率来融合AHRS（飞行速度和航向）和GPS数据从而估计地面速度。时间常数为0.1/beta。较大的时间常数使得GPS数据减少。相反的，一个小的时间常数将使得飞行数据减少。
    // @Range: 0.001 0.5
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("COMP_BETA",  10, AP_AHRS, beta, 0.1f),

    // @Param: GPS_MINSATS
    // @DisplayName: AHRS GPS最小卫星数
    // @Description: 利用GPS对速度姿态进行矫正所需要的最小卫星数。默认值为6，大约在这一点上来自GPS的速度数据对加速度计精度的矫正将会变得不可靠。
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_MINSATS", 11, AP_AHRS, _gps_minsats, 6),

    // NOTE: index 12 was for GPS_DELAY, but now removed, fixed delay
    // of 1 was found to be the best choice

#if AP_AHRS_NAVEKF_AVAILABLE
    // @Param: EKF_USE
    // @DisplayName: 使用NavEKF卡尔曼滤波器估算姿态和位置
    // @Description: 这个参数控制是否使用NavEKF卡尔曼滤波器进行姿态和位置的估算。
    // @Values: 0:关闭,1:启用
    // @User: Advanced
    AP_GROUPINFO("EKF_USE",  13, AP_AHRS, _ekf_use, AHRS_EKF_USE_DEFAULT),
#endif

    AP_GROUPEND
};

// return airspeed estimate if available
bool AP_AHRS::airspeed_estimate(float *airspeed_ret) const
{
	if (_airspeed && _airspeed->use()) {
		*airspeed_ret = _airspeed->get_airspeed();
		if (_wind_max > 0 && _gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
                    // constrain the airspeed by the ground speed
                    // and AHRS_WIND_MAX
                    float gnd_speed = _gps.ground_speed();
                    float true_airspeed = *airspeed_ret * get_EAS2TAS();
                    true_airspeed = constrain_float(true_airspeed,
                                                    gnd_speed - _wind_max, 
                                                    gnd_speed + _wind_max);
                    *airspeed_ret = true_airspeed / get_EAS2TAS();
		}
		return true;
	}
	return false;
}

// set_trim
void AP_AHRS::set_trim(Vector3f new_trim)
{
    Vector3f trim;
    trim.x = constrain_float(new_trim.x, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(new_trim.y, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    _trim.set_and_save(trim);
}

// add_trim - adjust the roll and pitch trim up to a total of 10 degrees
void AP_AHRS::add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom)
{
    Vector3f trim = _trim.get();

    // add new trim
    trim.x = constrain_float(trim.x + roll_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(trim.y + pitch_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));

    // set new trim values
    _trim.set(trim);

    // save to eeprom
    if( save_to_eeprom ) {
        _trim.save();
    }
}

// return a ground speed estimate in m/s
Vector2f AP_AHRS::groundspeed_vector(void)
{
    // Generate estimate of ground speed vector using air data system
    Vector2f gndVelADS;
    Vector2f gndVelGPS;
    float airspeed;
    bool gotAirspeed = airspeed_estimate_true(&airspeed);
    bool gotGPS = (_gps.status() >= AP_GPS::GPS_OK_FIX_2D);
    if (gotAirspeed) {
	    Vector3f wind = wind_estimate();
	    Vector2f wind2d = Vector2f(wind.x, wind.y);
	    Vector2f airspeed_vector = Vector2f(cosf(yaw), sinf(yaw)) * airspeed;
	    gndVelADS = airspeed_vector - wind2d;
    }
    
    // Generate estimate of ground speed vector using GPS
    if (gotGPS) {
        float cog = radians(_gps.ground_course_cd()*0.01f);
        gndVelGPS = Vector2f(cosf(cog), sinf(cog)) * _gps.ground_speed();
    }
    // If both ADS and GPS data is available, apply a complementary filter
    if (gotAirspeed && gotGPS) {
	    // The LPF is applied to the GPS and the HPF is applied to the air data estimate
	    // before the two are summed
	    //Define filter coefficients
	    // alpha and beta must sum to one
	    // beta = dt/Tau, where
	    // dt = filter time step (0.1 sec if called by nav loop)
	    // Tau = cross-over time constant (nominal 2 seconds)
	    // More lag on GPS requires Tau to be bigger, less lag allows it to be smaller
	    // To-Do - set Tau as a function of GPS lag.
	    const float alpha = 1.0f - beta; 
	    // Run LP filters
	    _lp = gndVelGPS * beta  + _lp * alpha;
	    // Run HP filters
	    _hp = (gndVelADS - _lastGndVelADS) + _hp * alpha;
	    // Save the current ADS ground vector for the next time step
	    _lastGndVelADS = gndVelADS;
	    // Sum the HP and LP filter outputs
	    return _hp + _lp;
    }
    // Only ADS data is available return ADS estimate
    if (gotAirspeed && !gotGPS) {
	    return gndVelADS;
    }
    // Only GPS data is available so return GPS estimate
    if (!gotAirspeed && gotGPS) {
	    return gndVelGPS;
    }
    return Vector2f(0.0f, 0.0f);
}

// update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
//      should be called after _dcm_matrix is updated
/*
功能:用get_dcm_matrix();得到当前飞行器的姿态以方向余矩阵DCM表示(3x3）。用方向余矩阵DCM计算姿态欧拉角（roll横滚角pitch仰俯角,yaw航向角）

*/
void AP_AHRS::update_trig(void)
{
    Vector2f yaw_vector;
    const Matrix3f &temp = get_dcm_matrix();

    // sin_yaw, cos_yaw
    yaw_vector.x = temp.a.x;
    yaw_vector.y = temp.b.x;
    yaw_vector.normalize();//对x,y矢量进行单位归一化
    _sin_yaw = constrain_float(yaw_vector.y, -1.0, 1.0);//限制范围因为sin,cos的范围只能在-1至1之间
    _cos_yaw = constrain_float(yaw_vector.x, -1.0, 1.0);

    // cos_roll, cos_pitch
    float cx2 = temp.c.x * temp.c.x;
    if (cx2 >= 1.0f) {
        _cos_pitch = 0;
        _cos_roll = 1.0f;
    } else {
        _cos_pitch = safe_sqrt(1 - cx2);
        _cos_roll = temp.c.z / _cos_pitch;
    }
    _cos_pitch = constrain_float(_cos_pitch, 0, 1.0);
    _cos_roll = constrain_float(_cos_roll, -1.0, 1.0); // this relies on constrain_float() of infinity doing the right thing,which it does do in avr-libc

    // sin_roll, sin_pitch
    _sin_pitch = -temp.c.x;
    _sin_roll = temp.c.y / _cos_pitch;
}

/*
  update the centi-degree values
  功能:将计算出来的欧拉角(roll横滚角pitch仰俯角,yaw航向角）弧度转化到角速并放大100倍。这样做的目的是方面后面的控制使用。
  degrees(）函数功能就是将角度转弧度.
 */
void AP_AHRS::update_cd_values(void)
{
    roll_sensor  = degrees(roll) * 100;
    pitch_sensor = degrees(pitch) * 100;
    yaw_sensor   = degrees(yaw) * 100;
    //因为航向角yaw是定义在0-360度弧度转角度之后范围会变成-180至180，所以这里是将们全部转到0-360度的范围内。
    if (yaw_sensor < 0)
        yaw_sensor += 36000;
}
