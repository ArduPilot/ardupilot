/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  ArduPlane parameter definitions
 *
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} } 
#define ASCALAR(v, name, def) { aparm.v.vtype, name, Parameters::k_param_ ## v, &aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
    // @Param: FORMAT_VERSION
    // @DisplayName: 存储器格式化次数
    // @Description: 每次更新存储器后会增加该数值。
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: 软件类型
    // @Description: 地面站用来识别软件的种类 (例如：固定翼 或 多轴)
    // @User: Advanced
    GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink系统ID
    // @Description: MAVLink协议中，当前设备的识别编号。
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: 地面站MAVLink系统ID
    // @Description: MAVLink协议中，地面站的识别编号。除非同时对地面站进行修改，否则不要更改。
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),

    // @Param: SERIAL0_BAUD
    // @DisplayName: USB端口波特率
    // @Description: 这是USB接口的波特率设置。APM2可以支持最高到115, 也可以支持500。PX4能支持到1500。如果你在APM2上设置了一个不能支持的波特率，导致无法连接飞控，可以通过重新加载固件的方式恢复默认参数来连接，这将导致所有设置的参数恢复默认。
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    GSCALAR(serial0_baud,           "SERIAL0_BAUD",   SERIAL0_BAUD/1000),

    // @Param: SERIAL1_BAUD
    // @DisplayName: 第一数传接口波特率
    // @Description: 通常这个设置作用于第一数传端口。APM2可以支持最高到115, 也可以支持500。PX4能支持到1500。如果你在APM2上设置了一个不能支持的波特率，导致无法连接飞控，可以通过重新加载固件的方式恢复默认参数来连接，这将导致所有设置的参数恢复默认。
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    GSCALAR(serial1_baud,           "SERIAL1_BAUD",   SERIAL1_BAUD/1000),

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // @Param: SERIAL2_BAUD
    // @DisplayName: 第二数传接口波特率
    // @Description: 这个设置作用于第二数传端口。APM2可以支持最高到115, 也可以支持500。PX4能支持到1500。如果你在APM2上设置了一个不能支持的波特率，导致无法连接飞控，可以通过重新加载固件的方式恢复默认参数来连接，这将导致所有设置的参数恢复默认。
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    GSCALAR(serial2_baud,           "SERIAL2_BAUD",   SERIAL2_BAUD/1000),

#if FRSKY_TELEM_ENABLED == ENABLED
    // @Param: SERIAL2_PROTOCOL
    // @DisplayName: 第二数传接口协议选择
    // @Description: 该参数控制第二数传接口使用哪种协议。
    // @Values: 1:地面站Mavlink,2:睿思凯D-PORT
    // @User: Standard
    GSCALAR(serial2_protocol,        "SERIAL2_PROTOCOL", SERIAL2_MAVLINK),
#endif // FRSKY_TELEM_ENABLED

#endif // MAVLINK_COMM_NUM_BUFFERS

    // @Param: AUTOTUNE_LEVEL
    // @DisplayName: 自动调参级别
    // @Description: 自动调参时的动作激烈程度。 当自动调参运行于较低级别时，调参的程度会比较“软”，得到较平和的增益。推荐大多数使用者设置为6。
    // @Range: 1 10
    // @Increment: 1
    // @User: Standard
    ASCALAR(autotune_level, "AUTOTUNE_LEVEL",  6),

    // @Param: TELEM_DELAY
    // @DisplayName: 数传启动延时 
    // @Description: 延迟数传连接的时间（秒），用于解决Xbee上电后会延时反应。
    // @User: Standard
    // @Units: 秒
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: KFF_RDDRMIX
    // @DisplayName: 方向舵混控
    // @Description: 方向舵配合副翼动作时的混控比例。0 = 0 %, 1 = 100%
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX",    RUDDER_MIX),

    // @Param: KFF_THR2PTCH
    // @DisplayName: 油门到俯仰的混控
    // @Description: 油门作用于俯仰的前馈增益。
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH",   0),

    // @Param: STAB_PITCH_DOWN
    // @DisplayName: 低油门下降微调 
    // @Description: 这个参数控制了，当在低油门杆量的时候，FBWA和AUTOTUNE模式飞行时的下降量。油门杆位置超过TRIM_THROTTLE参数的设定值时，飞机不会进行下降修正。低于TRIM_THROTTLE参数的设定值时，就会按照具体的油门杆位置，进行下降修正。0油门杆位置时，就会使用当前参数设定的量进行下降。这个参数的目的就是为了在FBWA模式小油门量飞行时，帮助飞机保持空速，比如在着陆进近时可以不用依赖空速计。2度的默认值，适合大部分飞机，更高的数值可能适合较大飞行阻力的飞机。
    // @Range: 0 15
    // @Increment: 0.1
    // @Units: 度
    // @User: Advanced
    GSCALAR(stab_pitch_down, "STAB_PITCH_DOWN",   2.0f),

    // @Param: GLIDE_SLOPE_MIN
    // @DisplayName: 滑翔下降高度
    // @Description: 设置最小的航点高度改变，可以进行滑翔，而不用直接下降高度。默认值为15米，这将帮助飞机在两个较近的航点间，平滑的进行高度调整。如果不想在此使用滑翔功能，设置0为禁用。
    // @Range: 0 1000
    // @Increment: 1
    // @Units: 米
    // @User: Advanced
    GSCALAR(glide_slope_threshold, "GLIDE_SLOPE_MIN", 15),

    // @Param: STICK_MIXING
    // @DisplayName: 摇杆混控
    // @Description: 如果允许使用，将在自动飞行模式时，允许RC控制飞机而不用切换飞行模式。这里有两种模式可以设置，设置为1时，可以用FBW的模式来控制飞机，这和FBW-A的侧倾和俯仰控制一样。如果你常用FBW-A或FBW-B模式来飞行，这是个安全的选择。设置为2时，将使用自稳模式STABILIZE来飞行，可以让你在AUTO模式进行更激烈的操作。
    // @Values: 0:Disabled,1:FBWMixing,2:DirectMixing
    // @User: Advanced
    GSCALAR(stick_mixing,           "STICK_MIXING",   STICK_MIXING_FBW),

    // @Param: SKIP_GYRO_CAL
    // @DisplayName: 禁止陀螺仪启动自检
    // @Description: 当允许这个选项时，APM将在启动时跳过陀螺仪校准，并使用上次飞行时保存的陀螺仪数据，如此一定要在飞行前确认高度等数据，因为一些板子可能在启动时的陀螺仪校准，会有明显的不同, 尤其是温度的变化带来的影响。而如果陀螺仪校准被跳过，APM将在启动的几分钟后，依赖陀螺仪的移动探测功能去收集正确的校准数据。这个功能在解决某些问题时非常有用。
    // @Values: 0:禁用,1:允许
    // @User: Advanced
    GSCALAR(skip_gyro_cal,           "SKIP_GYRO_CAL",   0),

    // @Param: AUTO_FBW_STEER
    // @DisplayName: 在AUTO飞行模式里使用FBW-A的控制功能
    // @Description: 允许这个选项后，会在Auto飞行模式里，允许使用与FBW-A相同的手动操控功能，而Auto模式的导航功能将被完全禁用，常规飞行时不建议使用此功能。
    // @Values: 0:禁用,1:允许
    // @User: Advanced
    GSCALAR(auto_fbw_steer,          "AUTO_FBW_STEER",   0),

    // @Param: TKOFF_THR_MINSPD
    // @DisplayName: 起飞时自动开启油门的最小速度
    // @Description: 该参数用于控制自动起飞时，油门启动与否，所依据的最小GPS地面速度检测。这个参数用在弹射起飞时，希望飞机被弹射出去后，再启动并加速电机。但对于弹射起飞，最好配合 TKOFF_THR_MINACC 和 TKOFF_THR_DELAY 参数使用进行，以规避GPS误差而导致的意外。强烈建议螺旋桨飞机投掷起飞时，设置的数值不低于4米/秒，这可以避免电机过早启动。注意：GPS速率会滞后于实际0.5秒。同时地速检测也会被 TKOFF_THR_DELAY 参数所延迟。
    // @Units: 米/秒
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_throttle_min_speed,     "TKOFF_THR_MINSPD",  0),

    // @Param: TKOFF_THR_MINACC
    // @DisplayName: 起飞时自动开启油门的最小加速度
    // @Description: 自动起飞模式下，油门启动需要的最小的向前地面加速度（米/秒/秒），这意味着要投掷起飞。设置为0将禁用加速度检测，油门将处于解锁状态，并允许GPS速率启动电机。投掷和弹射起飞应该设置在15米/秒附近。
    // @Units: 米/秒/秒
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_throttle_min_accel,     "TKOFF_THR_MINACC",  0),

    // @Param: TKOFF_THR_DELAY
    // @DisplayName: 起飞油门延迟开启
    // @Description: 当TKOFF_THR_MINACC参数设置的最小加速度达到后，再延迟多长时间（1/10秒）电机再启动。后推式螺旋桨飞机在投掷起飞时，这个值不能低于2（0.2秒）来确保螺旋桨离开投掷者的手臂范围后再启动。弹射起飞时，可以设置较大的数值（比如30）来给出足够的时间，等飞机离开弹射架后再启动电机。
    // @Units: 0.1 秒
    // @Range: 0 127
    // @Increment: 1
    // @User: User
    GSCALAR(takeoff_throttle_delay,     "TKOFF_THR_DELAY",  2),

    // @Param: TKOFF_TDRAG_ELEV
    // @DisplayName: 后三点机型的起飞时升降舵设置
    // @Description: 这个参数用来设置在最初的起飞阶段，升降舵面的应用比例。这个功能是用于在地面滑行时，保持住后三点飞机的尾轮（转向轮）,保持飞机在起飞时的方向稳定。这个参数应该结合 TKOFF_TDRAG_SPD1 和 GROUND_STEER_ALT 参数一起进行地面转向控制的调整。0值为忽略在起飞时的尾轮保持，并用在投掷和弹射起飞模式下。对于后三点飞机，通常设置为100，这将在起飞时使升降舵处于爬升满舵状态。0值适用于大多数前三点起落架，但是有一些前三点飞机需要在起飞时让升降舵向下，用来保持飞机在起飞加速时，前轮能保持对方向的控制（设置为-20到-30）。除非发现起飞离地前，前轮不能很好的接触地面，否则不要设置为负值。前三点飞机使用大幅度的升降舵向下时，会造成转向不灵，所以调试时，每次最多降低10%来测试。
    // @Units: 百分比
    // @Range: -100 100
    // @Increment: 1
    // @User: User
    GSCALAR(takeoff_tdrag_elevator,     "TKOFF_TDRAG_ELEV",  0),

    // @Param: TKOFF_TDRAG_SPD1
    // @DisplayName: 后三点机型起飞离地前的地面速度1
    // @Description: 这个参数设置在起飞离地前，达到什么速度后停止保持尾部向下，而转用方向舵控制地面转向。当这个参数值达到后，飞机将保持水平，直到速度达到TKOFF_ROTATE_SPD参数的设置，才会开始转向、爬升、飞向航点。设置为0时将直接进入转向，适合投掷和弹射起飞，前三点飞机也应设为0。后三点飞机的设置应稍低于失速速度。
    // @Units: 米/秒
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_tdrag_speed1,     "TKOFF_TDRAG_SPD1",  0),

    // @Param: TKOFF_ROTATE_SPD
    // @DisplayName: 起飞后开始转向所需达到速度
    // @Description: 起飞后达到什么速度开始转向。这个参数设定在任务中达到设定空速后，飞机开始转向并进行指定的爬升。如果这个参数为0，起飞后会立即进入爬升。手抛和弹射起飞应设置为0，所有地面起飞应设置为超过失速的速度，通常为10-30%。
    // @Units: 米/秒
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_rotate_speed,     "TKOFF_ROTATE_SPD",  0),

    // @Param: TKOFF_THR_SLEW
    // @DisplayName: 起飞油门增速
    // @Description: 这个参数设置了自动起飞时，油门的增减速率。设置为0时，起飞将使用 THR_SLEWRATE 的设置数值。在侧倾旋转起飞时，最好设置为较低的油门增减速率，较慢的加速将增强地面转向的控制力。这个数值是百分比/秒，所以，20意味着5秒后才会达到起飞油门量。不推荐使用低于20的值，它会导致飞机在小油门量的时候就开始爬升。
    // @Units: 百分比
    // @Range: 0 127
    // @Increment: 1
    // @User: User
    GSCALAR(takeoff_throttle_slewrate, "TKOFF_THR_SLEW",  0),

    // @Param: TKOFF_FLAP_PCNT
    // @DisplayName: 起飞襟翼百分比
    // @Description: 自动起飞时，襟翼打开的开度（百分比）。
    // @Range: 0 100
    // @Units: 百分比
    // @User: Advanced
    GSCALAR(takeoff_flap_percent,     "TKOFF_FLAP_PCNT", 0),

    // @Param: FBWA_TDRAG_CHAN
    // @DisplayName: FBWA模式下的后三点起飞模式
    // @Description: 选择一个RC通道进行设置，该通道PWM值超过1700以上的时候，在FBW-A模式中，启用后三点起落架的起飞模式。该通道应该被设置到RC遥控的某个两段开关上。起飞使用时，一旦这个功能被打开，将保持这个起飞模式，直到飞机空速超过TKOFF_TDRAG_SPD1 参数的设定值，除非中途改变飞行模式，或俯仰姿态有变化。同时，这个功能一旦打开，升降舵将强制进入TKOFF_TDRAG_ELEV的设定值。这些功能都是为了能让后三点式飞机在FBW-A模式中更容易的起飞，同时方便处理自动起飞和转向。0为禁用。
    // @User: Standard
    GSCALAR(fbwa_tdrag_chan,          "FBWA_TDRAG_CHAN",  0),

    // @Param: LEVEL_ROLL_LIMIT
    // @DisplayName: 水平飞行时侧倾度限制
    // @Description: 平飞时允许机身进行侧倾调整的最大角度（例如5度）。比如在起飞着陆时，过大的允许角度，可能会使机翼擦碰到跑道。设置为0时，在自动起飞和着陆时将完全禁用方向保持。
    // @Units: 度
    // @Range: 0 45
    // @Increment: 1
    // @User: User
    GSCALAR(level_roll_limit,              "LEVEL_ROLL_LIMIT",   5),

    // @Param: LAND_PITCH_CD
    // @DisplayName: 着陆俯仰角
    // @Description: 在没有空速计的飞机自动着陆时，使用的俯仰分度（百分之一度）。
    // @Units: 分度
    // @User: Advanced
    ASCALAR(land_pitch_cd,          "LAND_PITCH_CD",  0),

    // @Param: LAND_FLARE_ALT
    // @DisplayName: 着陆平飘高度
    // @Description: 自动着陆时，在什么高度锁定机头方向并开始平飘到LAND_PITCH_CD设定的着陆俯仰角度
    // @Units: 米
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(land_flare_alt,          "LAND_FLARE_ALT",  3.0),

    // @Param: LAND_FLARE_SEC
    // @DisplayName: 着陆平飘时间
    // @Description: 距着陆点还有有多长时间时，锁定方向和平飘到到LAND_PITCH_CD设定的俯仰角度。
    // @Units: 秒
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(land_flare_sec,          "LAND_FLARE_SEC",  2.0),

	// @Param: NAV_CONTROLLER
	// @DisplayName: N导航控制器选择
	// @Description: 允许使用什么导航控制系统，当前只有一个L1可选，未来可能会有增加实验性的选项。
	// @Values: 0:默认,1:L1控制器
	// @User: Standard
	GSCALAR(nav_controller,          "NAV_CONTROLLER",   AP_Navigation::CONTROLLER_L1),

    // @Param: ALT_MIX
    // @DisplayName: GPS混合气压定高比例
    // @Description: 混合GPS高度和气压高度的计算权重比例，0为绝对依据GPS，1为绝对用气压，强烈建议不要更改默认值1，也就要保持使用气压定高，因为GPS定高实在不可靠。当然，除非你有个高精度的GPS，并且是在几万米以上的高空用气球投放飞机。
    // @Units: 百分比
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(altitude_mix,           "ALT_MIX",        ALTITUDE_MIX),

    // @Param: ALT_CTRL_ALG
    // @DisplayName: 高度控制算法
    // @Description: 控制飞行高度用什么算法。默认的0会根据机身选择最合适的算法，当前这个默认的算法使用的是TECS（总体动力控制系统），未来会不定时的增加实验性的新算法。
    // @Values: 0:自动
    // @User: Advanced
    GSCALAR(alt_control_algorithm, "ALT_CTRL_ALG",    ALT_CONTROL_DEFAULT),

    // @Param: ALT_OFFSET
    // @DisplayName: 高度偏移量
    // @Description: 在执行自动飞行（Auto）任务时，添加目标高度。这里可以使用全球的海拔高度到任务里。
    // @Units: 米
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Advanced
    GSCALAR(alt_offset, "ALT_OFFSET",                 0),

    // @Param: WP_RADIUS
    // @DisplayName: 航点半径
    // @Description: 定义距离航点的最大半径距离，并在此距离之内经过即可算为已经抵达航点。为了防止飞机在抵达当前航点时，因为航路垂直于上个航点，而错过了航点导致飞机会反复绕圈。这里设置的最大半径完成线，只要穿过就算作已经抵达航点。注意，在一个航点前，导航控制器可能会在这个定义的最大半径之后才控制转向，这决定于转向角度的大小和当前飞机速度。最好设置这个参数较大于飞机的正常转向半径，这样导航控制器才能确保工作正常。如果设置这个参数过小，就会造成转弯过度的现象。
    // @Units: Meters
    // @Range: 1 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",      WP_RADIUS_DEFAULT),

    // @Param: WP_MAX_RADIUS
    // @DisplayName: 航点最大半径
    // @Description: 设置一个距航点的最大距离来帮辅助确认抵达航点。这个参数优先于“跨越完成线”的逻辑去认为航点已经抵达，对于普通的自动控制，这个参数应该设为0。除非飞机只是接近定义的半径而反复绕圈试图抵达航点时，才建议修改此参数。 如果飞机的转弯半径大于这个设定值，还是会造成飞机反复绕圈。
    // @Units: 米
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_max_radius,        "WP_MAX_RADIUS",      0),

    // @Param: WP_LOITER_RAD
    // @DisplayName: 定点半径
    // @Description: 定义飞机在进行定点飞行时，到定点中心的半径距离。如果设置为负值，将以该数值进行逆时针飞行。
    // @Units: 米
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(loiter_radius,          "WP_LOITER_RAD",  LOITER_RADIUS_DEFAULT),

#if GEOFENCE_ENABLED == ENABLED
    // @Param: FENCE_ACTION
    // @DisplayName: 触发围栏后的动作
    // @Description: 设置围栏触发后怎么办。 设置为0不会有任何动作；设置为1时触发GUIDED模式, 同时目标航点成为围栏返回点；设置为2时只报告，不做任何动作；3会进入GUIDED模式并保持手动油门控制。
    // @Values: 0:不反应,1:Guided模式,2:只报告,3:手动油门的Guided模式
    // @User: Standard
    GSCALAR(fence_action,           "FENCE_ACTION",   0),

    // @Param: FENCE_TOTAL
    // @DisplayName: 围栏数量
    // @Description: 当前启用的围栏点数量。
    // @User: Advanced
    GSCALAR(fence_total,            "FENCE_TOTAL",    0),

    // @Param: FENCE_CHANNEL
    // @DisplayName: 围栏遥控通道
    // @Description: 开启围栏功能的RC遥控通道，PWM超过1750将开启围栏。
    // @User: Standard
    GSCALAR(fence_channel,          "FENCE_CHANNEL",  0),

    // @Param: FENCE_MINALT
    // @DisplayName: 围栏最小高度
    // @Description: 触发围栏功能的最小高度限制。
    // @Units: 米
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_minalt,           "FENCE_MINALT",   0),

    // @Param: FENCE_MAXALT
    // @DisplayName: 围栏最大高度
    // @Description: 触发围栏功能的最大高度。
    // @Units: 米
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_maxalt,           "FENCE_MAXALT",   0),

    // @Param: FENCE_RETALT
    // @DisplayName: 围栏触发返回高度
    // @Description: 定义围栏被触发后，飞机将返回到什么高度；0为回到围栏的最大高度和最小高度的中间点。
    // @Units: 米
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_retalt,           "FENCE_RETALT",   0),

    // @Param: FENCE_AUTOENABLE
    // @DisplayName: 围栏自动开启
    // @Description: 围栏自动开启功能，设置为1时，围栏在自动起飞时打开，并在开始自动降落时关闭。建议不要在目视飞行时使用，实在要用，就用通道开关（FENCE_CHANNEL）来控制围栏的启用与否。
    // @Values: 0:禁止自动启用,1:自动开启
    // @User: Standard
    GSCALAR(fence_autoenable,       "FENCE_AUTOENABLE", 0),

    // @Param: FENCE_RET_RALLY
    // @DisplayName: 围栏返回集结点
    // @Description: 触发围栏后是否返回集结点，1为打开此功能；如果没有设置集结点，将会返回home point。
    // @Values: 0:返回出发点,1:返回最近的集结点
    // @User: Standard
    GSCALAR(fence_ret_rally,        "FENCE_RET_RALLY",  0),     
#endif

    // @Param: STALL_PREVENTION
    // @DisplayName: 阻止失速
    // @Description: 这个参数用来控制阻止失速的功能，功能包括限制低速下的侧倾角度和提高转弯时的最小空速，这些限制都是基于倾转时的空气动力因素，且这个参数依托于正确设置的 ARSPD_FBW_MIN 数值。注意，如果飞机没有空速计，阻止失速功能将使用基于地速和气流的评估空速，进行自动倾转响应。 这个合成的空速可能会不正确，所以在没有空速计的情况下，不能绝对认为这个功能事有效地。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    ASCALAR(stall_prevention, "STALL_PREVENTION",  1),

    // @Param: ARSPD_FBW_MIN
    // @DisplayName: 最小空速
    // @Description: 在有自动油门控制的飞行模式中，所允许的最小空速。这个数值应设置为高于飞机失速速度的20%左右，同时这个参数也作用于STALL_PREVENTION参数。
    // @Units: 米/秒
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_min, "ARSPD_FBW_MIN",  AIRSPEED_FBW_MIN),

    // @Param: ARSPD_FBW_MAX
    // @DisplayName: 最大空速
    // @Description: 在所有自动油门控制的飞行模式中，所允许的最大空速。你应该确保这个数值足够高于ARSPD_FBW_MIN 的参数值，来保证足够的高度和空速控制能力，建议该值最少要超过 ARSPD_FBW_MIN数值50%以上。
    // @Units: 米/秒
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_max, "ARSPD_FBW_MAX",  AIRSPEED_FBW_MAX),

    // @Param: FBWB_ELEV_REV
    // @DisplayName: FBW模式下升降舵反向
    // @Description: 在FBWB和CRUISE模式中，升降舵反向。设置为0时，向后拉升降舵杆将会降低高度；设置为1时，向后拉升降舵杆为提升高度。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    GSCALAR(flybywire_elev_reverse, "FBWB_ELEV_REV",  0),

#if AP_TERRAIN_AVAILABLE
    // @Param: TERRAIN_FOLLOW
    // @DisplayName: 使用地形跟随
    // @Description: 本参数允许CRUISE模式, FBWB模式, RTL和集结点等飞行模式使用地形跟随功能。使用这个功能需要设置 TERRAIN_ENABLE 为1，这将允许从地面站获得地形数据，且需要地面站支持发送地形数据到飞控。当允许使用地形跟随功能时，CRUISE和FBWB飞行模式将保持飞机在地形高度之上，而不是home点高度之上，这意味着有时飞行的高度可能还会低于出发点高度。 在返回（RTL)模式下，出发点的实际高度计算会基于地形高度之上。集结点的高度也会被设置到地形高度以上。飞行中，不管实际高度是高于home点，还是高于地形，都会预先在航点上有所标示，且不会影响具体的飞行任务。要使用地形跟随功能，在地面站规划航点时，需要将航点的种类改为地形高度航点。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    GSCALAR(terrain_follow, "TERRAIN_FOLLOW",  0),

    // @Param: TERRAIN_LOOKAHD
    // @DisplayName: 地形预判
    // @Description: 这里控制地形跟随提前计算多远的距离，来确保跟随在前方的地形的高度之上。0值为不预判，所以控制器将只跟踪飞机之下的地形。在AUTO模式下也不会对下一个航点进行预判。
    // @Range: 0 10000
    // @Units: 米
    // @User: Standard
    GSCALAR(terrain_lookahead, "TERRAIN_LOOKAHD",  2000),
#endif

    // @Param: FBWB_CLIMB_RATE
    // @DisplayName: FBW-B高度改变速率
    // @Description: 在FBWB和CRUISE模式中，使用升降舵打满时，将会改变高度的速比。注意，飞机的实际爬升速率可以低于这个数值，这取决于空速和油门控制的设置。例如，设置为默认的2m/s，你满舵爬升10秒，将会爬升20米。
    // @Range: 1-10
	// @Increment: 0.1
    // @User: Standard
    GSCALAR(flybywire_climb_rate, "FBWB_CLIMB_RATE",  2.0f),

    // @Param: THR_MIN
    // @DisplayName: 最小油门设置
    // @Description: 飞控控制的最小油门开启比例（百分比）。在最后阶段的自动降落应该设置为0。
    // @Units: 百分比
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_min,           "THR_MIN",        THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: 最大油门设置
    // @Description: 飞控控制的最大油门开启比例（百分比）。
    // @Units: 百分比
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_max,           "THR_MAX",        THROTTLE_MAX),

    // @Param: TKOFF_THR_MAX
    // @DisplayName: 起飞最大油门量
    // @Description: 自动起飞时设置的最大油门量。如果设为0，将应用 THR_MAX 参数（自动控制时的最大油门量） 设定的油门量。
    // @Units: 百分比
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    GSCALAR(takeoff_throttle_max,   "TKOFF_THR_MAX",        0),

    // @Param: THR_SLEWRATE
    // @DisplayName: 油门变化速率
    // @Description: 每秒油门量变化的百分比。假如设为10，那么推油门时，油门的增加速度不会超过油门总量的10%/秒。
    // @Units: 百分比
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_slewrate,      "THR_SLEWRATE",   100),

    // @Param: FLAP_SLEWRATE
    // @DisplayName: 襟翼变化速率Flap slew rate
    // @Description: 襟翼输出的最大变化速率百分比/秒。比如，设定25，会让襟翼在一秒之内的动作速度，不超过襟翼总行程的25%。0值为不限定速率。
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    GSCALAR(flap_slewrate,          "FLAP_SLEWRATE",   75),

    // @Param: THR_SUPP_MAN
    // @DisplayName: 手动油门直通自动油门
    // @Description: 在自动飞行模式下，当飞控需要抑制油门时，通常会强制到0油门。如果允许这个选项，油门限制将被手动油门控制所取代。这在汽油发动机上很有用处，它能让你在起飞前一直手动控制保持怠速。
	// @Values: 0:禁用,1:允许
    // @User: Advanced
    GSCALAR(throttle_suppress_manual,"THR_SUPP_MAN",   0),

    // @Param: THR_PASS_STAB
    // @DisplayName: 自稳模式中的油门直通
    // @Description: 这个选项如果被允许使用，那么在 STABILIZE, FBWA 或 ACRO 飞行模式中，RC油门控制将直通到输出，不再受到最小油门量 THR_MIN 和最大油门量THR_MAX 的限制。这将对使用汽油发动机的风门开关设置非常有用，因为它抑制了油门低于最小时的影响。同时，对于比较短的跑道，可以让电动飞机用全油门快速升空，并在空中自动油门控制时，按照限定的最大油门量飞行，这在长时间处于爬升状态时，可以降低电调负载，节省电量。
	// @Values: 0:禁用,1:允许
    // @User: Advanced
    GSCALAR(throttle_passthru_stabilize,"THR_PASS_STAB",   0),

    // @Param: THR_FAILSAFE
    // @DisplayName: 油门失控保护
    // @Description: 当油门通道出现问题时，是否使用失控保护功能。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",   THROTTLE_FAILSAFE),


    // @Param: THR_FS_VALUE
    // @DisplayName: 油门失控保护触发值
    // @Description: 当油门通道PWM值低于多少时，触发失控保护功能。
    // @Range: 925 1100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_fs_value,      "THR_FS_VALUE",   THROTTLE_FS_VALUE),

    // @Param: TRIM_THROTTLE
    // @DisplayName: 巡航油门百分比
    // @Description: 正常飞行时的目标油门百分比。
    // @Units: 百分比
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_cruise,        "TRIM_THROTTLE",  THROTTLE_CRUISE),

    // @Param: THROTTLE_NUDGE
    // @DisplayName: 油门助推
    // @Description: 当允许这个选项时，在任何有自动油门的飞行模式中，油门摇杆的位置决定了持续增加或减少油门量的变化。如果使用了空速计，油门摇杆的位置超过50%时，飞控将持续增加空速直到达到ARSPD_FBW_MAX参数所设定的最大空速。如果没有空速计，油门摇杆推到什么位置，飞控就增加电机转速到什么位置。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    // @User: Standard
    GSCALAR(throttle_nudge,         "THROTTLE_NUDGE",  1),

    // @Param: FS_SHORT_ACTN
    // @DisplayName: 短时失控保护动作
    // @Description: 在AUTO, GUIDED 或 LOITER模式下，短超时 FS_SHORT_TIMEOUT 参数设定的时间达到，并触发失控保护后的选项。短时失控保护可以被丢失遥控信号（参考THR_FS_VALUE参数）和丢失地面站控制（参考FS_GCS_ENABL参数）所触发。如果设置为1，在有自稳和手动的模式下，短时失控保护将的触发将切换飞行模式到CIRCLE模式；如果设置为2，将切换到FBWA模式。在所有其他模式下（包括ATUO和GUIDED模式），如果设置为0，短时失控保护的触发不会改变飞行模式；如果设为1，将改为CIRCLE模式；如果设为2，将改为FBWA模式。参考 FS_LONG_ACTN 和 FS_LONG_TIMEOUT参数所触发的动作。
    // @Values: 0:继续任务,1:绕圈/返回出发点,2:滑行
    // @User: Standard
    GSCALAR(short_fs_action,        "FS_SHORT_ACTN",  SHORT_FAILSAFE_ACTION),

    // @Param: FS_SHORT_TIMEOUT
    // @DisplayName: 短时失控保护超时
    // @Description: 失控保护情况出现后，多长时间触发短时失控保护动作。默认为1.5秒。
    // @Units: 秒
    // @Range: 1 100
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(short_fs_timeout,        "FS_SHORT_TIMEOUT", 1.5f),

    // @Param: FS_LONG_ACTN
    // @DisplayName: 长时失控保护动作
    // @Description: 长时失控保护的时间达到后，会采取的动作。如果设置为0，不会触发任何动作。如果设为1，将触发回家（RTL）模式。如果设为2，将进入FBWA模式。
    // @Values: 0:继续,1:返回,2:滑翔
    // @User: Standard
    GSCALAR(long_fs_action,         "FS_LONG_ACTN",   LONG_FAILSAFE_ACTION),

    // @Param: FS_LONG_TIMEOUT
    // @DisplayName: 长时失控保护超时
    // @Description: 失控保护情况出现后，多长时间触发长时失控保护动作。默认为20秒。
    // @Units: 秒
    // @Range: 1 300
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(long_fs_timeout,        "FS_LONG_TIMEOUT", 20),

    // @Param: FS_BATT_VOLTAGE
    // @DisplayName: 电压保护
    // @Description: 触发失控保护的电压。设置为0为禁用此功能。如果填写了具体的数值，而电压一旦低于此数值超过10秒钟，飞控将进入返回（RTL）模式。
    // @Units: 伏特
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(fs_batt_voltage,        "FS_BATT_VOLTAGE", 0),

    // @Param: FS_BATT_MAH
    // @DisplayName: 电池电量保护（毫安时）
    // @Description: 低于设定的电池容量，将会触发失控保护。设为0禁用此功能。如果电池剩余容量低于设置的数值，将会立即触发返回（RTL）模式。
    // @Units: 毫安时
    // @Increment: 50
    // @User: Standard
    GSCALAR(fs_batt_mah,            "FS_BATT_MAH", 0),

    // @Param: FS_GCS_ENABL
    // @DisplayName: 地面站失控保护
    // @Description: 是否允许地面站数据传输的失控保护功能。如果FS_LONG_TIMEOUT参数设定的时间依然没有MAVLink心跳信号，将触发失控保护。这里有两种情况可以设置，设置为1时，如果飞控收不到MAVLink心跳信号，将触发失控保护。设置为2意味着不管是收不到飞控心跳信号，还是无法接收飞控的更新数据，都会触发失控保护。这个情况在 RADIO_STATUS里的remrssi项显示为0. (一般是由于在单向传输数据的地面站或飞控端的无线电噪音造成的）。警告：允许这个选项可能会在地面调试的时候，造成电机意外启动，所以应设置解锁要求ARMING_REQUIRED参数为允许。 
    // @Values: 0:禁用,1:心跳包,2:心跳包和REMRSSI
    // @User: Standard
    GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL", GCS_FAILSAFE_OFF),

    // @Param: FLTMODE_CH
    // @DisplayName: 飞行模式切换通道
    // @Description: 飞行模式切换所使用的遥控通道。
    // @User: Advanced
    GSCALAR(flight_mode_channel,    "FLTMODE_CH",     FLIGHT_MODE_CHANNEL),

    // @Param: FLTMODE1
    // @DisplayName: 飞行模式1
    // @Values: 0:手动,1:绕圈,2:自稳,3:教练,4:特技,5:FBWA,6:FBWB,7:巡航,8:自动调参,10:自动任务,11:返回,12:定点,15:引导
    // @User: Standard
    // @Description: 位置1 的飞行模式(910 to 1230 and above 2049)
    GSCALAR(flight_mode1,           "FLTMODE1",       FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: 飞行模式2
    // @Description: 位置2 的飞行模式 (1231 to 1360)
    // @Values: 0:手动,1:绕圈,2:自稳,3:教练,4:特技,5:FBWA,6:FBWB,7:巡航,8:自动调参,10:自动任务,11:返回,12:定点,15:引导
    // @User: Standard
    GSCALAR(flight_mode2,           "FLTMODE2",       FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: 飞行模式3
    // @Description: 位置3 的飞行模式 (1361 to 1490)
    // @Values: 0:手动,1:绕圈,2:自稳,3:教练,4:特技,5:FBWA,6:FBWB,7:巡航,8:自动调参,10:自动任务,11:返回,12:定点,15:引导
    // @User: Standard
    GSCALAR(flight_mode3,           "FLTMODE3",       FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: 飞行模式4
    // @Description:位置4 的飞行模式 (1491 to 1620)
    // @Values: 0:手动,1:绕圈,2:自稳,3:教练,4:特技,5:FBWA,6:FBWB,7:巡航,8:自动调参,10:自动任务,11:返回,12:定点,15:引导
    // @User: Standard
    GSCALAR(flight_mode4,           "FLTMODE4",       FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: 飞行模式5
    // @Description: 位置5 的飞行模式 (1621 to 1749)
    // @Values: 0:手动,1:绕圈,2:自稳,3:教练,4:特技,5:FBWA,6:FBWB,7:巡航,8:自动调参,10:自动任务,11:返回,12:定点,15:引导
    // @User: Standard
    GSCALAR(flight_mode5,           "FLTMODE5",       FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: 飞行模式6
    // @Description: 位置6 的飞行模式 (1750 to 2049)
    // @Values: 0:手动,1:绕圈,2:自稳,3:教练,4:特技,5:FBWA,6:FBWB,7:巡航,8:自动调参,10:自动任务,11:返回,12:定点,15:引导
    // @User: Standard
    GSCALAR(flight_mode6,           "FLTMODE6",       FLIGHT_MODE_6),

    // @Param: LIM_ROLL_CD
    // @DisplayName: 最大侧倾角度警告
    // @Description: 左右两个方向的侧倾度超过设定值后报警。
    // @Units: 分度
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(roll_limit_cd,          "LIM_ROLL_CD",    HEAD_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MAX
    // @DisplayName: 最大爬升角度警告
    // @Description: 爬升角度超过设定值后报警。
    // @Units: 分度
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    ASCALAR(pitch_limit_max_cd,     "LIM_PITCH_MAX",  PITCH_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MIN
    // @DisplayName: 最小下降角度警告
    // @Description: 下降角度超过设定值后报警。
    // @Units: centi-Degrees
    // @Range: -9000 0
    // @Increment: 1
    // @User: Standard
    ASCALAR(pitch_limit_min_cd,     "LIM_PITCH_MIN",  PITCH_MIN_CENTIDEGREE),

    // @Param: ACRO_ROLL_RATE
    // @DisplayName: 特技模式的侧倾速率
    // @Description: 在特技模式下，满打翻转舵时最大的侧倾速率。
    // @Units: 度/秒
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_roll_rate,          "ACRO_ROLL_RATE",    180),

    // @Param: ACRO_PITCH_RATE
    // @DisplayName: 特技模式的俯仰速率
    // @Description: 在特技模式下，满打升降舵时最大的升降速率。
    // @Units: 度/秒
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_pitch_rate,          "ACRO_PITCH_RATE",  180),

    // @Param: ACRO_LOCKING
    // @DisplayName: 特技模式高度锁定
    // @Description: 允许这个选项，在特技模式下松开遥控摇杆，将会锁定飞机的当前高度上。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    GSCALAR(acro_locking,             "ACRO_LOCKING",     0),

    // @Param: GROUND_STEER_ALT
    // @DisplayName: 地面转向高度
    // @Description: 设置在什么高度上，开始在方向舵上使用地面转向控制来修正方向。非0的数值，将会启用STEER2SRV控制器，在home的高度限定之内用转向舵控制。
    // @Units: 米
    // @Range: -100 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(ground_steer_alt,         "GROUND_STEER_ALT",   0),

    // @Param: GROUND_STEER_DPS
    // @DisplayName: 地面转向速率
    // @Description: 在RC上满方向舵杆时，应用在地面转向的舵面速率为多少度/秒。
    // @Units: 度/秒
    // @Range: 10 360
    // @Increment: 1
    // @User: Advanced
    GSCALAR(ground_steer_dps,         "GROUND_STEER_DPS",  90),

    // @Param: TRIM_AUTO
    // @DisplayName: 自动中间位调整
    // @Description: 是否允许从手动模式（manual）切换到其它飞行模式的时候，把当前副翼、升降舵和方向舵的RC遥控杆位置，定义为其它飞行模式对应的遥控杆中心点。它将使用当前各相关遥控杆的PWM值，作为RC1_TRIM, RC2_TRIM 和 RC4_TRIM 的值。默认为0是因为某些飞手不知道这个功能时，会导致误操作。启用这个选项后，用手动模式起飞，看看飞机有什么反应，然后切到FBW-A模式调整中间点，然后在手动模式中再次调整。每次切回手动模式的时候，APM都会设置输入端为中间点。全部设置好后，可以禁用此功能。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    GSCALAR(auto_trim,              "TRIM_AUTO",      AUTO_TRIM),

    // @Param: ELEVON_MIXING
    // @DisplayName: 升降副翼混控
    // @Description: 是否允许在升降副翼的输入和输出通道都使用混控。如果只想在输出通道使用升降副翼混控，到ELEVON_OUTPUT进行设置。
    // @Values: 0:禁用,1:允许
    // @User: User
    GSCALAR(mix_mode,               "ELEVON_MIXING",  ELEVON_MIXING),

    // @Param: ELEVON_REVERSE
    // @DisplayName: 升降副翼反向
    // @Description: 反向升降副翼混控。
    // @Values: 0:禁用,1:允许
    // @User: User
    GSCALAR(reverse_elevons,        "ELEVON_REVERSE", ELEVON_REVERSE),


    // @Param: ELEVON_CH1_REV
    // @DisplayName: 升降副翼1通道反向
    // @Description: 反向升降副翼通道1。
    // @Values: -1:禁用,1:允许
    // @User: User
    GSCALAR(reverse_ch1_elevon,     "ELEVON_CH1_REV", ELEVON_CH1_REVERSE),

    // @Param: ELEVON_CH2_REV
    // @DisplayName: 升降副翼2通道反向
    // @Description: 反向升降副翼通道2.
    // @Values: -1:禁用,1:允许
    // @User: User
    GSCALAR(reverse_ch2_elevon,     "ELEVON_CH2_REV", ELEVON_CH2_REVERSE),

    // @Param: VTAIL_OUTPUT
    // @DisplayName: V形尾翼输出设置
    // @Description: V尾的控制模式。允许使用的话，APM将对升降和方向通道进行软件混合控制。这里有4种不同的混控模式，将会对V尾飞机尾部的2个舵机分别进行控制。注意：不能设置RC输入为线路直通。
    // @Values: 0:禁用,1:上上,2:上下,3:下上,4:下下
    // @User: User
    GSCALAR(vtail_output,           "VTAIL_OUTPUT",  0),

    // @Param: ELEVON_OUTPUT
    // @DisplayName: 升降副翼输出设置
    // @Description: 使用软件进行升降副翼输出通道的混控。如果允许使用，APM将对副翼和升降舵通道进行软件混控。这里提供4种不同的混控模式，使升降舵集合进两个升降副翼系统。注意：这里不能使用RC遥控信号直通到混控输出端，比如通道8功能设置为手动（manual）。同时要对 MIXING_GAIN 进行混控增益选择。
    // @Values: 0:禁用,1:上上,2:上下,3:下上,4:下下
    // @User: User
    GSCALAR(elevon_output,           "ELEVON_OUTPUT",  0),

    // @Param: MIXING_GAIN
    // @DisplayName: 混控增益
    // @Description: 对V尾和升降副翼的混控输出增益。默认为0.5，可以确保不让混控过载，并允许上述两种混控在输入通道达到极限，而输出通道还能保持控制。硬件混控经常设置为1.0的增益，让舵机反应更快, 但是会有过载。如果在V尾或升降副翼上的舵机没有足够的反应速度，就可以在此提高这个参数，这个混控允许的输出范围在900-2100微秒。
    // @Range: 0.5 1.2
    // @User: User
    GSCALAR(mixing_gain,            "MIXING_GAIN",    0.5f),

    // @Param: SYS_NUM_RESETS
    // @DisplayName: 重置次数
    // @Description: APM板的重置次数。
    // @User: Advanced
    GSCALAR(num_resets,             "SYS_NUM_RESETS", 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 选择需要保存的Log文件种类，在APM2上，只有4MByte闪存，所以不要保存太多种类。各Log种类如下： ATTITUDE_FAST=1, ATTITUDE_MEDIUM=2, GPS=4, PerformanceMonitoring=8, ControlTuning=16, NavigationTuning=32, Mode=64, IMU=128, Commands=256, Battery=512, Compass=1024, TECS=2048, Camera=4096, RCandServo=8192, Sonar=16384, Arming=32768, LogWhenDisarmed=65536。将所需log功能的编号相加填入即可存储所需的数据。
    // @Values: 0:禁用,5190:APM2-默认,65535:PX4/Pixhawk-默认
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",    DEFAULT_LOG_BITMASK),

    // @Param: RST_SWITCH_CH
    // @DisplayName: 重置飞行模式通道
    // @Description: 设置一个RC通道，用来重新设定触发围栏前的最后一个飞行模式。
    // @User: Advanced
    GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",  0),

    // @Param: RST_MISSION_CH
    // @DisplayName: 重置任务通道
    // @Description: 重置任务到第一航点的RC通道设定。当这个设定的通道PWM值超过1750，任务将被重置。0为禁用。
    // @User: Advanced
    GSCALAR(reset_mission_chan,      "RST_MISSION_CH",  0),

    // @Param: TRIM_ARSPD_CM
    // @DisplayName: 目标空速
    // @Description: 在Auto模式下，当空速达到该设定值的时候，将开始对准目标点飞行。
    // @Units: cm/s
    // @User: User
    GSCALAR(airspeed_cruise_cm,     "TRIM_ARSPD_CM",  AIRSPEED_CRUISE_CM),

    // @Param: SCALING_SPEED
    // @DisplayName: 速度测量计算
    // @Description: 空速达到多少米/秒时，进行线性速度计算。注意，更改该参数会影响所有的P、I、D参数。
    // @Units: 米/秒
    // @User: Advanced
    GSCALAR(scaling_speed,        "SCALING_SPEED",    SCALING_SPEED),

    // @Param: MIN_GNDSPD_CM
    // @DisplayName: 最小地面速度
    // @Description: 使用空速计控制飞机时，所允许的最小地面速度。
    // @Units: 厘米/秒
    // @User: Advanced
    GSCALAR(min_gndspeed_cm,      "MIN_GNDSPD_CM",  MIN_GNDSPEED_CM),

    // @Param: TRIM_PITCH_CD
    // @DisplayName: 俯仰角度偏移量
    // @Description: 用来修正飞行中的俯仰中间点。建议在地面平置飞机进行调整。
    // @Units: 分度
    // @User: Advanced
    GSCALAR(pitch_trim_cd,        "TRIM_PITCH_CD",  0),

    // @Param: ALT_HOLD_RTL
    // @DisplayName: 返回高度
    // @Description: 返回到出发点的高度，这将是飞机在返回时瞄准的高度，也是回到到出发点并盘旋的高度，如果设为-1，飞机将使用当前高度进入RTL模式。注意，如果设定了集结点（rally point），那么集结点的预设高度，将会替换这个RTL模式的预设高度。
    // @Units: 厘米
    // @User: User
    GSCALAR(RTL_altitude_cm,        "ALT_HOLD_RTL",   ALT_HOLD_HOME_CM),

    // @Param: ALT_HOLD_FBWCM
    // @DisplayName: FBWB模式下的最小飞行高度
    // @Description: 在FBW-B和CRUISE飞行模式下，限制的最低飞行高度，如果飞机下降到这个设定值，飞控会自动修正，不使飞机低于这个预设的最小高度。0为不限制。
    // @Units: 厘米
    // @User: User
    GSCALAR(FBWB_min_altitude_cm,   "ALT_HOLD_FBWCM", ALT_HOLD_FBW_CM),

    // @Param: MAG_ENABLE
    // @DisplayName: 允许使用罗盘
    // @Description: 是否启用罗盘，注意：这个选项跟COMPASS_USE是不同的。这里启用的低感度的感应器，并且允许保存log文件，要使用罗盘进行导航，还必须设置 COMPASS_USE 为 1。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",     1),

    // @Param: FLAP_IN_CHANNEL
    // @DisplayName: 襟翼输入通道
    // @Description: 使用哪个遥控通道来控制襟翼。如果设定了某个遥控通道来控制，这个通道的最小到中心位置的控制行程会决定襟翼开度的百分比，如果输入信号高于中心位置的信号值，将使襟翼反向打开，变成减速板的功能。本选项需要结合启用该输出通道FUNCTION设置里的一个襟翼功能。当该遥控通道与自动襟翼功能组合使用后，会有更多种类的襟翼开启比例。这里必须同时启用FLAPERON_OUTPUT里的襟副翼混控设定。
    // @User: User
    GSCALAR(flapin_channel,         "FLAP_IN_CHANNEL",  0),

    // @Param: FLAPERON_OUTPUT
    // @DisplayName: 襟副翼输出设置
    // @Description: 是否允许软件控制襟副翼的输出。如果启用这个参数，APM将通过软件，给FLAPERON1和FLAPERON2这两个FUNCTION选项里的辅助选项，进行襟副翼混控调制。这里有4种不同的混控模式，对应到襟翼和副翼的混控伺服系统。注意：这里不能使用RC遥控信号直通到混控输出端，比如在APM1的通道8上使用手动控制。所以，如果你使用APM1，就要先设置飞行模式的控制通道到8以外的其他通道。同时要注意对混控的增益（MIXING_GAIN）调整到合适的大小。本参数不能与升降副翼（ELEVON_OUTPUT） 或 升降副翼增益（ELEVON_MIXING） 进行关联设置。
    // @Values: 0:禁用,1:上上,2:上下,3:下上,4:下下
    // @User: User
    GSCALAR(flaperon_output,        "FLAPERON_OUTPUT",  0),

    // @Param: FLAP_1_PERCNT
    // @DisplayName: 襟翼1位置
    // @Description: 当FLAP_1_SPEED参数的值被触发后，襟翼开启到什么位置（襟翼开启的百分比），0为禁止使用襟翼。
    // @Range: 0 100
    // @Units: 百分比
    // @User: Advanced
    GSCALAR(flap_1_percent,         "FLAP_1_PERCNT",  FLAP_1_PERCENT),

    // @Param: FLAP_1_SPEED
    // @DisplayName: 襟翼1开启速度
    // @Description: 当目标空速触及这个数值时，触发FLAP_1_PERCNT定义的襟翼开度。注意，这个速度应该大于或等于FLAP_2_SPEED。
    // @Range: 0 100
	// @Increment: 1
    // @Units: 米/秒
    // @User: Advanced
    GSCALAR(flap_1_speed,           "FLAP_1_SPEED",   FLAP_1_SPEED),

    // @Param: FLAP_2_PERCNT
    // @DisplayName: 襟翼2位置
    // @Description: 当FLAP_2_SPEED参数的值被触发后，襟翼开启到什么位置（襟翼开启的百分比），0为禁止使用襟翼。
    // @Range: 0 100
	// @Units: 百分比
    // @User: Advanced
    GSCALAR(flap_2_percent,         "FLAP_2_PERCNT",  FLAP_2_PERCENT),

    // @Param: FLAP_2_SPEED
    // @DisplayName: 襟翼2开启速度
    // @Description: 当目标空速触及这个数值时，触发FLAP_2_PERCNT定义的襟翼开度。注意，FLAP_1_SPEED应该大于或等于FLAP_2_SPEED。
    // @Range: 0 100
	// @Units: 米/秒
	// @Increment: 1
    // @User: Advanced
    GSCALAR(flap_2_speed,           "FLAP_2_SPEED",   FLAP_2_SPEED),

    // @Param: LAND_FLAP_PERCNT
    // @DisplayName: 着陆时襟翼开启程度
    // @Description:在自动着陆进近和平飘时襟翼开启的百分比。
    // @Range: 0 100
    // @Units: 百分比
    // @User: Advanced
    GSCALAR(land_flap_percent,     "LAND_FLAP_PERCNT", 0),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Param: OVERRIDE_CHAN
    // @DisplayName: PX4IO override channel
    // @Description: If set to a non-zero value then this is an RC input channel number to use for testing manual control in case the main FMU microcontroller on a PX4 or Pixhawk fails. When this RC input channel goes above 1750 the FMU will stop sending servo controls to the PX4IO board, which will trigger the PX4IO board to start using its failsafe override behaviour, which should give you manual control of the aircraft. That allows you to test for correct manual behaviour without actually crashing the FMU. This parameter is normally only set to a non-zero value for ground testing purposes. When the override channel is used it also forces the PX4 safety switch into an armed state. This allows it to be used as a way to re-arm a plane after an in-flight reboot. Use in that way is considered a developer option, for people testing unstable developer code. Note that you may set OVERRIDE_CHAN to the same channel as FLTMODE_CH to get PX4IO based override when in flight mode 6. Note that when override is triggered the 6 auxillary output channels on Pixhawk will no longer be updated, so all the flight controls you need must be assigned to the first 8 channels.
    // @User: Advanced
    GSCALAR(override_channel,      "OVERRIDE_CHAN",  0),
#endif

    // @Param: RSSI_PIN
    // @DisplayName: RSSI信号感应针脚
    // @Description: 选择那个辅助针脚用来接收RSSI电压。这里默认的最大rssi为5v电压，0为最小。
    // @Values: -1:禁用, 0:APM2 A0, 1:APM2 A1, 13:APM2 A13, 103:Pixhawk SBUS
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: RSSI_RANGE
    // @DisplayName: 定义接收RSSI的电压范围
    // @Description: 定义接收RSSI的电压范围
    // @Units: 伏特
    // @Values: 3.3:3.3V, 5.0:5V
    // @User: Standard
    GSCALAR(rssi_range,          "RSSI_RANGE",         5.0),

    // @Param: INVERTEDFLT_CH
    // @DisplayName: 倒飞遥控通道
    // @Description: 设置某个RC输入通道控制倒飞。当设置为某个通道后，该通道PWM值一旦超过1750，APM会控制飞机反转进行倒飞。
    // @Values: 0:禁用,1:通道1,2:通道2,3:通道3,4:通道4,5:通道5,6:通道6,7:通道7,8:通道8
    // @User: Standard
    GSCALAR(inverted_flight_ch,     "INVERTEDFLT_CH", 0),

#if HIL_MODE != HIL_MODE_DISABLED
    // @Param: HIL_SERVOS
    // @DisplayName: HIL Servos enable
    // @Description: This controls whether real servo controls are used in HIL mode. If you enable this then the APM will control the real servos in HIL mode. If disabled it will report servo values, but will not output to the real servos. Be careful that your motor and propeller are not connected if you enable this option.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(hil_servos,            "HIL_SERVOS",      0),

    // @Param: HIL_ERR_LIMIT
    // @DisplayName: Limit of error in HIL attitude before reset
    // @Description: This controls the maximum error in degrees on any axis before HIL will reset the DCM attitude to match the HIL_STATE attitude. This limit will prevent poor timing on HIL from causing a major attitude error. If the value is zero then no limit applies.
    // @Units: degrees
    // @Range: 0 90
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(hil_err_limit,         "HIL_ERR_LIMIT",   5),
#endif

    // @Param: RTL_AUTOLAND
    // @DisplayName: 返回并自动着陆
    // @Description: 返回出发点后自动执行着陆动作序列，这需要用 DO_LAND_START 任务设置并激活为一个着陆序列。当前位置会算则最近的着陆序列。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    GSCALAR(rtl_autoland,         "RTL_AUTOLAND",   0),

    // barometer ground calibration. The GND_ prefix is chosen for
    // compatibility with previous releases of ArduPlane
    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // GPS driver
    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,                  "CAM_", AP_Camera),
#endif

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming),

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/RangeFinder.cpp
    GOBJECT(rangefinder,            "RNGFND", RangeFinder),

    // @Param: RNGFND_LANDING
    // @DisplayName: 着陆时使用测距仪
    // @Description: 允许这个参数将会在自动着陆时使用测距仪。测距仪将会在进近和最后平飘时都会被使用。
    // @Values: 0:禁用,1:允许
    // @User: Standard
    GSCALAR(rangefinder_landing,    "RNGFND_LANDING",   0),

#if AP_TERRAIN_AVAILABLE
    // @Group: TERRAIN_
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_1,                    "RC1_", RC_Channel),

    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_2,                    "RC2_", RC_Channel),

    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_3,                    "RC3_", RC_Channel),

    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_4,                    "RC4_", RC_Channel),

    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,                    "RC5_", RC_Channel_aux),

    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,                    "RC6_", RC_Channel_aux),

    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,                    "RC7_", RC_Channel_aux),

    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,                    "RC8_", RC_Channel_aux),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Group: RC12_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_12,                    "RC12_", RC_Channel_aux),

    // @Group: RC13_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_13,                    "RC13_", RC_Channel_aux),

    // @Group: RC14_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_14,                    "RC14_", RC_Channel_aux),
#endif

    // @Group: RLL2SRV_
    // @Path: ../libraries/APM_Control/AP_RollController.cpp
	GOBJECT(rollController,         "RLL2SRV_",   AP_RollController),

    // @Group: PTCH2SRV_
    // @Path: ../libraries/APM_Control/AP_PitchController.cpp
	GOBJECT(pitchController,        "PTCH2SRV_",  AP_PitchController),

    // @Group: YAW2SRV_
    // @Path: ../libraries/APM_Control/AP_YawController.cpp
	GOBJECT(yawController,          "YAW2SRV_",   AP_YawController),

    // @Group: STEER2SRV_
    // @Path: ../libraries/APM_Control/AP_SteerController.cpp
	GOBJECT(steerController,        "STEER2SRV_",   AP_SteerController),

	// variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,                "COMPASS_",     Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap,                "RCMAP_",         RCMapper),

    // @Group: SR0_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[0], gcs0,        "SR0_",     GCS_MAVLINK),

    // @Group: SR1_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[1],  gcs1,       "SR1_",     GCS_MAVLINK),

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // @Group: SR2_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[2],  gcs2,       "SR2_",     GCS_MAVLINK),
#endif

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // @Group: ARSPD_
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed,                               "ARSPD_",   AP_Airspeed),

    // @Group: NAVL1_
    // @Path: ../libraries/AP_L1_Control/AP_L1_Control.cpp
    GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),

    // @Group: TECS_
    // @Path: ../libraries/AP_TECS/AP_TECS.cpp
    GOBJECT(TECS_controller,         "TECS_",   AP_TECS),

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

#if MOUNT2 == ENABLED
    // @Group: MNT2_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
#endif

    // @Group: BATT_
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT_",       AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL),
#endif

#if OBC_FAILSAFE == ENABLED
    // @Group: AFS_
    // @Path: ../libraries/APM_OBC/APM_OBC.cpp
    GOBJECT(obc,  "AFS_", APM_OBC),
#endif

#if AP_AHRS_NAVEKF_AVAILABLE
    // @Group: EKF_
    // @Path: ../libraries/AP_NavEKF/AP_NavEKF.cpp
    GOBJECTN(ahrs.get_NavEKF(), NavEKF, "EKF_", NavEKF),
#endif

#if OPTFLOW == ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", OpticalFlow),
#endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_",       AP_Mission),

    // @Group: RALLY_
    // @Path: ../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,  "RALLY_",       AP_Rally),

    AP_VAREND
};

/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.

  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed

  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */
const AP_Param::ConversionInfo conversion_table[] PROGMEM = {
    { Parameters::k_param_pidServoRoll, 0, AP_PARAM_FLOAT, "RLL2SRV_P" },
    { Parameters::k_param_pidServoRoll, 1, AP_PARAM_FLOAT, "RLL2SRV_I" },
    { Parameters::k_param_pidServoRoll, 2, AP_PARAM_FLOAT, "RLL2SRV_D" },
    { Parameters::k_param_pidServoRoll, 3, AP_PARAM_FLOAT, "RLL2SRV_IMAX" },

    { Parameters::k_param_pidServoPitch, 0, AP_PARAM_FLOAT, "PTCH2SRV_P" },
    { Parameters::k_param_pidServoPitch, 1, AP_PARAM_FLOAT, "PTCH2SRV_I" },
    { Parameters::k_param_pidServoPitch, 2, AP_PARAM_FLOAT, "PTCH2SRV_D" },
    { Parameters::k_param_pidServoPitch, 3, AP_PARAM_FLOAT, "PTCH2SRV_IMAX" },

    { Parameters::k_param_battery_monitoring, 0,      AP_PARAM_INT8,  "BATT_MONITOR" },
    { Parameters::k_param_battery_volt_pin,   0,      AP_PARAM_INT8,  "BATT_VOLT_PIN" },
    { Parameters::k_param_battery_curr_pin,   0,      AP_PARAM_INT8,  "BATT_CURR_PIN" },
    { Parameters::k_param_volt_div_ratio,     0,      AP_PARAM_FLOAT, "BATT_VOLT_MULT" },
    { Parameters::k_param_curr_amp_per_volt,  0,      AP_PARAM_FLOAT, "BATT_AMP_PERVOLT" },
    { Parameters::k_param_curr_amp_offset,    0,      AP_PARAM_FLOAT, "BATT_AMP_OFFSET" },
    { Parameters::k_param_pack_capacity,      0,      AP_PARAM_INT32, "BATT_CAPACITY" },
    { Parameters::k_param_log_bitmask_old,    0,      AP_PARAM_INT16, "LOG_BITMASK" },
    { Parameters::k_param_rally_limit_km_old, 0,      AP_PARAM_FLOAT, "RALLY_LIMIT_KM" },
    { Parameters::k_param_rally_total_old,    0,      AP_PARAM_INT8, "RALLY_TOTAL" },
};

static void load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        cliSerial->printf_P(PSTR("Bad parameter table\n"));        
        hal.scheduler->panic(PSTR("Bad parameter table"));
    }
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        cliSerial->println_P(PSTR("done."));
    } else {
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();
        AP_Param::convert_old_parameters(&conversion_table[0], sizeof(conversion_table)/sizeof(conversion_table[0]));
        cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
