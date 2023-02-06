// Grab compass parameters from log and populate form
function compass_params(_instance) {
    this.instance = _instance
    this.valid = false

    this.offsets = {}
    this.diagonals = {}
    this.off_diagonals = {}
    this.motor = {}
    this.scale_raw = null
    this.scale = 1.0
    this.comp_type = null
    this.mat = Matrix3()
    this.mat.identity()

    this.compare_param_name = function(string, key, instance, axis) {
        if (!string.startsWith("COMPASS")) {
            return false
        }
        if ((axis != null) && !string.includes(axis)) {
            return false
        }
        if (instance != null) {
            if (instance == 0) {
                if (/\d/.test(string)) {
                    return false
                }
            } else {
                if (!string.includes((instance+1).toString())) {
                    return false
                }
            }
        }
        return string.includes(key)
    }

    this.show_param = function(param, key, instance, axis) {
        var inputs = document.forms["params"].getElementsByTagName("input");
        for (var i=0;i<inputs.length;i++) {
            if (this.compare_param_name(inputs[i].name, key, instance, axis)) {
                inputs[i].value = param
                return
            }
        }
        console.log(instance + "Could not show param:" + key + axis + instance)
        this.valid = false
    }

    this.show_param_vec = function(param, key) {
        this.show_param(param.X, key, this.instance, "X")
        this.show_param(param.Y, key, this.instance, "Y")
        this.show_param(param.Z, key, this.instance, "Z")
    }

    this.show = function() {
        this.show_param_vec(this.offsets, "OFS")
        this.show_param_vec(this.diagonals, "DIA")
        this.show_param_vec(this.off_diagonals, "ODI")
        this.show_param_vec(this.motor, "MOT")
        this.show_param(this.scale_raw, "SCALE", this.instance)
    }

    this.get_param_value = function(PARM_log, key, instance, axis) {
        var value = null
        for (let i = 0; i < PARM_log.Name.length; i++) {
            if (this.compare_param_name(PARM_log.Name[i], key, instance, axis)) {
                if (value == null) {
                    value = PARM_log.Value[i]
                } else if (value != PARM_log.Value[i]) {
                    // Cant deal with calibration changing during flight
                    console.log(instance + "duplicate param:" + key + " " + axis)
                    this.valid = false
                    return null
                }
            }
        }
        if (value == null) {
            console.log(instance + "Could not find param:" + key + axis)
            this.valid = false
        }
        return value
    }

    this.get_param_value_vec = function(param, PARM_log, key) {
        param.X = this.get_param_value(PARM_log, key, this.instance, "X")
        param.Y = this.get_param_value(PARM_log, key, this.instance, "Y")
        param.Z = this.get_param_value(PARM_log, key, this.instance, "Z")
    }

    this.load_from_log = function(PARM_log) {
        this.valid = true
        this.get_param_value_vec(this.offsets, PARM_log, "OFS")
        this.get_param_value_vec(this.diagonals, PARM_log, "DIA")
        this.get_param_value_vec(this.off_diagonals, PARM_log, "ODI")
        this.get_param_value_vec(this.motor, PARM_log, "MOT")
        this.scale_raw = this.get_param_value(PARM_log, "SCALE", this.instance)
        this.comp_type = this.get_param_value(PARM_log, "MOTCT")
        if (this.valid) {
            this.show()
            if ((this.diagonals.X != 0) || (this.diagonals.Y != 0) || (this.diagonals.Z != 0)) {
                // Setup matrix to apply to raw values
                this.mat.a.x = this.diagonals.X
                this.mat.a.y = this.off_diagonals.X
                this.mat.a.z = this.off_diagonals.y

                this.mat.b.x = this.off_diagonals.X
                this.mat.b.y = this.diagonals.Y
                this.mat.b.z = this.off_diagonals.z

                this.mat.c.x = this.off_diagonals.y
                this.mat.c.y = this.off_diagonals.z
                this.mat.c.z = this.diagonals.Z
            }
            const MAX_SCALE_FACTOR = 1.5
            if ((this.scale_raw <= MAX_SCALE_FACTOR) && (this.scale_raw >=  (1/MAX_SCALE_FACTOR))) {
                this.scale = this.scale_raw
            }
        }
    }

    this.apply = function(sample) {
        var ret = {}

        ret.x = (sample.raw_X + offsets.X) * this.scale
        ret.y = (sample.raw_Y + offsets.Y) * this.scale
        ret.z = (sample.raw_Z + offsets.Z) * this.scale

        ret = this.mat.mul(ret)

        if (this.comp_type == 1) {
            ret.x += motor.X * sample.QTUN_throttle
            ret.y += motor.Y * sample.QTUN_throttle
            ret.z += motor.Z * sample.QTUN_throttle
        } else if (this.comp_type == 1) {
            ret.x += motor.X * sample['BATT[0]'].Curr
            ret.y += motor.Y * sample['BATT[0]'].Curr
            ret.z += motor.Z * sample['BATT[0]'].Curr
        }

        return ret
    }

    this.remove = function(sample) {

        raw = {}
        raw.x = sample.X
        raw.y = sample.Y
        raw.z = sample.Z

        if (this.comp_type == 1) {
            ret.x -= motor.X * sample.QTUN_throttle
            ret.y -= motor.Y * sample.QTUN_throttle
            ret.z -= motor.Z * sample.QTUN_throttle
        } else if (this.comp_type == 1) {
            ret.x -= motor.X * sample['BATT[0]'].Curr
            ret.y -= motor.Y * sample['BATT[0]'].Curr
            ret.z -= motor.Z * sample['BATT[0]'].Curr
        }

        raw = this.mat.mul_transpose(raw)

        ret.x /= this.scale
        ret.y /= this.scale
        ret.z /= this.scale

        ret.x -= this.offsets.X
        ret.y -= this.offsets.Y
        ret.z -= this.offsets.Z

        return ret

    }

    return this
}

function get_param(PARM_log, name) {
    var value = null
    for (let i = 0; i < PARM_log.Name.length; i++) {
        if (PARM_log.Name[i] == name) {
            if (value == null) {
                value = PARM_log.Value[i]
            } else if (value != PARM_log.Value[i]) {
                // Cant deal with changing during flight
                console.log("Duplicate param:" + name)
                return null
            }
        }
    }
    if (value == null) {
        console.log("Could not find param:" + name)
    }
    return value
}

var axis = ['X', 'Y', 'Z']
var chart = {}

for (let axi of axis) {
    chart[axi] = new Chart("MAG " + axi, {
        type : "scatter",
        data: {
            datasets: [
                {
                    label: 'expected',
                    borderColor: "rgba(0,0,0,1)",
                    pointBackgroundColor: "rgba(0,0,0,1)",
                    fill: false,
                    showLine: true,
                    parsing: { xAxisKey: 'time', yAxisKey: "expected_" + axi },
                    pointRadius: 0,
                },
                {
                    label: 'Compass 1', // Could pull devID's and report type
                    borderColor: "rgba(255,0,0,1)",
                    pointBackgroundColor: "rgba(255,0,0,1)",
                    fill: false,
                    showLine: true,
                    parsing: { xAxisKey: 'time', yAxisKey: axi },
                    pointRadius: 0,
                },
                {
                    label: 'Compass 2',
                    borderColor: "rgba(0,255,0,1)",
                    pointBackgroundColor: "rgba(0,255,0,1)",
                    fill: false,
                    showLine: true,
                    parsing: { xAxisKey: 'time', yAxisKey: axi },
                    pointRadius: 0,
                },
                {
                    label: 'Compass 3',
                    borderColor: "rgba(0,0,255,1.0)",
                    pointBackgroundColor: "rgba(0,0,255,1.0)",
                    fill: false,
                    showLine: true,
                    parsing: { xAxisKey: 'time', yAxisKey: axi },
                    pointRadius: 0,
                },
            ]
            },
            options: {
                aspectRatio: 8,
                scales: {
                    y: {
                        title: { display: true, text: "Field " + axi + " (mGauss)" },
                    },
                },
                plugins: {
                    legend: {
                        labels: {
                            usePointStyle: true,
                            pointStyle: 'rectRounded',
                            pointStyleWidth: 50,
                            filter: function(legendItem, data) {
                                return data.datasets[legendItem.datasetIndex].data.length > 0
                            }
                        },
                        onClick: null
                    },
                    tooltip: {
                        enabled: false
                    }
                }
            }
        })

    if (axi != 'X') {
        chart[axi].options.plugins.legend = null
    }
    chart[axi].update()
}

chart.Error = new Chart("Error", {
    type : "scatter",
    data: {
        datasets: [
            {
                label: 'Compass 1',
                borderColor: "rgba(255,0,0,1)",
                pointBackgroundColor: "rgba(255,0,0,1)",
                fill: false,
                showLine: true,
                parsing: { xAxisKey: 'time', yAxisKey: "Error" },
                pointRadius: 0,
            },
            {
                label: 'Compass 2',
                borderColor: "rgba(0,255,0,1)",
                pointBackgroundColor: "rgba(0,255,0,1)",
                fill: false,
                showLine: true,
                parsing: { xAxisKey: 'time', yAxisKey: "Error" },
                pointRadius: 0,
            },
            {
                label: 'Compass 3',
                borderColor: "rgba(0,0,255,1.0)",
                pointBackgroundColor: "rgba(0,0,255,1.0)",
                fill: false,
                showLine: true,
                parsing: { xAxisKey: 'time', yAxisKey: "Error" },
                pointRadius: 0,
            },
        ]
        },
        options: {
            aspectRatio: 8,
            scales: {
                y: {
                    title: { display: true, text: "Error" },
                },
            },
            plugins: {
                legend: {
                    labels: {
                        usePointStyle: true,
                        pointStyle: 'rectRounded',
                        pointStyleWidth: 50,
                        filter: function(legendItem, data) {
                            return data.datasets[legendItem.datasetIndex].data.length > 0
                        }
                    },
                    onClick: null
                },
                tooltip: {
                    enabled: false
                }
            }
        }
    })

chart.Interference = new Chart("Interference", {
    type : "scatter",
    data: {},
    options: {
        aspectRatio: 8,
        scales: {
            y: {
                title: { display: true, text: "Interference", },
                // hack to hide ticks but still have plot area align to plot above
                ticks: { color: "rgba(0,0,0,0)" }
            },
        },
        plugins: {
            legend: {
                labels: {
                    usePointStyle: true,
                    pointStyle: 'rectRounded',
                    pointStyleWidth: 50,
                    filter: function(legendItem, data) {
                        return data.datasets[legendItem.datasetIndex].data.length > 0
                    }
                },
            },
            tooltip: {
                enabled: false
            }
        }
    }
})

function add_Interference_dataset(data, key) {
    // Generate random color
    const r = Math.floor(Math.random() * 255)
    const g = Math.floor(Math.random() * 255)
    const b = Math.floor(Math.random() * 255)
    const color = `rgba(` + r + ',' + g + `,` + b + `,1)`

    // Plot
    chart.Interference.data.datasets.push({
        data: data,
        label: key,
        fill: false,
        showLine: true,
        parsing: { xAxisKey: 'time', yAxisKey: key },
        pointRadius: 0,
        yAxisID: key,
        borderColor: color,
        pointBackgroundColor: color
    })
    chart.Interference.options.scales[key] = { display: false, position: 'left' }
    chart.Interference.update()
}

function interpolate_log(des_time, log_time, log_values) {
    var log_length = log_time.length
    var ret = {}
    if (des_time < log_time[0]) {
        for (const key in log_values) {
            ret[key] = log_values[key][0]
        }

    } else if (des_time > log_time[log_length - 1]) {
        for (const key in log_values) {
            ret[key] = log_values[key][log_length - 1]
        }

    } else {
        for (let n = 0; n < log_length - 1; n++) {
            if ((des_time >= log_time[n]) && (des_time < log_time[n+1])) {
                const time_ratio = (des_time - log_time[n]) / (log_time[n+1] - log_time[n])
                for (const key in log_values) {
                    ret[key] = log_values[key][n] + time_ratio * (log_values[key][n+1] - log_values[key][n])
                }
                break
            }
        }
    }
    return ret
}

var log
var MAG_Data
var MAG_Data
var MAG_params_original
var trim
function load(log_file) {

    // Clear and reset state
    log = new DataflashParser()
    log.processData(log_file)

    MAG_Data = []
    MAG_Data = []
    MAG_params_original = []
    trim = {}

    // also need to reset plots...

    // Grab and plot MAG data
    log.parseAtOffset('MAG')

    var max = -Infinity
    var min = Infinity
    for (let i = 0; i < 3; i++) {
        var msg_name = 'MAG[' + i + ']'
        if (log.messages[msg_name] == null) {
            MAG_Data[i] = null
            continue
        }
        MAG_Data[i] = []
        for (let j = 0; j < log.messages[msg_name].time_boot_ms.length; j++) {
            MAG_Data[i][j] = { time:log.messages[msg_name].time_boot_ms[j], X:log.messages[msg_name].MagX[j], Y:log.messages[msg_name].MagY[j], Z:log.messages[msg_name].MagZ[j] }
        }
        for (let axi of axis) {
            chart[axi].data.datasets[i+1].data = MAG_Data[i]
        }
        min = Math.min(min, MAG_Data[i][0].time)
        max = Math.max(max, MAG_Data[i][MAG_Data[i].length-1].time)
    }
    for (let axi of axis) {
        chart[axi].options.scales.x.min = min
        chart[axi].options.scales.x.max = max
        chart[axi].update()
    }
    chart.Error.options.scales.x.min = min
    chart.Error.options.scales.x.max = max
    chart.Interference.options.scales.x.min = min
    chart.Interference.options.scales.x.max = max

    // Grab parameters
    log.parseAtOffset('PARM')

    for (let i = 0; i < 3; i++) {
        if (MAG_Data[i] == null) {
            continue
        }
        MAG_params_original[i] = new compass_params(i)
        MAG_params_original[i].load_from_log(log.messages['PARM'])
    }

    trim.X = get_param(log.messages['PARM'], "AHRS_TRIM_X")
    trim.Y = get_param(log.messages['PARM'], "AHRS_TRIM_Y")
    trim.Z = get_param(log.messages['PARM'], "AHRS_TRIM_Z")

    var EKF_TYPE = get_param(log.messages['PARM'], "AHRS_EKF_TYPE")
    var EKF3_PRIMARY = get_param(log.messages['PARM'], "EK3_PRIMARY")
    var COMP_TYPE = get_param(log.messages['PARM'], "COMPASS_MOTCT")

    if ((trim.X == null) || (trim.Y == null) || (trim.Z == null) || (EKF_TYPE == null) || (COMP_TYPE == null)) {
        console.log("Could not find params")
        return
    }

    // Use last EKF origin for earth field
    log.parseAtOffset("ORGN")
    if (log.messages["ORGN[0]"] == null) {
        console.log("Could not get EFK origin")
        return
    }
    var Lat = log.messages["ORGN[0]"].Lat[log.messages["ORGN[0]"].Lat.length-1] * 10**-7
    var Lng = log.messages["ORGN[0]"].Lng[log.messages["ORGN[0]"].Lng.length-1] * 10**-7
    var earth_field = expected_earth_field_lat_lon(Lat, Lng)
    if (earth_field == null) {
        console.log("Could not get earth field")
        return
    }
    console.log("EF: " + earth_field.x + ", " + earth_field.y + ", " + earth_field.z + " at Lat: " + Lat + " Lng: " + Lng)

    // Get vehicle attitude
    var att_msg = "ATT"

    // Note that this is not clever enough to deal with primary changing in flight
    if (EKF_TYPE == 2) {
        log.parseAtOffset("NKF1")
        att_msg = "NKF1[0]"
    } else if (EKF_TYPE == 3) {
        log.parseAtOffset("XKF1")
        var primary = 0
        if (EKF3_PRIMARY != null) {
            primary = EKF3_PRIMARY
        }
        att_msg = "XKF1[" + primary + "]"
    }

    if (log.messages[att_msg] == null) {
        console.log("Could not get attitude from: " + att_msg)
        return
    }

    // Rotate expected EF to body for each sample
    var rot = Matrix3()
    for (let i = 0; i < 3; i++) {
        if (MAG_Data[i] == null) {
            continue
        }
        for (let j = 0; j < MAG_Data[i].length; j++) {
            var interpolated = interpolate_log(MAG_Data[i][j].time, log.messages[att_msg].time_boot_ms, { roll:log.messages[att_msg].Roll, pitch:log.messages[att_msg].Pitch, yaw:log.messages[att_msg].Yaw } )

            // Save attitude for later use
            MAG_Data[i][j].roll = interpolated.roll * (Math.PI/180)
            MAG_Data[i][j].pitch = interpolated.pitch * (Math.PI/180)
            MAG_Data[i][j].yaw = interpolated.yaw * (Math.PI/180)

            // Rotate earth felid
            rot.from_euler_transposed(MAG_Data[i][j].roll, MAG_Data[i][j].pitch, MAG_Data[i][j].yaw)
            var expected_field = rot.mul(earth_field)
            MAG_Data[i][j].expected_X = expected_field.x
            MAG_Data[i][j].expected_Y = expected_field.y
            MAG_Data[i][j].expected_Z = expected_field.z
        }
    }

    // Plot expected, first compass only
    for (let axi of axis) {
        chart[axi].data.datasets[0].data = MAG_Data[0]
        chart[axi].update()
    }

    // Calculate and plot error
    for (let i = 0; i < 3; i++) {
        if (MAG_Data[i] == null) {
            continue
        }
        for (let j = 0; j < MAG_Data[i].length; j++) {
            MAG_Data[i][j].Error = Math.sqrt((MAG_Data[i][j].X - MAG_Data[i][j].expected_X)**2 + (MAG_Data[i][j].Y - MAG_Data[i][j].expected_Y)**2 + (MAG_Data[i][j].Z - MAG_Data[i][j].expected_Z)**2)
        }
        chart.Error.data.datasets[i].data = MAG_Data[i]
    }
    chart.Error.update()

    // Gather all sources of interference

    // FW throttle
    log.parseAtOffset("CTUN")
    if (log.messages["CTUN"] != null) {
        for (let i = 0; i < 3; i++) {
            if (MAG_Data[i] == null) {
                continue
            }
            for (let j = 0; j < MAG_Data[i].length; j++) {
                var interpolated = interpolate_log(MAG_Data[i][j].time, log.messages["CTUN"].time_boot_ms, { thr:log.messages["CTUN"].ThO } )
                MAG_Data[i][j].CTUN_throttle = interpolated.thr
            }
        }
        add_Interference_dataset(MAG_Data[0], "CTUN_throttle")
    }

    // VTOL throttle
    log.parseAtOffset("QTUN")
    if (log.messages["QTUN"] != null) {
        for (let i = 0; i < 3; i++) {
            if (MAG_Data[i] == null) {
                continue
            }
            for (let j = 0; j < MAG_Data[i].length; j++) {
                var interpolated = interpolate_log(MAG_Data[i][j].time, log.messages["QTUN"].time_boot_ms, { thr:log.messages["QTUN"].ThO } )
                MAG_Data[i][j].QTUN_throttle = interpolated.thr
            }
        }
        add_Interference_dataset(MAG_Data[0], "QTUN_throttle")
    }

    // Battery current and voltage
    log.parseAtOffset("BAT")
    for (let k = 0; k < 20; k++) {
        const batt_instance = "BAT[" + k + "]"
        if (log.messages[batt_instance] == null) {
            continue
        }
        for (let i = 0; i < 3; i++) {
            if (MAG_Data[i] == null) {
                continue
            }
            for (let j = 0; j < MAG_Data[i].length; j++) {
                var interpolated = interpolate_log(MAG_Data[i][j].time, log.messages[batt_instance].time_boot_ms, { Volt:log.messages[batt_instance].Volt, Curr:log.messages[batt_instance].Curr } )
                MAG_Data[i][j][batt_instance + "_Volt"] = interpolated.Volt
                MAG_Data[i][j][batt_instance + "_Curr"] = interpolated.Curr
            }
        }
        // Only add current, voltage is used in combination with throttle in some cases
        add_Interference_dataset(MAG_Data[0], batt_instance + "_Curr")
    }

    // Remove existing calibration to get raw values
    for (let i = 0; i < 3; i++) {
        if (MAG_Data[i] == null) {
            continue
        }
        for (let j = 0; j < MAG_Data[i].length; j++) {
            const raw = MAG_params_original[i].remove(MAG_Data[i][j])
            MAG_Data[i][j].raw_X = raw.x
            MAG_Data[i][j].raw_Y = raw.y
            MAG_Data[i][j].raw_Z = raw.z
            MAG_Data[i][j].raw_Error = Math.sqrt((MAG_Data[i][j].raw_X - MAG_Data[i][j].expected_X)**2 + (MAG_Data[i][j].raw_Y - MAG_Data[i][j].expected_Y)**2 + (MAG_Data[i][j].raw_Z - MAG_Data[i][j].expected_Z)**2)
        }
    }

    // Now calculation is done enable display selection buttons
    document.getElementById("mag_disp_exst").disabled = false;
    document.getElementById("mag_disp_raw").disabled = false;

}

function mag_display_change(src) {
    if (src.value == "Existing") {
        for (let i = 0; i < 3; i++) {
            chart.Error.data.datasets[i].parsing.yAxisKey = "Error"
            for (let axi of axis) {
                chart[axi].data.datasets[i+1].parsing.yAxisKey = axi
            }
        }

    } else if (src.value == "Raw") {
        for (let i = 0; i < 3; i++) {
            chart.Error.data.datasets[i].parsing.yAxisKey = "raw_Error"
            for (let axi of axis) {
                chart[axi].data.datasets[i+1].parsing.yAxisKey = "raw_" + axi
            }
        }
    }

    for (let axi of axis) {
        chart[axi].update()
    }
    chart.Error.update()

}