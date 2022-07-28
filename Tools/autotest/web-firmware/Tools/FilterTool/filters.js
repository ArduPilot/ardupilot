function calc_lowpass_alpha_dt(dt, cutoff_freq)
{
    if (dt <= 0.0 || cutoff_freq <= 0.0) {
        return 1.0;
    }
    var rc = 1.0/(Math.PI*2*cutoff_freq);
    return dt/(dt+rc);
}

function PID(sample_rate,kP,kI,kD,filtE,filtD) {

    this._kP = kP;
    this._kI = kI;
    this._kD = kD;

    this._dt = 1.0/sample_rate;
    this.E_alpha = calc_lowpass_alpha_dt(this._dt,filtE)
    this.D_alpha = calc_lowpass_alpha_dt(this._dt,filtD)

    this._error = 0.0;
    this._derivative = 0.0;
    this._integrator = 0.0;

    this.reset = function(sample) {
        this._error = 0.0;
        this._derivative = 0.0;
        this._integrator = 0.0;
    }

    this.apply = function(error) {

        var error_last = this._error;
        this._error += this.E_alpha * (error - this._error);

        var derivative = (this._error - error_last) / this._dt;
        this._derivative += this.D_alpha * (derivative - this._derivative)

        this._integrator += this._error * this._kI * this._dt;

        var P_out = this._error * this._kP;
        var D_out = this._derivative * this._kD;

        return P_out + this._integrator + D_out;
    }
    return this;
}

function LPF_1P(sample_rate,cutoff) {
    this.reset = function(sample) {
        this.value = sample;
    }
    if (cutoff <= 0) {
        this.apply = function(sample) {
            return sample;
        }
        return this;
    }
    this.alpha = calc_lowpass_alpha_dt(1.0/sample_rate,cutoff)
    this.value = 0.0;
    this.apply = function(sample) {
        this.value += this.alpha * (sample - this.value);
        return this.value;
    }
    return this;
}

function DigitalBiquadFilter(sample_freq, cutoff_freq) {
    this.delay_element_1 = 0;
    this.delay_element_2 = 0;
    this.cutoff_freq = cutoff_freq;

    if (cutoff_freq <= 0) {
        // zero cutoff means pass-thru
        this.reset = function(sample) {
        }
        this.apply = function(sample) {
            return sample;
        }
        return this;
    }

    var fr = sample_freq/cutoff_freq;
    var ohm = Math.tan(Math.PI/fr);
    var c = 1.0+2.0*Math.cos(Math.PI/4.0)*ohm + ohm*ohm;

    this.b0 = ohm*ohm/c;
    this.b1 = 2.0*this.b0;
    this.b2 = this.b0;
    this.a1 = 2.0*(ohm*ohm-1.0)/c;
    this.a2 = (1.0-2.0*Math.cos(Math.PI/4.0)*ohm+ohm*ohm)/c;
    this.initialised = false;

    this.apply = function(sample) {
        if (!this.initialised) {
            this.reset(sample);
            this.initialised = true;
        }
        var delay_element_0 = sample - this.delay_element_1 * this.a1 - this.delay_element_2 * this.a2;
        var output = delay_element_0 * this.b0 + this.delay_element_1 * this.b1 + this.delay_element_2 * this.b2;

        this.delay_element_2 = this.delay_element_1;
        this.delay_element_1 = delay_element_0;
        return output;
    }

    this.reset = function(sample) {
        this.delay_element_1 = this.delay_element_2 = sample * (1.0 / (1 + this.a1 + this.a2));
    }
    
    return this;
}

function sq(v) {
    return v*v;
}

function constrain_float(v,vmin,vmax) {
    if (v < vmin) {
        return vmin;
    }
    if (v > vmax) {
        return vmax;
    }
    return v;
}

function NotchFilter(sample_freq,center_freq_hz,bandwidth_hz,attenuation_dB) {
    this.sample_freq = sample_freq;
    this.center_freq_hz = center_freq_hz;
    this.bandwidth_hz = bandwidth_hz;
    this.attenuation_dB = attenuation_dB;
    this.need_reset = true;
    this.initialised = false;

    this.calculate_A_and_Q = function() {
        this.A = Math.pow(10.0, -this.attenuation_dB / 40.0);
        if (this.center_freq_hz > 0.5 * this.bandwidth_hz) {
            var octaves = Math.log2(this.center_freq_hz / (this.center_freq_hz - this.bandwidth_hz / 2.0)) * 2.0;
            this.Q = Math.sqrt(Math.pow(2.0, octaves)) / (Math.pow(2.0, octaves) - 1.0);
        } else {
            this.Q = 0.0;
        }
    }

    this.init_with_A_and_Q = function() {
        if ((this.center_freq_hz > 0.0) && (this.center_freq_hz < 0.5 * this.sample_freq) && (this.Q > 0.0)) {
            var omega = 2.0 * Math.PI * this.center_freq_hz / this.sample_freq;
            var alpha = Math.sin(omega) / (2 * this.Q);
            this.b0 =  1.0 + alpha*sq(this.A);
            this.b1 = -2.0 * Math.cos(omega);
            this.b2 =  1.0 - alpha*sq(this.A);
            this.a0_inv =  1.0/(1.0 + alpha);
            this.a1 = this.b1;
            this.a2 =  1.0 - alpha;
            this.initialised = true;
        } else {
            this.initialised = false;
        }
    }

    // check center frequency is in the allowable range
    if ((center_freq_hz > 0.5 * bandwidth_hz) && (center_freq_hz < 0.5 * sample_freq)) {
        this.calculate_A_and_Q();
        this.init_with_A_and_Q();
    } else {
        this.initialised = false;
    }

    this.apply = function(sample) {
        if (!this.initialised || this.need_reset) {
            // if we have not been initialised when return the input
            // sample as output and update delayed samples
            this.signal1 = sample;
            this.signal2 = sample;
            this.ntchsig = sample;
            this.ntchsig1 = sample;
            this.ntchsig2 = sample;
            this.need_reset = false;
            return sample;
        }
        this.ntchsig2 = this.ntchsig1;
        this.ntchsig1 = this.ntchsig;
        this.ntchsig = sample;
        var output = (this.ntchsig*this.b0 + this.ntchsig1*this.b1 + this.ntchsig2*this.b2 - this.signal1*this.a1 - this.signal2*this.a2) * this.a0_inv;
        this.signal2 = this.signal1;
        this.signal1 = output;
        return output;
    }

    this.reset = function(sample) {
        this.need_reset = true;
        this.apply(sample);
    }

    return this;
}

function HarmonicNotchFilter(sample_freq,enable,mode,freq,bw,att,ref,fm_rat,hmncs,opts) {
    this.notches = []
    var chained = 1;
    var dbl = false;
    var triple = false;
    var composite_notches = 1;
    if (opts & 1) {
        dbl = true;
        composite_notches = 2;
    } else if (opts & 16) {
        triple = true;
        composite_notches = 3;
    }

    this.reset = function(sample) {
        for (n in this.notches) {
            this.notches[n].reset(sample);
        }
    }

    if (enable <= 0) {
        this.apply = function(sample) {
            return sample;
        }
        return this;
    }

    if (mode == 0) {
        // fixed notch
    }
    if (mode == 1) {
        var motors_throttle = Math.max(0,get_form("Throttle"));
        var throttle_freq = freq * Math.max(fm_rat,Math.sqrt(motors_throttle / ref));
        freq = throttle_freq;
    }
    if (mode == 2) {
        var rpm = get_form("RPM1");
        freq = Math.max(rpm/60.0,freq) * ref;
    }
    if (mode == 5) {
        var rpm = get_form("RPM2");
        freq = Math.max(rpm/60.0,freq) * ref;
    }
    if (mode == 3) {
        if (opts & 2) {
            chained = get_form("NUM_MOTORS");
        }
        var rpm = get_form("ESC_RPM");
        freq = Math.max(rpm/60.0,freq) * ref;
    }
    for (var n=0;n<8;n++) {
        var fmul = n+1;
        if (hmncs & (1<<n)) {
            var notch_center = freq * fmul;
            var bandwidth_hz = bw * fmul;
            for (var c=0; c<chained; c++) {
                var nyquist_limit = sample_freq * 0.48;
                var bandwidth_limit = bandwidth_hz * 0.52;

                // Calculate spread required to achieve an equivalent single notch using two notches with Bandwidth/2
                var notch_spread = bandwidth_hz / (32.0 * notch_center);

                // adjust the fundamental center frequency to be in the allowable range
                notch_center = constrain_float(notch_center, bandwidth_limit, nyquist_limit);

                if (composite_notches != 2) {
                    // only enable the filter if its center frequency is below the nyquist frequency
                    if (notch_center < nyquist_limit) {
                        this.notches.push(new NotchFilter(sample_freq,notch_center,bandwidth_hz/composite_notches,att));
                    }
                }
                if (composite_notches > 1) {
                    var notch_center_double;
                    // only enable the filter if its center frequency is below the nyquist frequency
                    notch_center_double = notch_center * (1.0 - notch_spread);
                    if (notch_center_double < nyquist_limit) {
                        this.notches.push(new NotchFilter(sample_freq,notch_center_double,bandwidth_hz/composite_notches,att));
                    }
                    // only enable the filter if its center frequency is below the nyquist frequency
                    notch_center_double = notch_center * (1.0 + notch_spread);
                    if (notch_center_double < nyquist_limit) {
                        this.notches.push(new NotchFilter(sample_freq,notch_center_double,bandwidth_hz/composite_notches,att));
                    }
                }
            }
        }
    }
    this.apply = function(sample) {
        for (n in this.notches) {
            sample = this.notches[n].apply(sample);
        }
        return sample;
    }
}

function get_form(vname) {
    var v = parseFloat(document.getElementById(vname).value);
    setCookie(vname, v);
    return v;
}

function run_filters(filters,freq,sample_rate,samples,fast_filters = null,fast_sample_rate = null) {

    for (var j=0;j<filters.length; j++) {
        filters[j].reset(0.0);
    }

    var num_best_fit_points = 100;
    var best_fit_offset = samples - num_best_fit_points;

    // Best fit to sin to get amplitude, phase and DC offset
    // https://math.stackexchange.com/questions/3926007/least-squares-regression-of-sine-wave
    // Expecting output to be of same frequency at input
    // Z = a*sin(t*kt + p) + O
    // A = a*cos(p)
    // B = a*sin(p)
    // S = sin(t*kt)
    // C = cos(t*kt)

    var X = ML.MatrixLib.Matrix.ones(num_best_fit_points, 3);
    var y = ML.MatrixLib.Matrix.zeros(num_best_fit_points, 1);

    var kt =  Math.PI * 2.0 * freq;
    var fast_filter_t = 0;
    var fast_dt = 1.0 / fast_sample_rate;
    for (var i=-0;i<samples;i++) {
        var t = i / sample_rate;

        // advance faster filters if any
        if (fast_filters && fast_sample_rate) {
            do {
                // run filters upto current time
                var output = Math.sin(fast_filter_t * kt);
                for (var j=0;j<fast_filters.length; j++) {
                    output = fast_filters[j].apply(output);
                }
                fast_filter_t += fast_dt;
            } while ((fast_filter_t + fast_dt) <= t)
            var input = output
        } else {
            var input = Math.sin(t * kt);
        }

        var output = input;
        for (var j=0;j<filters.length; j++) {
            output = filters[j].apply(output);
        }
        if (i >= best_fit_offset) {
            var index = i - best_fit_offset;
            X.data[index][0] = Math.sin(t * kt);
            X.data[index][1] = Math.cos(t * kt);
            y.data[index][0] = output;
        }
    }

    // Z = a*sin(t*kt + p) + O
    var ABO = ML.MatrixLib.solve(X, y);

    var amplitude = Math.sqrt(ABO.get(0,0)*ABO.get(0,0) + ABO.get(1,0)*ABO.get(1,0));
    var phase = Math.atan2(ABO.get(1,0),ABO.get(0,0)) * (-180.0 / Math.PI);
    // var DC_offset = ABO.get(2,0);

    return [amplitude,phase];
}

var chart;
var freq_log_scale;

function get_filters(sample_rate) {
    var filters = []
    filters.push(new HarmonicNotchFilter(sample_rate,
                                         get_form("INS_HNTCH_ENABLE"),
                                         get_form("INS_HNTCH_MODE"),
                                         get_form("INS_HNTCH_FREQ"),
                                         get_form("INS_HNTCH_BW"),
                                         get_form("INS_HNTCH_ATT"),
                                         get_form("INS_HNTCH_REF"),
                                         get_form("INS_HNTCH_FM_RAT"),
                                         get_form("INS_HNTCH_HMNCS"),
                                         get_form("INS_HNTCH_OPTS")));
    filters.push(new HarmonicNotchFilter(sample_rate,
                                         get_form("INS_HNTC2_ENABLE"),
                                         get_form("INS_HNTC2_MODE"),
                                         get_form("INS_HNTC2_FREQ"),
                                         get_form("INS_HNTC2_BW"),
                                         get_form("INS_HNTC2_ATT"),
                                         get_form("INS_HNTC2_REF"),
                                         get_form("INS_HNTC2_FM_RAT"),
                                         get_form("INS_HNTC2_HMNCS"),
                                         get_form("INS_HNTC2_OPTS")));
    filters.push(new DigitalBiquadFilter(sample_rate,get_form("INS_GYRO_FILTER")));

    return filters;
}

function calculate_filter() {
    var sample_rate = get_form("GyroSampleRate");
    var freq_max = get_form("MaxFreq");
    var samples = 1000;
    var freq_step = 0.1;
    var filters = get_filters(sample_rate);

    var use_dB = document.getElementById("ScaleLog").checked;
    setCookie("Scale", use_dB ? "Log" : "Linear");
    var use_RPM = document.getElementById("freq_Scale_RPM").checked;
    setCookie("feq_unit", use_RPM ? "RPM" : "Hz");
    var attenuation = []
    var phase_lag = []
    var min_phase_lag = 0.0;
    var max_phase_lag = 0.0;
    var phase_wrap = 0.0;
    var min_atten = 0.0;
    var max_atten = 1.0;
    var last_phase = 0.0;
    var atten_string = "Magnitude";
    if (use_dB) {
        max_atten = 0;
        min_atten = -10;
        atten_string = "Magnitude (dB)";
    }
    var freq_string = "Frequency (Hz)";
    if (use_RPM) {
        freq_string = "Frequency (RPM)";
    }

    // start at zero
    attenuation.push({x:0, y:1});
    phase_lag.push({x:0, y:0});

    for (freq=freq_step; freq<=freq_max; freq+=freq_step) {
        var v = run_filters(filters, freq, sample_rate, samples);
        var aten = v[0];
        var phase = v[1] + phase_wrap;
        if (use_dB) {
            // show power in decibels
            aten = 20 * Math.log10(aten);
        }
        var freq_value = freq;
        if (use_RPM) {
            freq_value *= 60;
        }
        var phase_diff = phase - last_phase;
        if (phase_diff > 180) {
            phase_wrap -= 360.0;
            phase -= 360.0;
        } else if (phase_diff < -180) {
            phase_wrap += 360.0;
            phase += 360.0;
        }
        attenuation.push({x:freq_value, y:aten});
        phase_lag.push({x:freq_value, y:-phase});

        min_atten = Math.min(min_atten, aten);
        max_atten = Math.max(max_atten, aten);
        min_phase_lag = Math.min(min_phase_lag, phase)
        max_phase_lag = Math.max(max_phase_lag, phase)
        last_phase = phase;
    }
    min_phase_lag = Math.floor((min_phase_lag-10)/10)*10;
    min_phase_lag = Math.min(Math.max(-get_form("MaxPhaseLag"), min_phase_lag), 0);
    max_phase_lag = Math.ceil((max_phase_lag+10)/10)*10;
    max_phase_lag = Math.min(get_form("MaxPhaseLag"), max_phase_lag);

    if (use_RPM) {
        freq_max *= 60.0;
    }

    var freq_log = document.getElementById("freq_ScaleLog").checked;
    setCookie("feq_scale", freq_log ? "Log" : "Linear");
    if ((freq_log_scale != null) && (freq_log_scale != freq_log)) {
        // Scale changed, no easy way to update, delete chart and re-draw
        chart.clear();
        chart.destroy();
        chart = null;
    }
    freq_log_scale = freq_log;

    if (chart) {
        chart.data.datasets[0].data = attenuation;
        chart.data.datasets[1].data = phase_lag;
        chart.options.scales.xAxes[0].ticks.max = freq_max;
        chart.options.scales.xAxes[0].scaleLabel.labelString = freq_string
        chart.options.scales.yAxes[0].ticks.min = min_atten
        chart.options.scales.yAxes[0].ticks.max = max_atten;
        chart.options.scales.yAxes[0].scaleLabel.labelString = atten_string;
        chart.options.scales.yAxes[1].ticks.min = -max_phase_lag;
        chart.options.scales.yAxes[1].ticks.max = -min_phase_lag;
        chart.update();
    } else {
        chart = new Chart("Attenuation", {
            type : "scatter",
            data: {
                datasets: [
                    {
                        label: 'Magnitude',
                        yAxisID: 'Magnitude',
                        pointRadius: 0,
                        hitRadius: 8,
                        borderColor: "rgba(0,0,255,1.0)",
                        pointBackgroundColor: "rgba(0,0,255,1.0)",
                        data: attenuation,
                        showLine: true,
                        fill: false
                    },
                    {
                        label: 'Phase',
                        yAxisID: 'Phase',
                        pointRadius: 0,
                        hitRadius: 8,
                        borderColor: "rgba(255,0,0,1.0)",
                        pointBackgroundColor: "rgba(255,0,0,1.0)",
                        data: phase_lag,
                        showLine: true,
                        fill: false
                    }
                ]
            },
            options: {
                legend: {display: true},
                scales: {
                    yAxes: [
                        {
                            scaleLabel: { display: true, labelString: atten_string },
                            id: 'Magnitude',
                            position: 'left',
                            ticks: {min:min_atten, max:max_atten}
                        },
                        {
                            scaleLabel: { display: true, labelString: "Phase (deg)" },
                            id: 'Phase',
                            position: 'right',
                            gridLines: {display:false},
                            ticks: {min:-max_phase_lag, max:-min_phase_lag}
                        }
                    ],
                    xAxes: [
                        {
                            type: (freq_log ? "logarithmic" : "linear"),
                            scaleLabel: { display: true, labelString: freq_string },
                            ticks:
                            {
                                min:0.0,
                                max:freq_max,
                                callback: function(value, index, ticks) {
                                    return value;
                                },
                            }
                        }
                    ],
                },
                tooltips: {
                    callbacks: {
                        label: function(tooltipItem, data) {
                            // round tooltip to two decimal places
                            return tooltipItem.xLabel.toFixed(2) + ', ' + tooltipItem.yLabel.toFixed(2);
                        }
                    }
                 }
            }
        });
    }
}

var PID_chart;
var PID_freq_log_scale;

function calculate_pid(axis_id) {
    //var sample_rate = get_form("GyroSampleRate");
    var PID_rate = get_form("SCHED_LOOP_RATE")
    var filters = []
    var freq_max = get_form("PID_MaxFreq");
    var samples = 1000;
    var freq_step = 0.1;

    // default to roll axis
    var axis_prefix = "ATC_RAT_RLL_";
    if (axis_id ==  "CalculatePitch") {
        var axis_prefix = "ATC_RAT_PIT_";
        document.getElementById("PID_title").innerHTML = "Pitch axis";
    } else if (axis_id ==  "CalculateYaw") {
        var axis_prefix = "ATC_RAT_YAW_";
        document.getElementById("PID_title").innerHTML = "Yaw axis";
    } else {
        document.getElementById("PID_title").innerHTML = "Roll axis";
    }

    filters.push(new PID(PID_rate,
                        get_form(axis_prefix + "P"),
                        get_form(axis_prefix + "I"),
                        get_form(axis_prefix + "D"),
                        get_form(axis_prefix + "FLTE"),
                        get_form(axis_prefix + "FLTD")));

    var use_dB = document.getElementById("PID_ScaleLog").checked;
    setCookie("PID_Scale", use_dB ? "Log" : "Linear");
    var use_RPM =  document.getElementById("PID_freq_Scale_RPM").checked;
    setCookie("PID_feq_unit", use_RPM ? "RPM" : "Hz");
    var attenuation = []
    var phase_lag = []
    var min_phase_lag = 0.0;
    var max_phase_lag = 0.0;
    var phase_wrap = 0.0;
    var min_atten = Infinity;
    var max_atten = -Infinity;
    var last_phase = 0.0;
    var atten_string = "Gain";
    if (use_dB) {
        max_atten = 0;
        min_atten = -10;
        atten_string = "Gain (dB)";
    }
    var freq_string = "Frequency (Hz)";
    if (use_RPM) {
        freq_string = "Frequency (RPM)";
    }

    var fast_filters = null;
    var fast_sample_rate = null;
    if (document.getElementById("PID_filtering_Post").checked) {
        fast_sample_rate = get_form("GyroSampleRate");
        fast_filters = get_filters(fast_sample_rate);
    }
    setCookie("filtering", fast_filters == null ? "Pre" : "Post");


    for (freq=freq_step; freq<=freq_max; freq+=freq_step) {
        var v = run_filters(filters, freq, PID_rate, samples, fast_filters, fast_sample_rate);
        var aten = v[0];
        var phase = v[1] + phase_wrap;
        if (use_dB) {
            // show power in decibels
            aten = 20 * Math.log10(aten);
        }
        var freq_value = freq;
        if (use_RPM) {
            freq_value *= 60;
        }
        var phase_diff = phase - last_phase;
        if (phase_diff > 180) {
            phase_wrap -= 360.0;
            phase -= 360.0;
        } else if (phase_diff < -180) {
            phase_wrap += 360.0;
            phase += 360.0;
        }
        attenuation.push({x:freq_value, y:aten});
        phase_lag.push({x:freq_value, y:-phase});

        min_atten = Math.min(min_atten, aten);
        max_atten = Math.max(max_atten, aten);
        min_phase_lag = Math.min(min_phase_lag, phase)
        max_phase_lag = Math.max(max_phase_lag, phase)
        last_phase = phase;
    }

    if (use_RPM) {
        freq_max *= 60.0;
    }

    var mean_atten = (min_atten + max_atten) * 0.5;
    var atten_range = Math.max((max_atten - min_atten) * 0.5 * 1.1, 1.0);
    min_atten = mean_atten - atten_range;
    max_atten = mean_atten + atten_range;

    min_phase_lag = Math.floor((min_phase_lag-10)/10)*10;
    min_phase_lag = Math.min(Math.max(-get_form("PID_MaxPhaseLag"), min_phase_lag), 0);
    max_phase_lag = Math.ceil((max_phase_lag+10)/10)*10;
    max_phase_lag = Math.min(get_form("PID_MaxPhaseLag"), max_phase_lag);

    var freq_log = document.getElementById("PID_freq_ScaleLog").checked;
    setCookie("PID_feq_scale", use_dB ? "Log" : "Linear");
    if ((PID_freq_log_scale != null) && (PID_freq_log_scale != freq_log)) {
        // Scale changed, no easy way to update, delete chart and re-draw
        PID_chart.clear();
        PID_chart.destroy();
        PID_chart = null;
    }
    PID_freq_log_scale = freq_log;

    if (PID_chart) {
        PID_chart.data.datasets[0].data = attenuation;
        PID_chart.data.datasets[1].data = phase_lag;
        PID_chart.options.scales.xAxes[0].ticks.max = freq_max;
        PID_chart.options.scales.xAxes[0].scaleLabel.labelString = freq_string
        PID_chart.options.scales.yAxes[0].ticks.min = min_atten
        PID_chart.options.scales.yAxes[0].ticks.max = max_atten;
        PID_chart.options.scales.yAxes[0].scaleLabel.labelString = atten_string;
        PID_chart.options.scales.yAxes[1].ticks.min = -max_phase_lag;
        PID_chart.options.scales.yAxes[1].ticks.max = -min_phase_lag;
        PID_chart.update();
    } else {
        PID_chart = new Chart("PID_Attenuation", {
            type : "scatter",
            data: {
                datasets: [
                    {
                        label: 'Gain',
                        yAxisID: 'Gain',
                        pointRadius: 0,
                        hitRadius: 8,
                        borderColor: "rgba(0,0,255,1.0)",
                        pointBackgroundColor: "rgba(0,0,255,1.0)",
                        data: attenuation,
                        showLine: true,
                        fill: false
                    },
                    {
                        label: 'PhaseLag',
                        yAxisID: 'PhaseLag',
                        pointRadius: 0,
                        hitRadius: 8,
                        borderColor: "rgba(255,0,0,1.0)",
                        pointBackgroundColor: "rgba(255,0,0,1.0)",
                        data: phase_lag,
                        showLine: true,
                        fill: false
                    }
                ]
            },
            options: {
                legend: {display: true},
                scales: {
                    yAxes: [
                        {
                            scaleLabel: { display: true, labelString: atten_string },
                            id: 'Gain',
                            position: 'left',
                            ticks: {min:min_atten, max:max_atten}
                        },
                        {
                            scaleLabel: { display: true, labelString: "Phase (deg)" },
                            id: 'PhaseLag',
                            position: 'right',
                            gridLines: {display:false},
                            ticks: {min:-max_phase_lag, max:-min_phase_lag}
                        }
                    ],
                    xAxes: [
                        {
                            type: (freq_log ? "logarithmic" : "linear"),
                            scaleLabel: { display: true, labelString: freq_string },
                            ticks:
                            {
                                min:0.0,
                                max:freq_max,
                                callback: function(value, index, ticks) {
                                    return value;
                                },
                            }
                        }
                    ],
                },
                tooltips: {
                    callbacks: {
                        label: function(tooltipItem, data) {
                            // round tooltip to two decimal places
                            return tooltipItem.xLabel.toFixed(2) + ', ' + tooltipItem.yLabel.toFixed(2);
                        }
                    }
                }
            }
        });
    }
}

function setCookie(c_name, value) {
    var exdate = new Date();
    var exdays = 365;
    exdate.setDate(exdate.getDate() + exdays);
    var c_value = escape(value) + ";expires=" + exdate.toUTCString();
    document.cookie = c_name + "=" + c_value + ";path=/";
}

function getCookie(c_name, def_value) {
    let name = c_name + "=";
    let decodedCookie = decodeURIComponent(document.cookie);
    let ca = decodedCookie.split(';');
    for(let i = 0; i <ca.length; i++) {
        let c = ca[i];
        while (c.charAt(0) == ' ') {
            c = c.substring(1);
        }
        if (c.indexOf(name) == 0) {
            return c.substring(name.length, c.length);
        }
    }
    return def_value;
}

function load_cookies() {
    var sections = ["params", "PID_params"];
    for (var i = 0; i < sections.length; i++) {
        var inputs = document.forms[sections[i]].getElementsByTagName("input");
        for (const v in inputs) {
            var name = inputs[v].name;
            if (inputs[v].type == "radio") {
                // only checked buttons are included
                if (inputs[v].value == getCookie(name)) {
                    inputs[v].checked = true;
                }
                continue;
            }
            inputs[v].value = parseFloat(getCookie(name,inputs[v].value));
        }
    }
}

function clear_cookies() {
    var cookies = document.cookie.split(";");
    for (var i = 0; i < cookies.length; i++) {
        var cookie = cookies[i];
        var eqPos = cookie.indexOf("=");
        var name = eqPos > -1 ? cookie.substr(0, eqPos) : cookie;
        document.cookie = name + "=;expires=Thu, 01 Jan 1970 00:00:00 GMT";
    }
}

function save_parameters() {
    var params = "";
    var inputs = document.forms["params"].getElementsByTagName("input");
    for (const v in inputs) {
        var name = "" + inputs[v].name;
        if (name.startsWith("INS_")) {
            var value = inputs[v].value;
            params += name + "=" + value + "\n";
        }
    }
    var blob = new Blob([params], { type: "text/plain;charset=utf-8" });
    saveAs(blob, "filter.param");
}

async function load_parameters(file) {
    var text = await file.text();
    var lines = text.split('\n');
    for (i in lines) {
        var line = lines[i];
        line = line.replace("Q_A_RAT_","ATC_RAT_");
        v = line.split(/[\s,=\t]+/);
        if (v.length >= 2) {
            var vname = v[0];
            var value = v[1];
            var fvar = document.getElementById(vname);
            if (fvar) {
                fvar.value = value;
                console.log("set " + vname + "=" + value);
            }
        }
    }
    fill_docs();
    update_all_hidden();
    calculate_filter();
}

function fill_docs()
{
    var inputs = document.forms["params"].getElementsByTagName("input");
    for (const v in inputs) {
        var name = inputs[v].name;
        var doc = document.getElementById(name + ".doc");
        if (!doc) {
            continue;
        }
        if (inputs[v].onchange == null) {
            inputs[v].onchange = fill_docs;
        }
        var value = parseFloat(inputs[v].value);
        if (name.endsWith("_ENABLE")) {
            if (value >= 1) {
                doc.innerHTML = "Enabled";
            } else {
                doc.innerHTML = "Disabled";
            }
        } else if (name.endsWith("_MODE")) {
            switch (Math.floor(value)) {
            case 0:
                doc.innerHTML = "Fixed notch";
                break;
            case 1:
                doc.innerHTML = "Throttle";
                break;
            case 2:
                doc.innerHTML = "RPM Sensor 1";
                break;
            case 3:
                doc.innerHTML = "ESC Telemetry";
                break;
            case 4:
                doc.innerHTML = "Dynamic FFT";
                break;
            case 5:
                doc.innerHTML = "RPM Sensor 2";
                break;
            default:
                doc.innerHTML = "INVALID";
                break;
            }
        } else if (name.endsWith("_OPTS")) {
            var ival = Math.floor(value);
            var bits = [];
            if (ival & 1) {
                bits.push("Double Notch");
            }
            if (ival & 2) {
                bits.push("Dynamic Harmonic");
            }
            if (ival & 4) {
                bits.push("Loop Rate");
            }
            if (ival & 8) {
                bits.push("All IMUs Rate");
            }
            if ((ival & 16) && (ival & 1) == 0) {
                bits.push("Triple Notch");
            }
            doc.innerHTML = bits.join(", ");
        } else if (name.endsWith("_HMNCS")) {
            var ival = Math.floor(value);
            var bits = [];
            if (ival & 1) {
                bits.push("Fundamental");
            }
            if (ival & 2) {
                bits.push("1st Harmonic");
            }
            if (ival & 4) {
                bits.push("2nd Harmonic");
            }
            if (ival & 8) {
                bits.push("3rd Harmonic");
            }
            if (ival & 16) {
                bits.push("4th Harmonic");
            }
            if (ival & 32) {
                bits.push("5th Harmonic");
            }
            if (ival & 64) {
                bits.push("6th Harmonic");
            }
            doc.innerHTML = bits.join(", ");
        }

    }
}

// update all hidden params, to be called at init
function update_all_hidden()
{
    var enable_params = ["INS_HNTCH_ENABLE", "INS_HNTC2_ENABLE"];
    for (var i=-0;i<enable_params.length;i++) {
        update_hidden(enable_params[i])
    }
}

// update hidden inputs based on param value
function update_hidden(enable_param)
{
    var enabled = parseFloat(document.getElementById(enable_param).value) > 0;
    var prefix = enable_param.split("_ENABLE")[0];

    // find all elements with same prefix
    var inputs = document.forms["params"].getElementsByTagName("*");
    for (var i=-0;i<inputs.length;i++) {
        var key = inputs[i].id;
        if (key.length == 0) {
            // no id, but bound to a valid one
            if (inputs[i].htmlFor == null) {
                continue;
            }
            key = inputs[i].htmlFor
        }
        if (key.startsWith(enable_param)) {
            // found original param, don't change
            continue;
        }
        if (key.startsWith(prefix)) {
            inputs[i].hidden = !enabled;
        }
    }

    update_hidden_mode();
}

function update_hidden_mode()
{
    var mode_params = ["INS_HNTCH_MODE", "INS_HNTC2_MODE"];
    var mode_options = [[[1], "Throttle_input"], [[3], "ESC_input"], [[2,5], "RPM_input"]];

    for (var i =0; i < mode_options.length; i++) {
        var hide = true;
        for (var j =0; j < mode_params.length; j++) {
            // check enable param
            if (!(parseFloat(document.getElementById(mode_params[j].replace("MODE","ENABLE")).value) > 0)) {
                continue;
            }

            var mode = Math.floor(get_form(mode_params[j]))
            for (var k =0; k < mode_options[i][0].length; k++) {
                if (mode == mode_options[i][0][k]) {
                    hide = false;
                }
            }
        }
        document.getElementById(mode_options[i][1]).hidden = hide;
    }
}

function check_nyquist()
{
    var checks = [["GyroSampleRate", "MaxFreq", "MaxFreq_warning"],
                  ["SCHED_LOOP_RATE", "PID_MaxFreq", "PID_MaxFreq_warning"]];

    for (var i = 0; i < checks.length; i++) {
        var freq_limit = get_form(checks[i][0]) * 0.5;
        var sample_rate = document.getElementById(checks[i][1]);
        if (parseFloat(sample_rate.value) > freq_limit) {
            sample_rate.value = freq_limit;
            document.getElementById(checks[i][2]).innerHTML = "Nyquist limit of half sample rate";
        } else {
            document.getElementById(checks[i][2]).innerHTML = "";
        }
    }
}
