function calc_lowpass_alpha_dt(dt, cutoff_freq)
{
    if (dt <= 0.0 || cutoff_freq <= 0.0) {
        return 1.0;
    }
    var rc = 1.0/(Math.PI*2*cutoff_freq);
    return dt/(dt+rc);
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

function run_filters(filters,freq,sample_rate,samples) {
    var integral_in = 0.0;
    var integral_out = 0.0;
    var crossing_lag = 0;
    var crossing_count = 0;
    var last_out = 0.0;
    var last_t = 0.0;

    for (var j=0;j<filters.length; j++) {
        filters[j].reset(0.0);
    }

    var period = 1 / freq;
    // start integration at the closest multiple of period then
    // offset to get whole number of periods before end
    // (start and end at the same phase)
    // could use the lag estimate to further offset the filter integration time
    var int_start_time = Math.round(((0.1*samples)/sample_rate) / period) * period;
    var end_offset = (samples/sample_rate) % period;
    int_start_time += end_offset;

    for (var i=-0;i<samples;i++) {
        var t = i / sample_rate;
        var input = Math.sin(t * Math.PI * 2.0 * freq);
        var output = input;
        for (var j=0;j<filters.length; j++) {
            output = filters[j].apply(output);
        }
        if (t > int_start_time) {
            // RMS amplitude of input and output to calculate attenuation
            integral_in += Math.pow(input,2);
            integral_out += Math.pow(output,2);
        }
        if (output >= 0 && last_out < 0) {
            // positive going zero crossing, latest value is best estimate
            crossing_count ++;
            crossing_lag = linear_interpolate(last_t,t,0,last_out,output) - period*crossing_count;
        }
        last_out = output;
        last_t = t;
    }
    var ratio = Math.sqrt(integral_out)/Math.sqrt(integral_in);
    var lag = 360.0 * crossing_lag * freq;

    // wrap to +- 180
    while (lag > 180.0) {
        lag -= 360.0;
    }
    while (lag < -180.0) {
        lag += 360.0;
    }
    return [ratio,lag];
}

function linear_interpolate(low_output, high_output, var_value, var_low, var_high) {
    var p = (var_value - var_low) / (var_high - var_low);
    return low_output + p * (high_output - low_output);
}

var chart;

function calculate_filter() {
    var sample_rate = get_form("GyroSampleRate");
    var filters = []
    var freq_max = get_form("MaxFreq");
    var samples = 10000;
    var freq_step = 0.5;
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
    filters.push(new LPF_1P(sample_rate,get_form("FLTD")));

    var scale = document.getElementById("ScaleLog");
    var use_dB = scale.checked;
    var attenuation = []
    var phase_lag = []
    var min_phase_lag = 0.0;
    var max_phase_lag = 0.0;
    var phase_wrap = 0.0;
    var min_atten = 0.0;
    var max_atten = 1.0;
    var last_phase = 0.0;
    var atten_string = "Attenuation";
    if (use_dB) {
        max_atten = 0;
        min_atten = -10;
        atten_string = "Attenuation (dB)";
    }

    // start at zero
    attenuation.push({x:0, y:0});
    phase_lag.push({x:0, y:0});

    for (freq=1; freq<=freq_max; freq+=freq_step) {
        var v = run_filters(filters, freq, sample_rate, samples);
        var aten = v[0];
        var phase = v[1] + phase_wrap;
        if (use_dB) {
            // show power in decibels
            aten = 20 * Math.log10(aten);
        } else {
            // attenuation is opposite of ratio
            aten = 1.0 - aten;
        }
        var phase_diff = phase - last_phase;
        if (phase_diff > 180) {
            phase_wrap -= 360.0;
            phase -= 360.0;
        } else if (phase_diff < -180) {
            phase_wrap += 360.0;
            phase += 360.0;
        }
        attenuation.push({x:freq, y:aten});
        phase_lag.push({x:freq, y:-phase});

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
    if (chart) {
        chart.data.datasets[0].data = attenuation;
        chart.data.datasets[1].data = phase_lag;
        chart.options.scales.xAxes[0].ticks.max = freq_max;
        chart.options.scales.yAxes[0].ticks.min = min_atten
        chart.options.scales.yAxes[0].ticks.max = max_atten;
        chart.options.scales.yAxes[0].scaleLabel.labelString = atten_string;
        chart.options.scales.yAxes[0].ticks.reverse = !use_dB;
        chart.options.scales.yAxes[1].ticks.min = -max_phase_lag;
        chart.options.scales.yAxes[1].ticks.max = -min_phase_lag;
        chart.update();
    } else {
        chart = new Chart("Attenuation", {
            type : "scatter",
            data: {
                datasets: [
                    {
                        label: 'Attenuation',
                        yAxisID: 'Attenuation',
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
                            id: 'Attenuation',
                            position: 'left',
                            ticks: {min:min_atten, max:max_atten, stepSize:0.1, reverse:!use_dB}
                        },
                        {
                            scaleLabel: { display: true, labelString: "Phase Lag(deg)" },
                            id: 'PhaseLag',
                            position: 'right',
                            gridLines: {display:false},
                            ticks: {min:-max_phase_lag, max:-min_phase_lag, stepSize:10}
                        }
                    ],
                    xAxes: [
                        {
                            scaleLabel: { display: true, labelString: "Frequency(Hz)" },
                            ticks: {min:0, max:freq_max, stepSize:10}
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
        //console.log("At 15Hz: " + attenuation[14].y + " " + phase_lag[14].y);
        //console.log("At 30Hz: " + attenuation[29].y + " " + phase_lag[29].y);
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
    var inputs = document.forms["params"].getElementsByTagName("input");
    for (const v in inputs) {
        var name = inputs[v].name;
        inputs[v].value = parseFloat(getCookie(name,inputs[v].value));
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
        if (line.startsWith("INS_")) {
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
        if (line.startsWith("ATC_RAT_RLL_FLTD") ||
            line.startsWith("Q_A_RAT_RLL_FLTD")) {
            var fvar = document.getElementById("FLTD");
            if (fvar) {
                v = line.split(/[\s,=\t]+/);
                if (v.length >= 2) {
                    var vname = v[0];
                    var value = v[1];
                    fvar.value = value;
                    console.log("set FLTD=" + value);
                }
            }
        }
    }
    fill_docs();
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
}
