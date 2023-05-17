// A js tool for plotting ArduPilot batch log data

// Generic Class to hold source for notch target
class NotchTarget {
    constructor(log, msg_name, key_name, name, mode_value) {
        this.name = name
        this.mode_value = mode_value
        this.data = []

        // Don't always need log data (static notch)
        if (log == null) {
            return
        }

        // Grab data from log
        log.parseAtOffset(msg_name)
        if ((log.messages[msg_name] == null) || (log.messages[msg_name].length == 0)) {
            return
        }

        // Convert ket into array if needed
        if (!Array.isArray(key_name)) {
            key_name = [key_name]
        }

        // Grab all given keys to data struct
        this.data.time = []
        for (var j=0; j<key_name.length; j++) {
            this.data[key_name[j]] = []
        }
        for (var i=0; i < log.messages[msg_name].time_boot_ms.length; i++) {
            this.data.time[i] = log.messages[msg_name].time_boot_ms[i] / 1000
            for (var j=0; j<key_name.length; j++) {
                this.data[key_name[j]][i] = log.messages[msg_name][key_name[j]][i]
            }
        }
    }

    get_target_freq(config) {
        if ((Object.keys(this.data).length == 0) || (this.data.time == null)) {
            return
        }
        const start_index = find_start_index(this.data.time)
        const end_index = find_end_index(this.data.time)
        if (config.ref == 0) {
            return { freq:[config.freq, config.freq], time:[this.data.time[start_index], this.data.time[end_index]] }
        }

        var freq = []
        for (let j=start_index;j<end_index;j++) {
            freq.push(this.get_target_freq_index(config, j))
        }
        return { freq:freq, time:this.data.time.slice(start_index, end_index) }
    }
}

// Tracking mode specific classes
class StaticTarget extends NotchTarget {
    constructor(log) {
        super(null, null, null, "Static", 0)
    }

    get_target_freq(config) {
        const start_time = parseFloat(document.getElementById("TimeStart").value)
        const end_time = parseFloat(document.getElementById("TimeEnd").value)
        return { freq:[config.freq, config.freq], time:[start_time, end_time] }
    }
}

class ThrottleTarget extends NotchTarget {
    constructor(log) {
        super(log, "RATE", "AOut", "Throttle", 1)
    }

    get_target_freq_index(config, index) {
        const motors_throttle = math.max(0, this.data.AOut[index])
        return config.freq * math.max(config.min_ratio, math.sqrt(motors_throttle / config.ref))
    }
}

class RPMTarget extends NotchTarget {
    constructor(log, instance, mode_value) {
        const key = "rpm" + instance
        super(log, "RPM", key, "RPM" + instance, mode_value)
        this.key = key
    }

    get_target_freq_index(config, index) {
        const rpm = this.data[this.key][index]
        if (rpm > 0) {
            return math.max(config.freq, rpm * config.ref * (1.0/60.0))
        }
        return config.freq
    }
}

class ESCTarget extends NotchTarget {
    constructor(log) {
        super(null, null, null, "ESC", 3)

        // Grab data from log, have to do it a little differently to get instances
        const msg = "ESC"
        log.parseAtOffset(msg)
        if ((log.messages[msg] == null) || (log.messages[msg].length == 0)) {
            return
        }

        // Individual RPM
        var instances = 0
        for (let i=0;i<16;i++) {
            const inst_msg = msg + "[" + i + "]"
            if (log.messages[inst_msg] != null) {
                this.data[i] = { time:[], freq:[] }
                for (var j=0; j < log.messages[inst_msg].time_boot_ms.length; j++) {
                    this.data[i].time[j] = log.messages[inst_msg].time_boot_ms[j] / 1000
                    this.data[i].freq[j] = log.messages[inst_msg].RPM[j] / 60
                }
                instances++
            }
        }

        // Average RPM
        this.data.avg_freq = []
        this.data.avg_time = []
        var inst = []
        for (let i=0;i<instances;i++) {
            inst[i] = { rpm:null, time_ms:null }
        }
        for (let i=0;i<log.messages[msg].length;i++) {
            // Update instance
            const instance = log.messages[msg][i].Instance
            const time_ms = log.messages[msg][i].time_boot_ms
            inst[instance].rpm = log.messages[msg][i].RPM
            inst[instance].time_ms = time_ms

            // Invalidate any instance that has timed out
            for (let j=0;j<instances;j++) {
                if ((j != instance) && (inst[j].time_ms != null) && ((time_ms - inst[j].time_ms) > 1000)) {
                    inst[j].time_ms = null
                    inst[j].rpm = null
                }
            }

            // If a full set of valid instances take average
            var expected_count = 0
            var count = 0
            var sum = 0
            for (let j=0;j<instances;j++) {
                if (inst[j].rpm != null) {
                    count++
                    sum += inst[j].rpm
                }
                if (inst[j].time_ms != null) {
                    expected_count++
                }
            }

            if ((count > 0) && (count == expected_count)) {
                this.data.avg_freq.push((sum / count) / 60)
                this.data.avg_time.push(time_ms / 1000)

                // Invalidate used values
                for (let j=0;j<instances;j++) {
                    inst[j].rpm = null
                }
            }
        }
    }

    get_target_freq(config) {
        if (this.data.length == 0) {
            return
        }

        const dynamic = (config.options & (1<<1)) != 0
        if (dynamic) {
            // Tracking individual motor RPM's
            let freq = []
            let time = []

            for (let i = 0; i < this.data.length; i++) {
                const start_index = find_start_index(this.data[i].time)
                const end_index = find_end_index(this.data[i].time)

                let inst_freq = this.data[i].freq.slice(start_index, end_index)
                for (let j = 0; j < inst_freq.length; j++) {
                    inst_freq[j] = math.max(inst_freq[j], config.freq)
                }

                time.push(...this.data[i].time.slice(start_index, end_index))
                freq.push(...inst_freq)

                // Add NAN to remove line from end back to the start
                time.push(NaN)
                freq.push(NaN)
            }
            return { freq:freq, time:time }

        }

        // Tracking average motor rpm
        const start_index = find_start_index(this.data.avg_time)
        const end_index = find_end_index(this.data.avg_time)

        let freq = this.data.avg_freq.slice(start_index, end_index)
        for (let j = 0; j < freq.length; j++) {
            freq[j] = math.max(freq[j], config.freq)
        }

        return { freq:freq, time:this.data.avg_time.slice(start_index, end_index) }
    }
}

class FFTTarget extends NotchTarget {
    constructor(log) {
        super(log, "FTN1", "PkAvg", "FFT", 4)

        // Grab data from log, have to do it a little differently to get instances
        const msg = "FTN2"
        log.parseAtOffset(msg)
        if ((log.messages[msg] == null) || (log.messages[msg].length == 0)) {
            return
        }

        for (let i=0;i<3;i++) {
            // FFT can track three peaks
            const inst_msg = msg + "[" + i + "]"
            if (log.messages[inst_msg] != null) {
                this.data[i] = { time:[], freq:[] }
                for (var j=0; j < log.messages[inst_msg].time_boot_ms.length; j++) {

                    // Do noise weighting between axis to get a single frequency
                    // Same as `get_weighted_freq_hz` function in AP_GyroFFT
                    const energy_x = log.messages[inst_msg].EnX[j]
                    const energy_y = log.messages[inst_msg].EnY[j]
                    const freq_x = log.messages[inst_msg].PkX[j]
                    const freq_y = log.messages[inst_msg].PkY[j]

                    if ((energy_x > 0) && (energy_y > 0)) {
                        // Weighted by relative energy
                        this.data[i].freq[j] = (freq_x*energy_x + freq_y*energy_y) / (energy_x + energy_y)
                    } else {
                        // Just take average
                        this.data[i].freq[j] = (freq_x + freq_y) * 0.5
                    }
                    this.data[i].time[j] = log.messages[inst_msg].time_boot_ms[j] / 1000
                }
            }
        }
    }

    get_target_freq(config) {
        const dynamic = (config.options & (1<<1)) != 0
        if (dynamic) {
            // Tracking multiple peaks
            let freq = []
            let time = []

            for (let i = 0; i < this.data.length; i++) {
                const start_index = find_start_index(this.data[i].time)
                const end_index = find_end_index(this.data[i].time)

                let inst_freq = this.data[i].freq.slice(start_index, end_index)
                for (let j = 0; j < inst_freq.length; j++) {
                    inst_freq[j] = math.max(inst_freq[j], config.freq)
                }

                time.push(...this.data[i].time.slice(start_index, end_index))
                freq.push(...inst_freq)

                // Add NAN to remove line from end back to the start
                time.push(NaN)
                freq.push(NaN)
            }
            return { freq:freq, time:time }
        }

        // Just center peak
        const start_index = find_start_index(this.data.time)
        const end_index = find_end_index(this.data.time)

        let freq = this.data.PkAvg.slice(start_index, end_index)
        for (let j = 0; j < freq.length; j++) {
            freq[j] = math.max(freq[j], config.freq)
        }

        return { freq:freq, time:this.data.time.slice(start_index, end_index) }
    }
}

// return hanning window array of given length (in tensorflow format)
function hanning(len) {
    const half = tf.scalar(0.5)
    return tf.sub(half, tf.mul(half, tf.cos(tf.linspace(0, 2*math.PI, len))))
}

// Calculate correction factors for linear and energy spectrum (window in tensorflow format)
// linear: 1 / mean(w)
// energy: 1 / sqrt(mean(w.^2))
function window_correction_factors(w) {
    return { 
        linear: 1/tf.mean(w).arraySync(),
        energy: 1/tf.sqrt(tf.mean(tf.square(w))).arraySync()
    }
}

// Length of real half of fft of len points
function real_length(len) {
    return math.floor(len / 2) + 1
}

// Frequency bins for given fft length and sample period (real only)
function rfft_freq(len, d) {
    const real_len = real_length(len)
    freq = []
    for (var i=0;i<real_len;i++) {
        freq[i] = i / (len * d)
    }
    return freq
}

function run_fft(data, window_size, window_spacing, windowing_function) {
    const num_points = data.x.length
    const real_len = real_length(window_size)

    var fft_x = []
    var fft_y = []
    var fft_z = []

    var center_sample = []

    // Convert to tensorflow format for faster fft
    const tf_data = tf.tensor2d([data.x, data.y, data.z])

    // Pre-allocate scale array.
    // double positive spectrum to account for discarded energy in the negative spectrum
    // Note that we don't scale the DC or Nyquist limit
    // normalize all points by the window size
    var scale = []
    scale[0] = 1 / window_size
    for (var j=1;j<real_len-1;j++) {
        scale[j] = 2 / window_size
    }
    scale[real_len-1] = 1 / window_size
    const tf_scale = tf.tensor1d(scale)

    const num_windows = math.floor((num_points-window_size)/window_spacing) + 1
    for (var i=0;i<num_windows;i++) {
        // Calculate the start of each window
        const window_start = i * window_spacing

        // Take average time for window
        center_sample[i] = window_start + window_size * 0.5

        // Get data and apply windowing function
        const windowed_data = tf.mul(tf_data.slice([0, window_start], [-1, window_size]), windowing_function)

        // Run fft
        const p2 = windowed_data.rfft()

        // Take magnitude to give real number
        const p1 = p2.abs()

        // Apply scale and convert back to js array
        const result = tf.mul(p1, tf_scale).arraySync()

        fft_x[i] = result[0]
        fft_y[i] = result[1]
        fft_z[i] = result[2]

    }

    return {x:fft_x, y:fft_y, z:fft_z, center:center_sample}
}

function run_batch_fft(data_set) {
    const num_batch = data_set.length
    const num_points = data_set[0].x.length

    var sample_rate_sum = 0
    for (let i=0;i<num_batch;i++) {
        if ((data_set[i].x.length != num_points) || (data_set[i].y.length != num_points) || (data_set[i].z.length != num_points)) {
            console.log("Uneven batch sizes")
            return
        }
        sample_rate_sum += data_set[0].sample_rate
    }

    // Average sample time
    const sample_time = num_batch / sample_rate_sum

    // Must have at least one window
    const window_per_batch = math.max(parseInt(document.getElementById("FFTWindow").value), 1)

    // Hard code 50% overlap
    const window_overlap = 0.5

    // Calculate window size for given number of windows and overlap
    const window_size = math.floor(num_points / (1 + (window_per_batch - 1)*(1-window_overlap)))

    const window_spacing = math.round(window_size * (1 - window_overlap))
    const windowing_function = hanning(window_size)

    // Get windowing correction factors for use later when plotting
    const window_correction = window_correction_factors(windowing_function)

    // Get bins
    var bins = rfft_freq(window_size, sample_time)

    var x = []
    var y = []
    var z = []

    var time = []

    for (let i=0;i<num_batch;i++) {
        var ret = run_fft(data_set[i], window_size, window_spacing, windowing_function)

        time.push(...math.add(data_set[i].sample_time, math.dotMultiply(sample_time, ret.center)))
        x.push(...ret.x)
        y.push(...ret.y)
        z.push(...ret.z)

    }

    return { bins: bins, time: time, average_sample_rate: 1/sample_time, window_size: window_size, correction: window_correction, x: x, y: y, z: z}
}

// Setup plots with no data
var Spectrogram = {}
var fft_plot = {}
const max_num_harmonics = 8
function reset() {
    let axis = ["X" , "Y", "Z"]

    // Disable all plot selection checkboxes
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            document.getElementById("Gyro" + i + "Pre" + axis[j]).disabled = true
            document.getElementById("Gyro" + i + "Post" + axis[j]).disabled = true
        }
        document.getElementById("SpecGyroInst" + i).disabled = true
    }
    document.getElementById("SpecGyroPre").disabled = true
    document.getElementById("SpecGyroPost").disabled = true
    for (let j = 0; j < 3; j++) {
        document.getElementById("SpecGyroAxis" +  axis[j]).disabled = true
    }

    // Clear extra text
    document.getElementById("FFTWindowInfo").innerHTML = ""
    document.getElementById("Gyro0_FFT_info").innerHTML = "<br><br><br>"
    document.getElementById("Gyro1_FFT_info").innerHTML = "<br><br><br>"
    document.getElementById("Gyro2_FFT_info").innerHTML = "<br><br><br>"

    // FFT plot setup
    fft_plot.data = []
    for (let i=0;i<Gyro_batch.length;i++) {
        // For each gyro
        var sensor_num = "?"
        var pre_post = ""
        if (Gyro_batch[i] != null) {
            sensor_num = Gyro_batch[i].sensor_num + 1
            pre_post = Gyro_batch[i].post_filter ? " Post-filter" : " Pre-filter"
        }
        for (let j=0;j<3;j++) {
            // For each axis
            const name = sensor_num + axis[j] + pre_post
            fft_plot.data[i*3 + j] = { mode: "lines",
                                       visible: true,
                                       name: name,
                                       // this extra data allows us to put the name in the hover tool tip
                                       meta: name }
        }
    }

    fft_plot.layout = {
        xaxis: {title: {text: ""}, type: "linear"},
        yaxis: {title: {text: ""}},
        showlegend: true,
        legend: {itemclick: false, itemdoubleclick: false },
        margin: { b: 50, l: 50, r: 50, t: 20 }
    }

    var FFTPlot = document.getElementById("FFTPlot")
    Plotly.purge(FFTPlot)
    Plotly.newPlot(FFTPlot, fft_plot.data, fft_plot.layout, {displaylogo: false});

    // Spectrogram setup
    // Add surface
    Spectrogram.data = [{
        type:"heatmap",
        colorbar: {title: {side: "right", text: ""}, orientation: "h"},
        transpose: true,
        zsmooth: "best",
        hovertemplate: ""
    }]

    // Add tracking lines
    // Two harmonic notch filters each with upto 8 harmonics
    for (let i=0;i<2;i++) {
        let Group_name = "Notch " + (i+1)
        let dash = (i == 0) ? "solid" : "dot"
        for (let j=0;j<max_num_harmonics;j++) {
            let name = (j == 0) ? "Fundamental" : "Harmonic " + j
            Spectrogram.data.push({
                type:"scatter",
                mode: "lines",
                line: { width: 4, dash: dash },
                visible: false,
                name: name,
                meta: Group_name + "<br>" + name,
                legendgroup: i,
                legendgrouptitle: { text: "" }
            })
        }
    }

    // Define Layout
    Spectrogram.layout = {
        xaxis: {title: {text: "Time (s)"}},
        yaxis: {title: {text: ""}, type: "linear"},
        showlegend: true,
        legend: {itemclick: false, itemdoubleclick: false },
        margin: { b: 50, l: 50, r: 50, t: 20 }
    }

    var SpectrogramPlot = document.getElementById("Spectrogram")
    Plotly.purge(SpectrogramPlot)
    Plotly.newPlot(SpectrogramPlot, Spectrogram.data, Spectrogram.layout, {displaylogo: false});
}

// Calculate if needed and re-draw, called from calculate button
function re_calc() {

    calculate()

    redraw()
}

// Force full re-calc on next run, on window size change
function clear_calculation() {
    if (Gyro_batch == null) {
        return
    }
    for (let i = 0; i < Gyro_batch.length; i++) {
        if (Gyro_batch[i] == null) {
            continue
        }
        Gyro_batch[i].FFT = null
    }
}

// Re-run all FFT's
function calculate() {
    for (let i = 0; i < Gyro_batch.length; i++) {
        if (Gyro_batch[i] == null) {
            continue
        }
        if (Gyro_batch[i].FFT == null) {
            Gyro_batch[i].FFT = run_batch_fft(Gyro_batch[i])
        }
    }
}

// Get configured amplitude scale
function get_amplitude_scale() {

    const use_DB = document.getElementById("ScaleLog").checked;
    const use_PSD = document.getElementById("SpectrumPSD").checked;

    var ret = {}
    if (use_DB) {
        if (use_PSD) {
            ret.fun = function (x) { return math.dotMultiply(math.log10(math.dotMultiply(x,x)), 10.0) } // 10 * log10(x.^2)
            ret.label = "PSD (db)"
        } else {
            ret.fun = function (x) { return math.dotMultiply(math.log10(x), 10.0) } // 10 * log10(x)
            ret.label = "Linear amplitude (db)"
        }
        ret.hover = function (axis) { return "%{" + axis + ":.2f} db" }

    } else {
        if (use_PSD) {
            ret.fun = function (x) { return math.dotMultiply(x,x) }
            ret.label = "PSD"
        } else {
            ret.fun = function (x) { return x }
            ret.label = "Linear amplitude"
        }
        ret.hover = function (axis) { return "%{" + axis + ":.2f}" }

    }

    ret.use_PSD = use_PSD

    return ret
}

// Get configured frequency scale object
function get_frequency_scale() {

    const use_RPM = document.getElementById("freq_Scale_RPM").checked;

    var ret = {}
    if (use_RPM) {
        ret.fun = function (x) { return math.dotMultiply(x, 60.0) }
        ret.label = "RPM"
        ret.hover = function (axis) { return "%{" + axis + ":.2f} RPM" }

    } else {
        ret.fun = function (x) { return x }
        ret.label = "Frequency (Hz)"
        ret.hover = function (axis) { return "%{" + axis + ":.2f} Hz" }
    }

    ret.type = document.getElementById("freq_ScaleLog").checked ? "log" : "linear"

    return ret
}

// Look through time array and return first index before start time
function find_start_index(time) {
    const start_time = parseFloat(document.getElementById("TimeStart").value)

    var start_index = 0
    for (j = 0; j<time.length; j++) {
        // Move forward start index while time is less than start time
        if (time[j] < start_time) {
            start_index = j
        }
    }
    return start_index
}

// Look through time array and return first index after end time
function find_end_index(time) {
    const end_time = parseFloat(document.getElementById("TimeEnd").value)

    var end_index
    for (j = 0; j<time.length; j++) {
        // Move forward end index while time is less than end time
        if (time[j] <= end_time) {
            end_index = j
        }
    }
    return end_index + 1
}

function redraw() {

    // Graph config
    const amplitude_scale = get_amplitude_scale()
    const frequency_scale = get_frequency_scale()

    // Setup axes
    fft_plot.layout.xaxis.type = frequency_scale.type
    fft_plot.layout.xaxis.title.text = frequency_scale.label
    fft_plot.layout.yaxis.title.text = amplitude_scale.label

    const fft_hovertemplate = "<extra></extra>%{meta}<br>" + frequency_scale.hover("x") + "<br>" + amplitude_scale.hover("y")

    for (let i = 0; i < Gyro_batch.length; i++) {
        if ((Gyro_batch[i] == null) || (Gyro_batch[i].FFT == null)) {
            continue
        }

        // Find the start and end index
        const start_index = find_start_index(Gyro_batch[i].FFT.time)
        const end_index = find_end_index(Gyro_batch[i].FFT.time)

        // Number of windows to plot
        const plot_length = end_index - start_index

        // Windowing amplitude correction depends on spectrum of interest
        const window_correction = amplitude_scale.use_PSD ? (Gyro_batch[i].FFT.correction.energy * math.sqrt(1/2)) : Gyro_batch[i].FFT.correction.linear

        // Take mean from start to end
        var fft_mean_x = 0
        var fft_mean_y = 0
        var fft_mean_z = 0
        for (let j=start_index;j<end_index;j++) {
            // Add to mean sum
            fft_mean_x = math.add(fft_mean_x, amplitude_scale.fun(math.dotMultiply(Gyro_batch[i].FFT.x[j], window_correction)))
            fft_mean_y = math.add(fft_mean_y, amplitude_scale.fun(math.dotMultiply(Gyro_batch[i].FFT.y[j], window_correction)))
            fft_mean_z = math.add(fft_mean_z, amplitude_scale.fun(math.dotMultiply(Gyro_batch[i].FFT.z[j], window_correction)))
        }

        // Set scaled y data
        fft_plot.data[i*3 + 0].y = math.dotMultiply(fft_mean_x, 1 / plot_length)
        fft_plot.data[i*3 + 1].y = math.dotMultiply(fft_mean_y, 1 / plot_length)
        fft_plot.data[i*3 + 2].y = math.dotMultiply(fft_mean_z, 1 / plot_length)

        // Set scaled x data
        const scaled_bins = frequency_scale.fun(Gyro_batch[i].FFT.bins)
        fft_plot.data[i*3 + 0].x = scaled_bins
        fft_plot.data[i*3 + 1].x = scaled_bins
        fft_plot.data[i*3 + 2].x = scaled_bins

        // Set hover over function
        fft_plot.data[i*3 + 0].hovertemplate = fft_hovertemplate
        fft_plot.data[i*3 + 1].hovertemplate = fft_hovertemplate
        fft_plot.data[i*3 + 2].hovertemplate = fft_hovertemplate

    }

    Plotly.redraw("FFTPlot")

    redraw_Spectrogram()

}

// Find the instance of "Gyro_batch" that matches the selection
function find_instance(gyro_instance, post_filter) {
    for (let i = 0; i < Gyro_batch.length; i++) {
        if ((Gyro_batch[i] == null) || (Gyro_batch[i].FFT == null)) {
            continue
        }
        if ((Gyro_batch[i].post_filter == post_filter) && (Gyro_batch[i].sensor_num == gyro_instance)) {
            return i
        }
    }
}

function redraw_Spectrogram() {

    // Work out which index to plot
    var gyro_instance
    if (document.getElementById("SpecGyroInst0").checked) {
        gyro_instance = 0
    } else if (document.getElementById("SpecGyroInst1").checked) {
        gyro_instance = 1
    } else {
        gyro_instance = 2
    }
    const post_filter = document.getElementById("SpecGyroPost").checked

    const batch_instance = find_instance(gyro_instance, post_filter)
    if (batch_instance == null) {
        console.log("Could not find matching dataset")
        return
    }

    var axis
    if (document.getElementById("SpecGyroAxisX").checked) {
        axis = "x"
    } else if (document.getElementById("SpecGyroAxisY").checked) {
        axis = "y"
    } else {
        axis = "z"
    }

    // Get scales
    const amplitude_scale = get_amplitude_scale()
    const frequency_scale = get_frequency_scale()

    // Setup axes
    Spectrogram.layout.yaxis.type = frequency_scale.type
    Spectrogram.layout.yaxis.title.text = frequency_scale.label

    Spectrogram.data[0].hovertemplate = "<extra></extra>" + "%{x:.2f} s<br>" + frequency_scale.hover("y") + "<br>" + amplitude_scale.hover("z")
    Spectrogram.data[0].colorbar.title.text = amplitude_scale.label

    // Find the start and end index
    const start_index = find_start_index(Gyro_batch[batch_instance].FFT.time)
    const end_index = find_end_index(Gyro_batch[batch_instance].FFT.time)

    // Number of windows to plot
    const plot_length = end_index - start_index

    // Setup xy data (x and y swapped because transpose flag is set)
    Spectrogram.data[0].x = Gyro_batch[batch_instance].FFT.time.slice(start_index, end_index)
    Spectrogram.data[0].y = frequency_scale.fun(Gyro_batch[batch_instance].FFT.bins)

    // Windowing amplitude correction depends on spectrum of interest
    const window_correction = amplitude_scale.use_PSD ? (Gyro_batch[batch_instance].FFT.correction.energy * math.sqrt(1/2)) : Gyro_batch[batch_instance].FFT.correction.linear

    // Setup z data
    Spectrogram.data[0].z = []
    for (j = 0; j<plot_length; j++) {
        const index = start_index + j
        Spectrogram.data[0].z[j] = amplitude_scale.fun(math.dotMultiply(Gyro_batch[batch_instance].FFT[axis][index], window_correction))
    }

    // Setup tracking lines
    const tracking_hovertemplate = "<extra></extra>%{meta}<br>" +  "%{x:.2f} s<br>" + frequency_scale.hover("y")
    for (let i=0;i<HNotch_setup.length;i++) {
        // Plus one for the spectrogram plot
        const plot_offset = i * max_num_harmonics + 1

        // Hide all
        for (let j=0;j<max_num_harmonics;j++) {
            Spectrogram.data[plot_offset + j].visible = false
        }

        // Notch disabled, nothing to do
        if ((HNotch_setup[i].enable <= 0) || (HNotch_setup[i].harmonics <= 0)) {
            continue
        }

        // Find configured tracking source
        let tracking
        for (let j=0;j<tracking_methods.length;j++) {
            if (HNotch_setup[i].mode == tracking_methods[j].mode_value) {
                tracking = tracking_methods[j]
            }
        }

        // Invalid mode
        if (tracking == null) {
            continue
        }

        const Group_name = "Notch " + (i+1) + ": " + tracking.name
        const fundamental = tracking.get_target_freq(HNotch_setup[i])

        // Enable each harmonic
        for (let j=0;j<max_num_harmonics;j++) {
            if ((HNotch_setup[i].harmonics & (1<<j)) == 0) {
                continue
            }
            const harmonic_freq = math.dotMultiply(fundamental.freq, j+1)
            if (harmonic_freq == null) {
                break
            }

            Spectrogram.data[plot_offset + j].visible = true
            Spectrogram.data[plot_offset + j].x = fundamental.time
            Spectrogram.data[plot_offset + j].y = frequency_scale.fun(harmonic_freq)
            Spectrogram.data[plot_offset + j].hovertemplate = tracking_hovertemplate
            Spectrogram.data[plot_offset + j].legendgrouptitle.text = Group_name
        }

    }

    Plotly.redraw("Spectrogram")
}

// Update lines that are shown in FFT plot
function update_hidden(checkbox) {

    const gyro_instance = parseFloat(checkbox.id.match(/\d+/g));
    const post_filter = checkbox.id.includes("Post");

    // Find the gyro data this box matches with
    const batch_instance = find_instance(gyro_instance, post_filter)
    if (batch_instance == null) {
        console.log("Could not find matching dataset for checkbox: " + checkbox.id)
        return
    }

    const axi = checkbox.id.substr(checkbox.id.length - 1)

    let axi_index
    let axis = ["X" , "Y", "Z"]
    for (let j=0;j<3;j++) {
        if (axis[j] == axi) {
            axi_index = j
            break
        }
    }

    fft_plot.data[batch_instance*3 + axi_index].visible = checkbox.checked

    Plotly.redraw("FFTPlot")

}

// Grab param from log
function get_param_value(param_log, name) {
    var value
    for (let i = 0; i < param_log.Name.length; i++) {
        if (param_log.Name[i] === name) {
            if ((value != null) && (value != PARM_log.Value[i])) {
                console.log("Param changed in flight: " + name)
            }
            value = param_log.Value[i]
        }
    }
    return value
}

function get_HNotch_param_names() {
    prefix = ["INS_HNTCH_", "INS_HNTC2_"]
    ret = []
    for (let i = 0; i < prefix.length; i++) {
        ret[i] = {enable: prefix[i] + "ENABLE",
                  mode: prefix[i] + "MODE",
                  freq: prefix[i] + "FREQ",
                  ref: prefix[i] + "REF",
                  min_ratio: prefix[i] + "FM_RAT",
                  harmonics: prefix[i] + "HMNCS",
                  options: prefix[i] + "OPTS"}
    }
    return ret
}

function HNotch_param_read() {
    const HNotch_params = get_HNotch_param_names("INS_HNTCH_")

    for (let i = 0; i < HNotch_params.length; i++) {
        // Enable all params in group if enable is set
        const enable_input = parseFloat(document.getElementById(HNotch_params[i].enable).value) > 0
        for (const [key, value] of Object.entries(HNotch_params[i])) {
            if (key != "enable") {
                document.getElementById(value).disabled = !enable_input
            }
        }
    }

    // Load values into HNotch object
    for (let i = 0; i < HNotch_params.length; i++) {
        HNotch_setup[i] = []
        for (const [key, value] of Object.entries(HNotch_params[i])) {
            HNotch_setup[i][key] = parseFloat(document.getElementById(value).value)
        }
    }
}

var Gyro_batch
var tracking_methods
var HNotch_setup = []
function load(log_file) {

    const start = performance.now()

    var log = new DataflashParser()
    log.processData(log_file)

    // Load batch messages
    log.parseAtOffset("ISBH")
    log.parseAtOffset("ISBD")
    if (log.messages.ISBH == null || log.messages.ISBD == null) {
        console.log("No Batch logging msg")
        return
    }

    // Assign batches to each sensor
    // Only interested in gyro here
    const IMU_SENSOR_TYPE_GYRO = 1
    Gyro_batch = []
    let data_index = 0
    for (let i = 0; i < log.messages.ISBH.N.length; i++) {
        // Parse headers
        if (log.messages.ISBH.type[i] != IMU_SENSOR_TYPE_GYRO) {
            continue
        }

        const instance = log.messages.ISBH.instance[i]
        if (Gyro_batch[instance] == null) {
            Gyro_batch[instance] = []
        }

        let decode_complete = false

        // Advance data index until sequence match
        const seq_num = log.messages.ISBH.N[i]
        while (log.messages.ISBD.N[data_index] != seq_num) {
            data_index++
            if (data_index >= log.messages.ISBD.N.length) {
                // This is expected at the end of a log, no more msgs to add, break here
                console.log("Could not find next sequence " + i + " of " + log.messages.ISBH.N.length-1)
                decode_complete = true
                break
            }
        }
        if (decode_complete) {
            break
        }

        let x = []
        let y = []
        let z = []
        const num_samples = log.messages.ISBH.smp_cnt[i]
        const num_data_msg = num_samples / 32
        for (let j = 0; j < num_data_msg; j++) {
            // Read in expected number of samples
            if ((log.messages.ISBD.N[data_index] != seq_num) || (log.messages.ISBD.seqno[data_index] != j)) {
                console.log("Missing or extra data msg")
                return
            }

            // Accumulate data for this batch
            x.push(...log.messages.ISBD.x[data_index])
            y.push(...log.messages.ISBD.y[data_index])
            z.push(...log.messages.ISBD.z[data_index])

            data_index++
            if (data_index >= log.messages.ISBD.N.length) {
                console.log("sequence incomplete " + i + " of " + log.messages.ISBH.N.length-1)
                decode_complete = true
                break
            }
        }
        if (decode_complete) {
            break
        }

        if ((x.length != num_samples) || (y.length != num_samples) || (z.length != num_samples)) {
            console.log("sample length wrong")
            return
        }

        // Remove logging scale factor
        const mul = 1/log.messages.ISBH.mul[i]
        x = math.dotMultiply(x, mul)
        y = math.dotMultiply(y, mul)
        z = math.dotMultiply(z, mul)

        // Add to batches for this instance
        Gyro_batch[instance].push({ sample_time: log.messages.ISBH.SampleUS[i] / 1000000,
                                    sample_rate: log.messages.ISBH.smp_rate[i],
                                    x: x,
                                    y: y,
                                    z: z })
    }

    if (Gyro_batch.length == 0) {
        console.log("no data")
        return
    }

    log.parseAtOffset('PARM')

    // Try and decode device IDs
    var num_gyro = 0
    for (let i = 0; i < 3; i++) {
        const ID_param = i == 0 ? "INS_GYR_ID" : "INS_GYR" + (i + 1) + "_ID"
        const ID = get_param_value(log.messages.PARM, ID_param)
        if ((ID != null) && (ID > 0)) {
            const decoded = decode_devid(ID, DEVICE_TYPE_IMU)
            if (decoded != null) {
                document.getElementById("Gyro" + i + "_info").innerHTML = decoded.name + " via " + decoded.bus_type
            }
            num_gyro++
        }
    }

    // Work out if logging is pre/post from param value
    const INS_LOG_BAT_OPT = get_param_value(log.messages.PARM, "INS_LOG_BAT_OPT")
    const _doing_sensor_rate_logging = (INS_LOG_BAT_OPT & (1 << 0)) != 0
    const _doing_post_filter_logging = (INS_LOG_BAT_OPT & (1 << 1)) != 0
    const _doing_pre_post_filter_logging = (INS_LOG_BAT_OPT & (1 << 2)) != 0
    const use_instance_offset = _doing_pre_post_filter_logging || (_doing_post_filter_logging && _doing_sensor_rate_logging)
    for (let i = 0; i < Gyro_batch.length; i++) {
        if (Gyro_batch[i] == null) {
            continue
        }
        if (use_instance_offset && (i >= num_gyro)) {
            Gyro_batch[i].sensor_num = i - num_gyro
            Gyro_batch[i].post_filter = true
        } else {
            Gyro_batch[i].sensor_num = i
            Gyro_batch[i].post_filter = _doing_post_filter_logging && !_doing_pre_post_filter_logging
        }
    }

    // setup/reset plot and options
    reset()


    // Load potential sources of notch tracking targets
    tracking_methods = [new StaticTarget(),
                        new ThrottleTarget(log),
                        new RPMTarget(log, 1, 2),
                        new ESCTarget(log),
                        new FFTTarget(log),
                        new RPMTarget(log, 2, 5)]

    // Read from log into HTML box
    const HNotch_params = get_HNotch_param_names()
    for (let i = 0; i < HNotch_params.length; i++) {
        for (const param of Object.values(HNotch_params[i])) {
            const value = get_param_value(log.messages.PARM, param)
            if (value != null) {
                document.getElementById(param).value = value
            }
        }
        // Allow enable values to be changed
        document.getElementById(HNotch_params[i].enable).disabled = false 
    }

    // Load notch params
    HNotch_param_read()

    // Update ranges of start and end time
    var start_time
    var end_time
    for (let i = 0; i < Gyro_batch.length; i++) {
        if (Gyro_batch[i] == null) {
            continue
        }

        const batch_start = Gyro_batch[i][0].sample_time
        if ((start_time == null) || (batch_start < start_time)) {
            start_time = batch_start
        }

        const batch_end = Gyro_batch[i][Gyro_batch[i].length - 1].sample_time
        if ((end_time == null) || (batch_end > end_time)) {
            end_time = batch_end
        }
    }
    start_time = math.floor(start_time)
    end_time = math.ceil(end_time)

    var start_input = document.getElementById("TimeStart")
    start_input.disabled = false;
    start_input.min = start_time
    start_input.value = start_time
    start_input.max = end_time

    var end_input = document.getElementById("TimeEnd")
    end_input.disabled = false;
    end_input.min = start_time
    end_input.value = end_time
    end_input.max = end_time

    // Enable checkboxes for sensors which are present
    var first_gyro
    var have_pre = false
    for (let i = 0; i < Gyro_batch.length; i++) {
        if (Gyro_batch[i] == null) {
            continue
        }
        const prepost = Gyro_batch[i].post_filter ? "Post" : "Pre"
        let axis = ["X" , "Y", "Z"]
        for (let j = 0; j < 3; j++) {
            var fft_check = document.getElementById("Gyro" + Gyro_batch[i].sensor_num + prepost + axis[j])
            fft_check.disabled = false
            fft_check.checked = true
        }

        // Track which sensors are present for spectrogram
        if (first_gyro == null || (Gyro_batch[i].sensor_num < first_gyro)) {
            first_gyro = Gyro_batch[i].sensor_num
        }
        if (Gyro_batch[i].post_filter == false) {
            document.getElementById("SpecGyroPre").disabled = false
            have_pre = true
        } else {
            document.getElementById("SpecGyroPost").disabled = false
        }
        document.getElementById("SpecGyroInst" + Gyro_batch[i].sensor_num).disabled = false
        document.getElementById("SpecGyroAxisX").checked = true
        document.getElementById("SpecGyroAxisY").disabled = false
        document.getElementById("SpecGyroAxisZ").disabled = false
    }

    // Default spectrograph to first sensor, pre if available and X axis
    document.getElementById("SpecGyroInst" + first_gyro).checked = true
    document.getElementById("SpecGyro" + (have_pre ? "Pre" : "Post")).checked = true
    document.getElementById("SpecGyroAxisX").disabled = false

    // Calculate FFT
    calculate()

    // Plot
    redraw()

    // Enable the calculate button
    document.getElementById("calculate").disabled = false

    // Set FFT info
    var set_batch_len_msg = false
    for (let i = 0; i < 3; i++) {
        let sample_rate = 0
        let window_size = 0
        let count = 0
        for (let j = 0; j < Gyro_batch.length; j++) {
            if ((Gyro_batch[j] == null) || Gyro_batch[j].sensor_num != i) {
                continue
            }
            sample_rate += Gyro_batch[j].FFT.average_sample_rate
            window_size += Gyro_batch[j].FFT.window_size
            count++
        }
        if (count == 0) {
            continue
        }
        sample_rate /= count
        window_size /= count

        document.getElementById("Gyro" + i + "_FFT_info").innerHTML = "Logging rate : " + (sample_rate).toFixed(2) + " Hz<br><br>" +
                                                                        "Frequency  resolution : " + (sample_rate/window_size).toFixed(2) + " Hz"

        if (set_batch_len_msg == false) {
            set_batch_len_msg = true
            document.getElementById("FFTWindowInfo").innerHTML = "Window size: " + window_size
        }
    }


    const end = performance.now();
    console.log(`Load took: ${end - start} ms`);
}
