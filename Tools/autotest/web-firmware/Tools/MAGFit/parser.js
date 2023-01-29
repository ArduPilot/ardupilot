const MAV_TYPE_GENERIC = 0 // Generic micro air vehicle.
const MAV_TYPE_FIXED_WING = 1 // Fixed wing aircraft.
const MAV_TYPE_QUADROTOR = 2 // Quadrotor
const MAV_TYPE_COAXIAL = 3 // Coaxial helicopter
const MAV_TYPE_HELICOPTER = 4 // Normal helicopter with tail rotor.
const MAV_TYPE_ANTENNA_TRACKER = 5 // Ground installation
const MAV_TYPE_GCS = 6 // Operator control unit / ground control station
const MAV_TYPE_AIRSHIP = 7 // Airship, controlled
const MAV_TYPE_FREE_BALLOON = 8 // Free balloon, uncontrolled
const MAV_TYPE_ROCKET = 9 // Rocket
const MAV_TYPE_GROUND_ROVER = 10 // Ground rover
const MAV_TYPE_SURFACE_BOAT = 11 // Surface vessel, boat, ship
const MAV_TYPE_SUBMARINE = 12 // Submarine
const MAV_TYPE_HEXAROTOR = 13 // Hexarotor
const MAV_TYPE_OCTOROTOR = 14 // Octorotor
const MAV_TYPE_TRICOPTER = 15 // Tricopter
const MAV_TYPE_FLAPPING_WING = 16 // Flapping wing
const MAV_TYPE_KITE = 17 // Kite
const MAV_TYPE_ONBOARD_CONTROLLER = 18 // Onboard companion controller
const MAV_TYPE_VTOL_DUOROTOR = 19 // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
const MAV_TYPE_VTOL_QUADROTOR = 20 // Quad-rotor VTOL using a V-shaped quad config in vertical operation.
// Tailsitter.
const MAV_TYPE_VTOL_TILTROTOR = 21 // Tiltrotor VTOL
const MAV_TYPE_VTOL_RESERVED2 = 22 // VTOL reserved 2
const MAV_TYPE_VTOL_RESERVED3 = 23 // VTOL reserved 3
const MAV_TYPE_VTOL_RESERVED4 = 24 // VTOL reserved 4
const MAV_TYPE_VTOL_RESERVED5 = 25 // VTOL reserved 5
const MAV_TYPE_GIMBAL = 26 // Onboard gimbal
const MAV_TYPE_ADSB = 27 // Onboard ADSB peripheral
const MAV_TYPE_PARAFOIL = 28 // Steerable, nonrigid airfoil
const MAV_TYPE_DODECAROTOR = 29 // Dodecarotor
const MAV_TYPE_CAMERA = 30 // Camera
const MAV_TYPE_CHARGING_STATION = 31 // Charging station
const MAV_TYPE_FLARM = 32 // Onboard FLARM collision avoidance system
const MAV_TYPE_ENUM_END = 33 //


const modeMappingApm = {
    0: 'MANUAL',
    1: 'CIRCLE',
    2: 'STABILIZE',
    3: 'TRAINING',
    4: 'ACRO',
    5: 'FBWA',
    6: 'FBWB',
    7: 'CRUISE',
    8: 'AUTOTUNE',
    10: 'AUTO',
    11: 'RTL',
    12: 'LOITER',
    13: 'TAKEOFF',
    14: 'AVOID_ADSB',
    15: 'GUIDED',
    16: 'INITIALISING',
    17: 'QSTABILIZE',
    18: 'QHOVER',
    19: 'QLOITER',
    20: 'QLAND',
    21: 'QRTL',
    22: 'QAUTOTUNE',
    23: 'QACRO',
    24: 'THERMAL'
}
const modeMappingAcm = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'LOITER',
    6: 'RTL',
    7: 'CIRCLE',
    9: 'LAND',
    11: 'DRIFT',
    13: 'SPORT',
    14: 'FLIP',
    15: 'AUTOTUNE',
    16: 'POSHOLD',
    17: 'BRAKE',
    18: 'THROW',
    19: 'AVOID_ADSB',
    20: 'GUIDED_NOGPS',
    21: 'SMART_RTL',
    22: 'FLOWHOLD',
    23: 'FOLLOW',
    24: 'ZIGZAG',
    25: 'SYSTEMID',
    26: 'AUTOROTATE'
}
const modeMappingRover = {
    0: 'MANUAL',
    1: 'ACRO',
    3: 'STEERING',
    4: 'HOLD',
    5: 'LOITER',
    6: 'FOLLOW',
    7: 'SIMPLE',
    10: 'AUTO',
    11: 'RTL',
    12: 'SMART_RTL',
    15: 'GUIDED',
    16: 'INITIALISING'
}
const modeMappingTracker = {
    0: 'MANUAL',
    1: 'STOP',
    2: 'SCAN',
    3: 'SERVO_TEST',
    10: 'AUTO',
    16: 'INITIALISING'
}
const modeMappingSub = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    7: 'CIRCLE',
    9: 'SURFACE',
    16: 'POSHOLD',
    19: 'MANUAL',
    20: 'MOTOR_DETECT'
}


const multipliers = {
    '-': 0, // no multiplier e.g. a string
    '?': 1, // multipliers which haven't been worked out yet....
    // <leave a gap here, just in case....>
    '2': 1e2,
    '1': 1e1,
    '0': 1e0,
    'A': 1e-1,
    'B': 1e-2,
    'C': 1e-3,
    'D': 1e-4,
    'E': 1e-5,
    'F': 1e-6,
    'G': 1e-7,
    // <leave a gap here, just in case....>
    '!': 3.6, // (ampere*second => milliampere*hour) and (km/h => m/s)
    '/': 3600 // (ampere*second => ampere*hour)
}

const multipliersTable = {
    0.000001: 'n',
    1000: 'M',
    0.001: 'm'
}

const HEAD1 = 163
const HEAD2 = 149

const units = {
    '-': '', // no units e.g. Pi, or a string
    '?': 'UNKNOWN', // Units which haven't been worked out yet....
    'A': 'A', // Ampere
    'd': '°', // of the angular variety, -180 to 180
    'b': 'B', // bytes
    'k': '°/s', // degrees per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    'D': '°', // degrees of latitude
    'e': '°/s/s', // degrees per second per second. Degrees are NOT SI, but is some situations more user-friendly
    'E': 'rad/s', // radians per second
    'G': 'Gauss', // Gauss is not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here
    'h': '°', // 0.? to 359.?
    'i': 'A.s', // Ampere second
    'J': 'W.s', // Joule (Watt second)
    // { 'l', "l" },          // litres
    'L': 'rad/s/s', // radians per second per second
    'm': 'm', // metres
    'n': 'm/s', // metres per second
    // { 'N', "N" },          // Newton
    'o': 'm/s/s', // metres per second per second
    'O': '°C', // degrees Celsius. Not SI, but Kelvin is too cumbersome for most users
    '%': '%', // percent
    'S': 'satellites', // number of satellites
    's': 's', // seconds
    'q': 'rpm', // rounds per minute. Not SI, but sometimes more intuitive than Hertz
    'r': 'rad', // radians
    'U': '°', // degrees of longitude
    'u': 'ppm', // pulses per minute
    'v': 'V', // Volt
    'P': 'Pa', // Pascal
    'w': 'Ohm', // Ohm
    'Y': 'us', // pulse width modulation in microseconds
    'z': 'Hz', // Hertz
    '#': 'instance' // instance number for message
}

function getModeMap (mavType) {
    let map
    if ([MAV_TYPE_QUADROTOR,
        MAV_TYPE_HELICOPTER,
        MAV_TYPE_HEXAROTOR,
        MAV_TYPE_OCTOROTOR,
        MAV_TYPE_COAXIAL,
        MAV_TYPE_TRICOPTER].includes(mavType)) {
        map = modeMappingAcm
    }
    if (mavType === MAV_TYPE_FIXED_WING) {
        map = modeMappingApm
    }
    if (mavType === MAV_TYPE_GROUND_ROVER) {
        map = modeMappingRover
    }
    if (mavType === MAV_TYPE_ANTENNA_TRACKER) {
        map = modeMappingTracker
    }
    if (mavType === MAV_TYPE_SUBMARINE) {
        map = modeMappingSub
    }
    if (map == null) {
        return null
    }
    return map
}

function assignColumn (obj) {
    var ArrayOfString = obj.split(',')
    return ArrayOfString
}

// Converts from degrees to radians.
Math.radians = function (degrees) {
    return degrees * Math.PI / 180
}

// Converts from radians to degrees.
Math.degrees = function (radians) {
    return radians * 180 / Math.PI
}

class DataflashParser {
    constructor () {
        this.time = null
        this.timebase = null
        this.buffer = null
        this.data = null
        this.FMT = []
        this.FMT[128] = {
            'Type': '128',
            'length': '89',
            'Name': 'FMT',
            'Format': 'BBnNZ',
            'Columns': 'Type,Length,Name,Format,Columns'
        }
        this.offset = 0
        this.msgType = []
        this.offsetArray = []
        this.totalSize = null
        this.messages = {}
        this.lastPercentage = 0
        this.sent = false
        this.maxPercentageInterval = 0.05
        this.messageTypes = {}
        this.alreadyParsed = []
    }

    FORMAT_TO_STRUCT (obj) {
        var temp
        var dict = {
            name: obj.Name,
            fieldnames: obj.Columns.split(',')
        }

        let column = assignColumn(obj.Columns)
        let low
        let n
        for (let i = 0; i < obj.Format.length; i++) {
            temp = obj.Format.charAt(i)
            switch (temp) {
            case 'b':
                dict[column[i]] = this.data.getInt8(this.offset)
                this.offset += 1
                break
            case 'B':
                dict[column[i]] = this.data.getUint8(this.offset)
                this.offset += 1
                break
            case 'h':
                dict[column[i]] = this.data.getInt16(this.offset, true)
                this.offset += 2
                break
            case 'H':
                dict[column[i]] = this.data.getUint16(this.offset, true)
                this.offset += 2
                break
            case 'i':
                dict[column[i]] = this.data.getInt32(this.offset, true)
                this.offset += 4
                break
            case 'I':
                dict[column[i]] = this.data.getUint32(this.offset, true)
                this.offset += 4
                break
            case 'f':
                dict[column[i]] = this.data.getFloat32(this.offset, true)
                this.offset += 4
                break
            case 'd':
                dict[column[i]] = this.data.getFloat64(this.offset, true)
                this.offset += 8
                break
            case 'Q':
                low = this.data.getUint32(this.offset, true)
                this.offset += 4
                n = this.data.getUint32(this.offset, true) * 4294967296.0 + low
                if (low < 0) n += 4294967296
                dict[column[i]] = n
                this.offset += 4
                break
            case 'q':
                low = this.data.getInt32(this.offset, true)
                this.offset += 4
                n = this.data.getInt32(this.offset, true) * 4294967296.0 + low
                if (low < 0) n += 4294967296
                dict[column[i]] = n
                this.offset += 4
                break
            case 'n':
                // TODO: fix these regex and unsilent linter
                // eslint-disable-next-line
                dict[column[i]] = String.fromCharCode.apply(null, new Uint8Array(this.buffer, this.offset, 4)).replace(/\x00+$/g, '')
                this.offset += 4
                break
            case 'N':
                // eslint-disable-next-line
                dict[column[i]] = String.fromCharCode.apply(null, new Uint8Array(this.buffer, this.offset, 16)).replace(/\x00+$/g, '')
                this.offset += 16
                break
            case 'Z':
                // eslint-disable-next-line
                dict[column[i]] = String.fromCharCode.apply(null, new Uint8Array(this.buffer, this.offset, 64)).replace(/\x00+$/g, '')
                this.offset += 64
                break
            case 'c':
                // this.this.data.setInt16(offset,true);
                dict[column[i]] = this.data.getInt16(this.offset, true) / 100
                this.offset += 2
                break
            case 'C':
                // this.data.setUint16(offset,true);
                dict[column[i]] = this.data.getUint16(this.offset, true) / 100
                this.offset += 2
                break
            case 'E':
                // this.data.setUint32(offset,true);
                dict[column[i]] = this.data.getUint32(this.offset, true) / 100
                this.offset += 4
                break
            case 'e':
                // this.data.setInt32(offset,true);
                dict[column[i]] = this.data.getInt32(this.offset, true) / 100
                this.offset += 4
                break
            case 'L':
                // this.data.setInt32(offset,true);
                dict[column[i]] = this.data.getInt32(this.offset, true)
                this.offset += 4
                break
            case 'M':
                // this.data.setInt32(offset,true);
                dict[column[i]] = this.data.getUint8(this.offset)
                this.offset += 1
                break
            }
        }
        return dict
    }

    gpstimetoTime (week, msec) {
        let epoch = 86400 * (10 * 365 + (1980 - 1969) / 4 + 1 + 6 - 2)
        return epoch + 86400 * 7 * week + msec * 0.001 - 15
    }

    setTimeBase (base) {
        this.timebase = base
    }

    findTimeBase (gps) {
        const temp = this.gpstimetoTime(parseInt(gps['GWk']), parseInt(gps['GMS']))
        this.setTimeBase(parseInt(temp - gps['TimeUS'] * 0.000001))
    }

    getMsgType (element) {
        for (let i = 0; i < this.FMT.length; i++) {
            if (this.FMT[i] != null) {
                // eslint-disable-next-line
                if (this.FMT[i].Name == element) {
                    return i
                }
            }
        }
    }

    onMessage (message) {
        if (this.totalSize == null) { // for percentage calculation
            this.totalSize = this.buffer.byteLength
        }
        if (message.name in this.messages) {
            this.messages[message.name].push(this.fixData(message))
        } else {
            this.messages[message.name] = [this.fixData(message)]
        }
        let percentage = 100 * this.offset / this.totalSize
        if ((percentage - this.lastPercentage) > this.maxPercentageInterval) {
            self.postMessage({percentage: percentage})
            this.lastPercentage = percentage
        }
    }

    messageHasInstances (name) {
        let type = this.FMT.find(msg => msg !== undefined && msg.Name === name)
        return type !== undefined && type.units !== undefined && type.units.includes('instance')
    }

    getInstancesFieldName (name) {
        let type = this.FMT.find(msg => msg !== undefined && msg.Name === name)
        if (type.units === undefined) {
            return null
        }
        return type.Columns.split(',')[type.units.indexOf('instance')]
    }

    // Next three functions are used for transfering data on postmessage, instead of cloning
    isTypedArray (arr) {
        return ArrayBuffer.isView(arr) && !(arr instanceof DataView)
    }

    getType (arr) {
        return this.isTypedArray(arr) && arr.constructor.name
    }

    postData (data) {
        data['dataType'] = {}
        const transferables = []
        for (let field of Object.keys(data.messageList)) {
            const arrayType = this.getType(data.messageList[field])
            if (arrayType) {
                transferables.push(data.messageList[field].buffer)
            }
            // Apparently it is magically decoded on the other end, no need for metadata
            // data['dataType'][field] = arrayType
        }
        self.postMessage(data, transferables)
    }

    parseAtOffset (name) {
        let type = this.getMsgType(name)
        var parsed = []
        for (var i = 0; i < this.msgType.length; i++) {
            if (type === this.msgType[i]) {
                this.offset = this.offsetArray[i]
                try {
                    let temp = this.FORMAT_TO_STRUCT(this.FMT[this.msgType[i]])
                    if (temp['name'] != null) {
                        parsed.push(this.fixData(temp))
                    }
                } catch (e) {
                    console.log('reached log end?')
                    console.log(e)
                }
            }
            if (i % 100000 === 0) {
                let perc = 100 * i / this.msgType.length
                self.postMessage({percentage: perc})
            }
        }
        delete this.messages[name]
        this.messages[name] = parsed

        self.postMessage({percentage: 100})
        console.log(name, this.messageHasInstances(name) ? 'has instances' : 'has no instances')
        if (parsed.length && this.messageHasInstances(name)) {
            let instanceField = this.getInstancesFieldName(name)
            let instances = {}
            for (let msg of parsed) {
                try {
                    instances[msg[instanceField]].push(msg)
                } catch (e) {
                    instances[msg[instanceField]] = [ msg ]
                }
            }
            if (Object.keys(instances).length === 1) {
                this.fixDataOnce(name)
                this.simplifyData(name)
                this.postData({messageType: name, messageList: this.messages[name]})
                return parsed
            }
            for (let [index, messages] of Object.entries(instances)) {
                let newName = name + '[' + index + ']'
                this.messages[newName] = messages
                this.fixDataOnce(newName)
                this.simplifyData(newName)
                this.postData({messageType: newName,
                    messageList: this.messages[newName]})
            }
        } else if (parsed.length) {
            this.fixDataOnce(name)
            this.simplifyData(name)
            this.postData({messageType: name, messageList: this.messages[name]})
        }
        this.alreadyParsed.push(name)
        return parsed
    }

    checkNumberOfInstances (name) {
        // Similar to parseOffset, but finishes earlier and updates messageTypes
        let type = this.getMsgType(name)
        let availableInstances = []
        let instanceField = this.getInstancesFieldName(name)
        if (instanceField === null) {
            return [1]
        }
        let repeats = 0
        for (var i = 0; i < this.msgType.length; i++) {
            if (type === this.msgType[i]) {
                this.offset = this.offsetArray[i]
                try {
                    let temp = this.FORMAT_TO_STRUCT(this.FMT[this.msgType[i]])
                    if (temp['name'] != null) {
                        let msg = temp
                        if (!msg.hasOwnProperty(instanceField)) {
                            break
                        }
                        // we do an early return after we get 20 repeated instance numbers. should we?
                        const instance = msg[instanceField]
                        if (availableInstances.includes(instance)) {
                            repeats += 1
                            if (repeats > 20) {
                                return availableInstances
                            }
                        } else {
                            availableInstances.push(instance)
                        }
                    }
                } catch (e) {
                    console.log(e)
                }
            }
        }
        return availableInstances
    }

    timestamp (TimeUs) {
        let temp = this.timebase + TimeUs * 0.000001
        if (temp > 0) {
            TimeUs = temp
        }
        let date = new Date(TimeUs * 1000)
        let hours = date.getHours()
        let minutes = '0' + date.getMinutes()
        let seconds = '0' + date.getSeconds()
        let formattedTime = hours + ':' + minutes.substr(-2) + ':' + seconds.substr(-2)
        date = date.toString()
        let time = date.split(' ')
        if (time[0] !== 'Invalid') {
            this.time = time[0] + ' ' + time[1] + ' ' + time[2] + ' ' + time[3]
        }
        return formattedTime
    }

    DfReader () {
        let lastOffset = 0
        while (this.offset < (this.buffer.byteLength - 3)) {
            if (this.data.getUint8(this.offset) !== HEAD1 || this.data.getUint8(this.offset + 1) !== HEAD2) {
                this.offset += 1
                continue
            }
            this.offset += 2

            let attribute = this.data.getUint8(this.offset)
            if (this.FMT[attribute] != null) {
                this.offset += 1
                this.offsetArray.push(this.offset)
                this.msgType.push(attribute)
                try {
                    var value = this.FORMAT_TO_STRUCT(this.FMT[attribute])
                    if (this.FMT[attribute].Name === 'GPS') {
                        this.findTimeBase(value)
                    }
                } catch (e) {
                    // console.log('reached log end?')
                    // console.log(e)
                    this.offset += 1
                }
                if (attribute === 128) {
                    this.FMT[value['Type']] = {
                        'Type': value['Type'],
                        'length': value['Length'],
                        'Name': value['Name'],
                        'Format': value['Format'],
                        'Columns': value['Columns']
                    }
                }
                // this.onMessage(value)
            } else {
                this.offset += 1
            }
            if (this.offset - lastOffset > 50000) {
                let perc = 100 * this.offset / this.buffer.byteLength
                self.postMessage({percentage: perc})
                lastOffset = this.offset
            }
        }
        self.postMessage({percentage: 100})
        self.postMessage({messages: this.messages})
        this.sent = true
    }

    getModeString (cmode) {
        let mavtype
        let msgs = this.messages['MSG']
        for (let i in msgs.Message) {
            if (msgs.Message[i].toLowerCase().includes('arduplane')) {
                mavtype = MAV_TYPE_FIXED_WING
                return getModeMap(mavtype)[cmode]
            } else if (msgs.Message[i].toLowerCase().includes('arducopter')) {
                mavtype = MAV_TYPE_QUADROTOR
                return getModeMap(mavtype)[cmode]
            } else if (msgs.Message[i].toLowerCase().includes('ardusub')) {
                mavtype = MAV_TYPE_SUBMARINE
                return getModeMap(mavtype)[cmode]
            } else if (msgs.Message[i].toLowerCase().includes('rover')) {
                mavtype = MAV_TYPE_GROUND_ROVER
                return getModeMap(mavtype)[cmode]
            } else if (msgs.Message[i].toLowerCase().includes('tracker')) {
                mavtype = MAV_TYPE_ANTENNA_TRACKER
                return getModeMap(mavtype)[cmode]
            }
        }
        console.log('defaulting to quadcopter')
        return getModeMap(MAV_TYPE_QUADROTOR)[cmode]
    }

    fixData (message) {
        if (message.name === 'MODE') {
            message.asText = this.getModeString(message['Mode'])
        }
        // eslint-disable-next-line
        message.time_boot_ms = message.TimeUS / 1000
        delete message.TimeUS
        delete message.fieldnames
        return message
    }

    fixDataOnce (name) {
        if (!['GPS', 'ATT', 'AHR2', 'MODE'].includes(name)) {
            if (this.messageTypes.hasOwnProperty(name)) {
                let fields = this.messages[name][0].fieldnames
                if (this.messageTypes[name].hasOwnProperty('multipliers')) {
                    for (let message in this.messages[name]) {
                        for (let i = 1; i < fields.length; i++) {
                            let fieldname = fields[i]
                            if (!isNaN(this.messageTypes[name].multipliers[i])) {
                                this.messages[name][message][fieldname] *= this.messageTypes[name].multipliers[i]
                            }
                        }
                    }
                }
            }
        }
    }

    concatTypedArrays (a, b) { // a, b TypedArray of same type
        var c = new (a.constructor)(a.length + b.length)
        c.set(a, 0)
        c.set(b, a.length)
        return c
    }

    createUint8ArrayFromString (str) {
        const array = new Uint8Array(str.length)
        for (let i = 0, strLen = str.length; i < strLen; i++) {
            array[i] = str.charCodeAt(i)
        }
        return array
    }

    processFiles () {
        this.files = {}
        for (let msg of this.messages['FILE']) {
            if (!this.files.hasOwnProperty(msg.FileName)) {
                this.files[msg.FileName] = this.createUint8ArrayFromString(msg.Data)
            } else {
                this.files[msg.FileName] = this.concatTypedArrays(
                    this.files[msg.FileName], this.createUint8ArrayFromString(msg.Data)
                )
            }
        }
        self.postMessage({files: this.files})
    }

    simplifyData (name) {
        if (name === 'MODE') {
            this.messageTypes[name].expressions.push('asText')
        }
        if (name === 'FILE') {
            return
        }
        if (!['FMTU'].includes(name)) {
            if (this.messageTypes.hasOwnProperty(name)) {
                let fields = this.messageTypes[name].expressions
                if (!fields.includes('time_boot_ms')) {
                    fields.push('time_boot_ms')
                }
                let mergedData = {}
                for (let field of fields) {
                    mergedData[field] = []
                }
                for (let message of this.messages[name]) {
                    for (let i = 1; i < fields.length; i++) {
                        let fieldname = fields[i]
                        mergedData[fieldname].push(message[fieldname])
                    }
                }
                delete this.messages[name]
                this.messages[name] = mergedData
                for (const field of this.messageTypes[name].expressions) {
                    if (this.messages[name][field] && !isNaN(this.messages[name][field][0])) {
                        const newData = new Float64Array(this.messages[name][field])
                        delete this.messages[name][field]
                        this.messages[name][field] = newData
                    }
                }
            }
        }
    }

    populateUnits () {
        // console.log(this.messages['FMTU'])
        for (let msg of this.messages['FMTU']) {
            this.FMT[msg.FmtType]['units'] = []
            for (let unit of msg.UnitIds) {
                this.FMT[msg.FmtType]['units'].push(units[unit])
            }
            this.FMT[msg.FmtType]['multipliers'] = []
            for (let mult of msg.MultIds) {
                this.FMT[msg.FmtType]['multipliers'].push(multipliers[mult])
            }
        }
    }

    extractStartTime () {
        let msgs = this.messages['GPS']
        for (let i in msgs.time_boot_ms) {
            if (msgs.GWk[i] > 1000) { // lousy validation
                let weeks = msgs.GWk[i]
                let ms = msgs.GMS[i]
                let d = new Date((315964800.0 + ((60 * 60 * 24 * 7) * weeks) + ms / 1000.0) * 1000.0)
                // adjusting for leap seconds
                d = new Date(d.getTime() - this.leapSecondsGPS(d.getUTCFullYear(), d.getUTCMonth() + 1) * 1000)
                return d
            }
        }
    }

    leapSecondsGPS (year, month) {
        return this.leapSecondsTAI(year, month) - 19
    }

    leapSecondsTAI (year, month) {
        const yyyymm = year * 100 + month
        if (yyyymm >= 201701) return 37
        if (yyyymm >= 201507) return 36
        if (yyyymm >= 201207) return 35
        if (yyyymm >= 200901) return 34
        if (yyyymm >= 200601) return 33
        if (yyyymm >= 199901) return 32
        if (yyyymm >= 199707) return 31
        if (yyyymm >= 199601) return 30

        return 0
    }

    processData (data) {
        this.buffer = data
        this.data = new DataView(this.buffer)
        this.DfReader()
        let messageTypes = {}
        this.parseAtOffset('FMTU')
        this.populateUnits()
        let typeSet = new Set(this.msgType)
        for (let msg of this.FMT) {
            if (msg) {
                if (typeSet.has(msg.Type)) {
                    let fields = msg.Columns.split(',')
                    // expressions = expressions.filter(e => e !== 'TimeUS')
                    let complexFields = {}
                    if (msg.hasOwnProperty('units')) {
                        for (let field in fields) {
                            complexFields[fields[field]] = {
                                name: fields[field],
                                units: (multipliersTable[msg.multipliers[field]] || '') + msg.units[field],
                                multiplier: msg.multipliers[field]
                            }
                        }
                    } else {
                        for (let field in fields) {
                            complexFields[fields[field]] = {
                                name: fields[field],
                                units: '?',
                                multiplier: 1
                            }
                        }
                    }
                    let availableInstances = this.checkNumberOfInstances(msg.Name)
                    if (availableInstances.length > 1) {
                        for (let instance of availableInstances) {
                            messageTypes[msg.Name + '[' + instance + ']'] = {
                                expressions: fields,
                                units: msg.units,
                                multipiers: msg.multipliers,
                                complexFields: complexFields
                            }
                        }
                    } else {
                        messageTypes[msg.Name] = {
                            expressions: fields,
                            units: msg.units,
                            multipiers: msg.multipliers,
                            complexFields: complexFields
                        }
                    }
                }
            }
        }
        self.postMessage({availableMessages: messageTypes})
        this.messageTypes = messageTypes
        this.parseAtOffset('CMD')
        this.parseAtOffset('MSG')
        this.parseAtOffset('FILE')
        this.processFiles()
        this.parseAtOffset('MODE')
        this.parseAtOffset('AHR2')
        this.parseAtOffset('ATT')
        this.parseAtOffset('GPS')
        this.parseAtOffset('POS')
        this.parseAtOffset('XKQ1')
        this.parseAtOffset('XKQ')
        this.parseAtOffset('NKQ1')
        this.parseAtOffset('NKQ2')
        this.parseAtOffset('XKQ2')
        this.parseAtOffset('PARM')
        this.parseAtOffset('MSG')
        this.parseAtOffset('STAT')
        this.parseAtOffset('EV')
        let metadata = {
            startTime: this.extractStartTime()
        }
        self.postMessage({metadata: metadata})

        self.postMessage({messagesDoneLoading: true})
        return {types: this.messageTypes, messages: this.messages}
    }

    loadType (type) {
        this.parseAtOffset(type)
        console.log('done')
    }
}

self.addEventListener('message', function (event) {
    if (event.data === null) {
        console.log('got bad file message!')
    } else if (event.data.action === 'parse') {
        parser = new DataflashParser()
        let data = event.data.file
        parser.processData(data)
    } else if (event.data.action === 'loadType') {
        parser.loadType(event.data.type.split('[')[0])
    } else if (event.data.action === 'trimFile') {
        parser.trimFile(event.data.time)
    }
})
