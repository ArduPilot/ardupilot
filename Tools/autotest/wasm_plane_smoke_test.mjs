import { pathToFileURL } from 'node:url';

const modulePath = process.argv[2];
const model = process.argv[3] ?? 'plane';
if (modulePath === undefined || !['plane', 'JSON'].includes(model)) {
    throw new Error('usage: wasm_plane_smoke_test.mjs <arduplane.js> [plane|JSON]');
}

let jsonReceived = false;
const { default: createModule } = await import(pathToFileURL(modulePath));
const module = await createModule({
    arguments: [
        '--model', model,
        '--serial0', 'wasm',
        '--serial1', 'none',
        '--serial2', 'none',
    ],
    print: (text) => {
        jsonReceived ||= text.includes('JSON received:');
        console.log(text);
    },
    printErr: console.error,
});

const malloc = module.cwrap('ardupilot_malloc', 'number', ['number']);
const read = module.cwrap('ardupilot_serial_read', 'number', ['number', 'number', 'number']);
const serialPort = 0;
const bufferSize = 4096;
const buffer = malloc(bufferSize);
const deadline = Date.now() + 15000;
const sleep = () => new Promise(resolve => setTimeout(resolve, 10));

function receivedMAVLink() {
    const length = read(serialPort, buffer, bufferSize);
    return module.HEAPU8.subarray(buffer, buffer + length).includes(0xfd);
}

if (model === 'plane') {
    while (Date.now() < deadline) {
        if (receivedMAVLink()) {
            console.log('Received MAVLink data from ArduPlane WebAssembly SITL');
            process.exit(0);
        }
        await sleep();
    }
    throw new Error('timed out waiting for MAVLink data from ArduPlane WebAssembly SITL');
}

/*
  act as a JSON physics backend in lockstep: answer every servo packet
  with one step of a vehicle at rest, through the buffers the wasm build
  uses in place of UDP
 */
const SERVO_PACKET_MAGIC = { 18458: 16, 29569: 32 };
const SERVO_PACKET_HEADER_SIZE = 8;
const FRAMES_NEEDED = 100;
const writeSensors = module.cwrap('ardupilot_json_sensors_write', 'number', ['number', 'number']);
const readServos = module.cwrap('ardupilot_json_servos_read', 'number', ['number', 'number']);
const servosBuffer = malloc(bufferSize);
const sensorsBuffer = malloc(bufferSize);
const textEncoder = new TextEncoder();
let timestamp = 0;
let firstFrame;
let lastFrame;
let mavlink = false;

while (Date.now() < deadline) {
    mavlink ||= receivedMAVLink();
    const length = readServos(servosBuffer, bufferSize);
    if (length >= SERVO_PACKET_HEADER_SIZE) {
        const packets = new DataView(module.HEAPU8.buffer, servosBuffer, length);
        const channels = SERVO_PACKET_MAGIC[packets.getUint16(0, true)];
        if (channels === undefined) {
            throw new Error(`unexpected servo packet magic ${packets.getUint16(0, true)}`);
        }
        const packetSize = SERVO_PACKET_HEADER_SIZE + channels * 2;
        const newest = (Math.floor(length / packetSize) - 1) * packetSize;
        const frameRate = packets.getUint16(newest + 2, true);
        const frameCount = packets.getUint32(newest + 4, true);
        if (lastFrame !== undefined && frameCount < lastFrame) {
            throw new Error(`servo frame_count went backwards: ${lastFrame} -> ${frameCount}`);
        }
        firstFrame ??= frameCount;
        lastFrame = frameCount;

        timestamp += 1 / (frameRate || 400);
        const sensors = textEncoder.encode(JSON.stringify({
            timestamp,
            imu: { gyro: [0, 0, 0], accel_body: [0, 0, -9.80665] },
            position: [0, 0, 0],
            attitude: [0, 0, 0],
            velocity: [0, 0, 0],
        }) + '\n');
        module.HEAPU8.set(sensors, sensorsBuffer);
        if (writeSensors(sensorsBuffer, sensors.length) !== sensors.length) {
            throw new Error('JSON sensor buffer full');
        }

        if (jsonReceived && mavlink && lastFrame - firstFrame >= FRAMES_NEEDED) {
            console.log(`ArduPlane WebAssembly SITL ran ${lastFrame - firstFrame} JSON frames and sent MAVLink`);
            process.exit(0);
        }
        continue;
    }
    await sleep();
}

throw new Error(`timed out running ArduPlane WebAssembly SITL with JSON physics: ` +
    `JSON received ${jsonReceived}, MAVLink ${mavlink}, frames ${firstFrame}..${lastFrame}`);
