import { pathToFileURL } from 'node:url';

const [modulePath, model] = process.argv.slice(2);
if (model === undefined) {
    throw new Error('usage: wasm_sitl_smoke_test.mjs <vehicle.js> <model>');
}

const { default: createModule } = await import(pathToFileURL(modulePath));
const module = await createModule({
    arguments: [
        '--model', model,
        '--serial0', 'wasm',
        '--serial1', 'none',
        '--serial2', 'none',
    ],
    print: console.log,
    printErr: console.error,
});

const malloc = module.cwrap('ardupilot_malloc', 'number', ['number']);
const read = module.cwrap('ardupilot_serial_read', 'number', ['number', 'number', 'number']);
const serialPort = 0;
const bufferSize = 4096;
const buffer = malloc(bufferSize);
const deadline = Date.now() + 15000;

while (Date.now() < deadline) {
    const length = read(serialPort, buffer, bufferSize);
    if (module.HEAPU8.subarray(buffer, buffer + length).includes(0xfd)) {
        console.log(`Received MAVLink data from ${model} WebAssembly SITL`);
        process.exit(0);
    }
    await new Promise(resolve => setTimeout(resolve, 10));
}

throw new Error(`timed out waiting for MAVLink data from ${model} WebAssembly SITL`);
