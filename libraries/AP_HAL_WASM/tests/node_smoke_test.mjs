import { pathToFileURL } from 'node:url';

const modulePath = process.argv[2];
if (modulePath === undefined) {
    throw new Error('usage: node_smoke_test.mjs <arduplane.js>');
}

const { default: createModule } = await import(pathToFileURL(modulePath));
const module = await createModule({
    arguments: ['--model', 'plane'],
    print: console.log,
    printErr: console.error,
});

const malloc = module.cwrap('ardupilot_malloc', 'number', ['number']);
const read = module.cwrap('ardupilot_serial0_read', 'number', ['number', 'number']);
const bufferSize = 4096;
const buffer = malloc(bufferSize);
const deadline = Date.now() + 15000;

while (Date.now() < deadline) {
    const length = read(buffer, bufferSize);
    if (module.HEAPU8.subarray(buffer, buffer + length).includes(0xfd)) {
        console.log('Received MAVLink data from ArduPlane WebAssembly SITL');
        process.exit(0);
    }
    await new Promise(resolve => setTimeout(resolve, 10));
}

throw new Error('timed out waiting for MAVLink data from ArduPlane WebAssembly SITL');