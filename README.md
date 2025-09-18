## MAX30100 for micro:bit (MakeCode) By Nikola Academy

MAX30100 Pulse Oximeter driver for BBC micro:bit using MakeCode.  
Driver initializes the sensor, reads raw IR/RED data from FIFO, and exposes simple blocks and TypeScript APIs.


## Features
- Initialize MAX30100 (SpO₂ mode, sample rate, pulse width, LED currents)
- Read raw IR/RED samples in background
- Easy MakeCode blocks and TypeScript API
- Designed to extend later with heart rate and SpO₂ algorithms


## Hardware
- Sensor: MAX30100, I²C address `0x57`
- Voltage: 3.3V only (micro:bit compatible)

| MAX30100 | micro:bit |
|----------|-----------|
| SDA      | P20       |
| SCL      | P19       |
| VCC      | 3V        |
| GND      | GND       |

Tip: Use short wires. Start with moderate LED current to avoid sensor saturation.


## Install (MakeCode)
- Open MakeCode for micro:bit.
- Project → Extensions → paste this GitHub repo URL (e.g. `https://github.com/Nikola-Academy/NikolaAcademy-MAX30100`) and add.
- The “MAX30100” toolbox category will appear.

## Quickstart (TypeScript)
```typescript
max30100.start()
basic.forever(function () {
    // It needs to collect data, so it will show 0 on the first print
    basic.showString("H:" + max30100.getHeartRate() + " O:" + max30100.getSpO2())
    basic.pause(1000)
})

// later, if needed
// max30100.stop()
```

## Credits
- Based on the MAX30100 register map and public Arduino libraries.  
  Original Arduino reference: `https://github.com/oxullo/Arduino-MAX30100`


## License

MIT

## Supported targets

- for PXT/microbit
  (The metadata above is needed for package search.)