## MAX30100 for micro:bit (MakeCode)

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


## Quickstart (Blocks)
```blocks
max30100.begin(max30100.SampleRate.SR100, max30100.PulseWidth.PW1600uS, max30100.LedCurrent.mA11, max30100.LedCurrent.mA11)
max30100.onSample(function (ir: number, red: number) {
    basic.showNumber(ir) // or process values
})
```

## Quickstart (TypeScript)
```typescript
max30100.begin(
    max30100.SampleRate.SR100,
    max30100.PulseWidth.PW1600uS,
    max30100.LedCurrent.mA11,
    max30100.LedCurrent.mA11
)

max30100.onSample((ir, red) => {
    serial.writeNumbers([ir, red])
})

// later, if needed
// max30100.stop()
```


## API
- `max30100.begin(rate?: SampleRate, pw?: PulseWidth, ir?: LedCurrent, red?: LedCurrent): void`  
  Initializes the sensor (400 kHz I²C, SpO₂ mode).
  - `SampleRate`: `SR50`, `SR100`, `SR167`, `SR200`, `SR400`, `SR600`, `SR800`, `SR1000`
  - `PulseWidth`: `PW200uS`, `PW400uS`, `PW800uS`, `PW1600uS`
  - `LedCurrent`: `mA0` … `mA50`
- `max30100.onSample(handler: (ir: number, red: number) => void): void`  
  Sets a background sampler callback (~every 10 ms).
- `max30100.stop(): void`  
  Stops the background sampler.

Notes:
- Start with `SR100` and `PW1600uS` for stable readings.
- If values clamp near 0 or 1023/2047, adjust LED current (e.g., `mA7_6` to `mA20_8`).
- Use `serial.writeNumbers([ir, red])` to log and tune.


## Roadmap (optional)
- Heartbeat detection
- SpO₂ estimation
- On-beat events and HR/SpO₂ getters


## Troubleshooting
- No readings:
  - Check wiring (SDA=P20, SCL=P19), power at 3.3V, address `0x57`.
  - Ensure only one I²C device with the same address on the bus.
- Noisy/saturated readings:
  - Lower LED current or ensure proper finger placement and ambient light shielding.
- Build errors:
  - Update MakeCode and remove conflicting extensions.


## Credits
- Based on the MAX30100 register map and public Arduino libraries.  
  Original Arduino reference: `https://github.com/oxullo/Arduino-MAX30100`


## License

MIT



## Supported targets

- for PXT/microbit
  (The metadata above is needed for package search.)