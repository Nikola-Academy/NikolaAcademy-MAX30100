// max30100.ts
//% color=#ff2f92 icon="\uf21e" block="MAX30100"
namespace max30100 {
  const I2C_ADDR = 0x57
  const REG_INT_STATUS = 0x00
  const REG_INT_ENABLE = 0x01
  const REG_FIFO_WR_PTR = 0x02
  const REG_OVF_COUNTER = 0x03
  const REG_FIFO_RD_PTR = 0x04
  const REG_FIFO_DATA = 0x05
  const REG_MODE_CONFIG = 0x06
  const REG_SPO2_CONFIG = 0x07
  const REG_LED_CONFIG = 0x09

  const MODE_HR = 0x02
  const MODE_SPO2 = 0x03

  export enum SampleRate {
      SR50 = 0x00, SR100 = 0x01, SR167 = 0x02, SR200 = 0x03, SR400 = 0x04, SR600 = 0x05, SR800 = 0x06, SR1000 = 0x07
  }
  export enum PulseWidth {
      PW200uS = 0x00, PW400uS = 0x01, PW800uS = 0x02, PW1600uS = 0x03
  }
  export enum LedCurrent {
      mA0 = 0x00, mA4_4 = 0x01, mA7_6 = 0x02, mA11 = 0x03, mA14_2 = 0x04, mA17_4 = 0x05, mA20_8 = 0x06, mA24 = 0x07,
      mA27_1 = 0x08, mA30_6 = 0x09, mA33_8 = 0x0A, mA37 = 0x0B, mA40_2 = 0x0C, mA43_6 = 0x0D, mA46_8 = 0x0E, mA50 = 0x0F
  }

  function writeReg(reg: number, val: number) {
      const buf = pins.createBuffer(2)
      buf[0] = reg
      buf[1] = val & 0xFF
      pins.i2cWriteBuffer(I2C_ADDR, buf)
  }

  function readReg(reg: number): number {
      pins.i2cWriteNumber(I2C_ADDR, reg, NumberFormat.UInt8BE)
      return pins.i2cReadNumber(I2C_ADDR, NumberFormat.UInt8BE)
  }

  function clearInterrupts() {
      // Read INT STATUS to clear any pending flags
      readReg(REG_INT_STATUS)
  }

  function waitPowerReady(timeoutMs: number): boolean {
      const start = control.millis()
      while (control.millis() - start < timeoutMs) {
          // PWR_RDY bit is bit0 of INT STATUS
          if (readReg(REG_INT_STATUS) & 0x01) return true
          basic.pause(1)
      }
      return false
  }

  function resetFifo() {
      writeReg(REG_FIFO_WR_PTR, 0x00)
      writeReg(REG_OVF_COUNTER, 0x00)
      writeReg(REG_FIFO_RD_PTR, 0x00)
  }

  function samplesAvailable(): number {
      const wr = readReg(REG_FIFO_WR_PTR) & 0x0F
      const rd = readReg(REG_FIFO_RD_PTR) & 0x0F
      return (wr - rd) & 0x0F
  }

  function readFIFOBurst(count: number): { ir: number, red: number }[] {
      if (count <= 0) return []
      pins.i2cWriteNumber(I2C_ADDR, REG_FIFO_DATA, NumberFormat.UInt8BE)
      const b = pins.i2cReadBuffer(I2C_ADDR, 4 * count)
      const out: { ir: number, red: number }[] = []
      for (let i = 0; i < count; i++) {
          const base = i * 4
          const ir = (b[base] << 8) | b[base + 1]
          const red = (b[base + 2] << 8) | b[base + 3]
          out.push({ ir: ir, red: red })
      }
      return out
  }

  let _onSample: (ir: number, red: number) => void = null
  let _running = false

  //% blockId=max30100_begin block="MAX30100 begin at %rate|Hz, pulse %pw|, IR %ir|, RED %red"
  //% rate.defl=SampleRate.SR100
  //% pw.defl=PulseWidth.PW1600uS
  //% ir.defl=LedCurrent.mA50
  //% red.defl=LedCurrent.mA50
  export function begin(rate: SampleRate = SampleRate.SR100, pw: PulseWidth = PulseWidth.PW1600uS, ir: LedCurrent = LedCurrent.mA50, red: LedCurrent = LedCurrent.mA50) {
      writeReg(REG_MODE_CONFIG, 0x40) // reset
      basic.pause(10)
      waitPowerReady(100)
      writeReg(REG_INT_ENABLE, 0x00) // disable all interrupts
      clearInterrupts()
      resetFifo()
      let spo2cfg = 0
      spo2cfg |= 0x40 // high-res
      spo2cfg |= (rate & 0x07) << 2
      spo2cfg |= (pw & 0x03)
      writeReg(REG_SPO2_CONFIG, spo2cfg)
      // Datasheet/Arduino mapping: upper nibble = RED, lower nibble = IR
      writeReg(REG_LED_CONFIG, ((red & 0x0F) << 4) | (ir & 0x0F))
      writeReg(REG_MODE_CONFIG, MODE_SPO2)
  }

  //% blockId=max30100_onSample block="on MAX30100 sample"
  export function onSample(handler: (ir: number, red: number) => void) {
      _onSample = handler
      if (!_running) {
          _running = true
          control.inBackground(() => {
              let firstSeen = false
              while (_running) {
                  const n = samplesAvailable()
                  if (n > 0) {
                      const samples = readFIFOBurst(n)
                      for (let i = 0; i < samples.length; i++) {
                          const s = samples[i]
                          if (!firstSeen && (s.ir > 0 || s.red > 0)) firstSeen = true
                          if (_onSample) _onSample(s.ir, s.red)
                      }
                  } else {
                      basic.pause(firstSeen ? 5 : 1)
                  }
              }
          })
      }
  }

  //% blockId=max30100_stop block="stop MAX30100"
  export function stop() { _running = false }
}