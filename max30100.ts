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
  const REG_TEMP_INT = 0x16
  const REG_TEMP_FRAC = 0x17
  const REG_REV_ID = 0xFE
  const REG_PART_ID = 0xFF

  const MODE_HR = 0x02
  const MODE_SPO2 = 0x03

  // Processing constants (mirroring Arduino library)
  const DC_REMOVER_ALPHA = 0.95
  const BEATDETECTOR_INIT_HOLDOFF = 2000
  const BEATDETECTOR_MASKING_HOLDOFF = 200
  const BEATDETECTOR_BPFILTER_ALPHA = 0.6
  let BEATDETECTOR_MIN_THRESHOLD = 20
  let BEATDETECTOR_MAX_THRESHOLD = 800
  let BEATDETECTOR_STEP_RESILIENCY = 30
  const BEATDETECTOR_THRESHOLD_FALLOFF_TARGET = 0.3
  const BEATDETECTOR_THRESHOLD_DECAY_FACTOR = 0.99
  const BEATDETECTOR_SAMPLES_PERIOD = 10

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
      pins.i2cWriteNumber(I2C_ADDR, reg, NumberFormat.UInt8BE, true)
      return pins.i2cReadNumber(I2C_ADDR, NumberFormat.UInt8BE)
  }

  function clearInterrupts() {
      // Read INT STATUS to clear any pending flags
      readReg(REG_INT_STATUS)
  }

  // Minimal reset polling; device clears RESET quickly
  function waitResetClear(timeoutMs: number): boolean {
      const start = control.millis()
      while (control.millis() - start < timeoutMs) {
          if ((readReg(REG_MODE_CONFIG) & 0x40) === 0) return true
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
      pins.i2cWriteNumber(I2C_ADDR, REG_FIFO_DATA, NumberFormat.UInt8BE, true)
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

  // No priming needed in the regular stable version

  let _onSample: (ir: number, red: number) => void = null
  let _running = false
  let _lastSampleMs = 0
  let _onBeat: () => void = null

  // Processing components and outputs
  class FilterBuLp1 {
      private v0: number
      private v1: number
      constructor() { this.v0 = 0; this.v1 = 0 }
      step(x: number): number {
          this.v0 = this.v1
          this.v1 = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * this.v0)
          return this.v0 + this.v1
      }
  }

  class DCRemover {
      private alpha: number
      private dcw: number
      constructor(alpha: number) { this.alpha = alpha; this.dcw = 0 }
      step(x: number): number {
          const olddcw = this.dcw
          this.dcw = x + this.alpha * this.dcw
          return this.dcw - olddcw
      }
      getDCW(): number { return this.dcw }
  }

  enum BeatDetectorState {
      INIT, WAITING, FOLLOWING_SLOPE, MAYBE_DETECTED, MASKING
  }

  class BeatDetector {
      private state: BeatDetectorState
      private threshold: number
      private beatPeriod: number
      private lastMaxValue: number
      private tsLastBeat: number
      constructor() {
          this.state = BeatDetectorState.INIT
          this.threshold = BEATDETECTOR_MIN_THRESHOLD
          this.beatPeriod = 0
          this.lastMaxValue = 0
          this.tsLastBeat = 0
      }
      addSample(sample: number): boolean { return this.checkForBeat(sample) }
      getRate(): number { return this.beatPeriod !== 0 ? (1 / this.beatPeriod) * 1000 * 60 : 0 }
      getCurrentThreshold(): number { return this.threshold }
      private checkForBeat(sample: number): boolean {
          let beatDetected = false
          switch (this.state) {
              case BeatDetectorState.INIT:
                  if (control.millis() > BEATDETECTOR_INIT_HOLDOFF) {
                      this.state = BeatDetectorState.WAITING
                  }
                  break
              case BeatDetectorState.WAITING:
                  if (sample > this.threshold) {
                      this.threshold = Math.min(sample, BEATDETECTOR_MAX_THRESHOLD)
                      this.state = BeatDetectorState.FOLLOWING_SLOPE
                  }
                  if (control.millis() - this.tsLastBeat > 2000) {
                      this.beatPeriod = 0
                      this.lastMaxValue = 0
                  }
                  this.decreaseThreshold()
                  break
              case BeatDetectorState.FOLLOWING_SLOPE:
                  if (sample < this.threshold) {
                      this.state = BeatDetectorState.MAYBE_DETECTED
                  } else {
                      this.threshold = Math.min(sample, BEATDETECTOR_MAX_THRESHOLD)
                  }
                  break
              case BeatDetectorState.MAYBE_DETECTED:
                  if (sample + BEATDETECTOR_STEP_RESILIENCY < this.threshold) {
                      beatDetected = true
                      this.lastMaxValue = sample
                      this.state = BeatDetectorState.MASKING
                      const delta = control.millis() - this.tsLastBeat
                      if (delta) {
                          this.beatPeriod = BEATDETECTOR_BPFILTER_ALPHA * delta + (1 - BEATDETECTOR_BPFILTER_ALPHA) * this.beatPeriod
                      }
                      this.tsLastBeat = control.millis()
                  } else {
                      this.state = BeatDetectorState.FOLLOWING_SLOPE
                  }
                  break
              case BeatDetectorState.MASKING:
                  if (control.millis() - this.tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF) {
                      this.state = BeatDetectorState.WAITING
                  }
                  this.decreaseThreshold()
                  break
          }
          return beatDetected
      }
      private decreaseThreshold() {
          if (this.lastMaxValue > 0 && this.beatPeriod > 0) {
              this.threshold -= this.lastMaxValue * (1 - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET) / (this.beatPeriod / BEATDETECTOR_SAMPLES_PERIOD)
          } else {
              this.threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR
          }
          if (this.threshold < BEATDETECTOR_MIN_THRESHOLD) this.threshold = BEATDETECTOR_MIN_THRESHOLD
          if (this.threshold > BEATDETECTOR_MAX_THRESHOLD) this.threshold = BEATDETECTOR_MAX_THRESHOLD
      }
  }

  class SpO2Calculator {
      private static spO2LUT: number[] = [100,100,100,100,99,99,99,99,99,99,98,98,98,98,
          98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,95,95,95,95,94,94,94,94,94,93,93,93,93,93]
      private irACValueSqSum: number
      private redACValueSqSum: number
      private beatsDetectedNum: number
      private samplesRecorded: number
      private spO2: number
      constructor() { this.reset() }
      update(irACValue: number, redACValue: number, beatDetected: boolean) {
          this.irACValueSqSum += irACValue * irACValue
          this.redACValueSqSum += redACValue * redACValue
          this.samplesRecorded++
          if (beatDetected) {
              this.beatsDetectedNum++
              if (this.beatsDetectedNum === 3) {
                  const irMeanSq = this.irACValueSqSum / this.samplesRecorded
                  const redMeanSq = this.redACValueSqSum / this.samplesRecorded
                  let acSqRatio = 0
                  if (irMeanSq > 0 && redMeanSq > 0) {
                      acSqRatio = 100.0 * Math.log(redMeanSq) / Math.log(irMeanSq)
                  }
                  let index = 0
                  if (acSqRatio > 66) index = Math.floor(acSqRatio) - 66
                  else if (acSqRatio > 50) index = Math.floor(acSqRatio) - 50
                  if (index < 0) index = 0
                  if (index >= SpO2Calculator.spO2LUT.length) index = SpO2Calculator.spO2LUT.length - 1
                  this.spO2 = SpO2Calculator.spO2LUT[index]
                  this.reset()
              }
          }
      }
      reset() {
          this.irACValueSqSum = 0
          this.redACValueSqSum = 0
          this.beatsDetectedNum = 0
          this.samplesRecorded = 0
          this.spO2 = 0
      }
      getSpO2(): number { return this.spO2 }
  }

  let beatDetector = new BeatDetector()
  let irDCRemover = new DCRemover(DC_REMOVER_ALPHA)
  let redDCRemover = new DCRemover(DC_REMOVER_ALPHA)
  let lpf = new FilterBuLp1()
  let spO2calculator = new SpO2Calculator()
  let heartRateBpm = 0
  let spO2Percent = 0
  let lastFiltered = 0
  let lastThreshold = 0
  let highSensitivity = false
  let invertPulse = true // default matches Arduino path: use -irAC

  function resetProcessing() {
      beatDetector = new BeatDetector()
      irDCRemover = new DCRemover(DC_REMOVER_ALPHA)
      redDCRemover = new DCRemover(DC_REMOVER_ALPHA)
      lpf = new FilterBuLp1()
      spO2calculator = new SpO2Calculator()
      heartRateBpm = 0
      spO2Percent = 0
  }


  //% blockId=max30100_begin block="MAX30100 begin at %rate|Hz, pulse %pw|, IR %ir|, RED %red"
  //% rate.defl=SampleRate.SR100
  //% pw.defl=PulseWidth.PW1600uS
  //% ir.defl=LedCurrent.mA50
  //% red.defl=LedCurrent.mA50
  //% blockHidden=true
  export function begin(rate: SampleRate = SampleRate.SR100, pw: PulseWidth = PulseWidth.PW1600uS, ir: LedCurrent = LedCurrent.mA50, red: LedCurrent = LedCurrent.mA50) {
      writeReg(REG_MODE_CONFIG, 0x40) // reset
      basic.pause(5)
      waitResetClear(20)
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
      _lastSampleMs = control.millis()
      resetProcessing()
  }

  //% blockId=max30100_start block="start MAX30100" weight=100
  export function start() {
      begin(SampleRate.SR100, PulseWidth.PW1600uS, LedCurrent.mA50, LedCurrent.mA27_1)
  }

  //% blockId=max30100_onSample block="on MAX30100 sample" weight=90
  export function onSample(handler: (ir: number, red: number) => void) {
      _onSample = handler
      if (!_running) {
          _running = true
          control.inBackground(() => {
              while (_running) {
                  const n = samplesAvailable()
                  if (n > 0) {
                      const samples = readFIFOBurst(n)
                      for (let i = 0; i < samples.length; i++) {
                          const s = samples[i]
                          _lastSampleMs = control.millis()
                          // Processing chain similar to Arduino PulseOximeter
                          const irAC = irDCRemover.step(s.ir)
                          const redAC = redDCRemover.step(s.red)
                          const rawPulse = invertPulse ? -irAC : irAC
                          const filtered = lpf.step(rawPulse)
                          const beat = beatDetector.addSample(filtered)
                          lastFiltered = filtered
                          lastThreshold = beatDetector.getCurrentThreshold()
                          const rate = beatDetector.getRate()
                          if (rate > 0) heartRateBpm = rate
                          spO2calculator.update(irAC, redAC, beat)
                          const spo2 = spO2calculator.getSpO2()
                          if (spo2 > 0) spO2Percent = spo2
                          if (beat && _onBeat) _onBeat()
                          if (_onSample) _onSample(s.ir, s.red)
                      }
                  } else {
                      basic.pause(5)
                  }
              }
          })
      }
  }

  //% blockId=max30100_onBeat block="on MAX30100 beat" weight=85
  export function onBeatDetected(handler: () => void) { _onBeat = handler }

  //% blockId=max30100_getHeartRate block="MAX30100 heart rate (bpm)" weight=70
  export function getHeartRate(): number { return Math.round(heartRateBpm) }

  //% blockId=max30100_getSpO2 block="MAX30100 SpO2 (%)" weight=60
  export function getSpO2(): number { return spO2Percent }

  //% blockId=max30100_getPulseDebug block="MAX30100 pulse debug" blockHidden=true
  export function getPulseDebug(): { filtered: number, threshold: number } {
      return { filtered: lastFiltered, threshold: lastThreshold }
  }

  //% blockId=max30100_getIds block="MAX30100 get IDs" blockHidden=true
  //% weight=10
  export function getIds(): { partId: number, revisionId: number } {
      const part = readReg(REG_PART_ID)
      const rev = readReg(REG_REV_ID)
      return { partId: part, revisionId: rev }
  }

  //% blockId=max30100_setLedsCurrent block="MAX30100 set LED currents IR %ir RED %red" blockHidden=true
  //% ir.defl=LedCurrent.mA50
  //% red.defl=LedCurrent.mA50
  export function setLedsCurrent(ir: LedCurrent, red: LedCurrent) {
      writeReg(REG_LED_CONFIG, ((red & 0x0F) << 4) | (ir & 0x0F))
  }

  //% blockId=max30100_setSamplingRate block="MAX30100 set sampling rate %rate" blockHidden=true
  //% rate.defl=SampleRate.SR100
  export function setSamplingRate(rate: SampleRate) {
      const prev = readReg(REG_SPO2_CONFIG)
      const next = (prev & 0xE3) | ((rate & 0x07) << 2)
      writeReg(REG_SPO2_CONFIG, next)
  }

  //% blockId=max30100_setPulseWidth block="MAX30100 set pulse width %pw" blockHidden=true
  //% pw.defl=PulseWidth.PW1600uS
  export function setPulseWidth(pw: PulseWidth) {
      const prev = readReg(REG_SPO2_CONFIG)
      const next = (prev & 0xFC) | (pw & 0x03)
      writeReg(REG_SPO2_CONFIG, next)
  }

  //% blockId=max30100_shutdown block="MAX30100 shutdown" blockHidden=true
  export function shutdown() {
      const prev = readReg(REG_MODE_CONFIG)
      writeReg(REG_MODE_CONFIG, prev | 0x80)
  }

  //% blockId=max30100_resume block="MAX30100 resume" blockHidden=true
  export function resume() {
      const prev = readReg(REG_MODE_CONFIG)
      writeReg(REG_MODE_CONFIG, prev & ~0x80)
  }

  //% blockId=max30100_stop block="stop MAX30100" weight=80
  export function stop() { _running = false }
}