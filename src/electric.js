/**
 * electric.js — synthesised electric-drive sound ("Taycan-style" EV).
 *
 * Inspired by the character of Porsche's Electric Sport Sound: a deep, tonal,
 * futuristic "turbine / spaceship" hum. Nothing here is a recording — every
 * component is an oscillator, a filter or filtered noise, driven per frame
 * from the same physics params the combustion voices get:
 *
 *   p.speed     road speed, m/s        → pitch (motor rpm ∝ speed, one ratio)
 *   p.throttle  pedal estimate, 0..1   → level, brightness, harmonic richness
 *   d(speed)/dt (derived here)         → regen: soft, darker descending tone
 *
 * Voices
 *   drone    two detuned saws at f0 + one at 2·f0 → resonant lowpass whose
 *            cutoff opens with load ("the sweep")
 *   sub      sine at f0 — the body, and the idle hum at standstill
 *   chord    sines at 1.5·f0 (fifth) and 2·f0 (octave) — the sci-fi chord,
 *            grows with load
 *   whine    sine pair at motorHz · order, a few cents apart, + 2nd harmonic —
 *            the e-motor / reduction-gear whir
 *   inverter faint high tone that rises gently with speed
 *   swoosh   band-passed noise that tracks the drone — the "turbine air"
 *   regen    triangle at 3·f0, only when decelerating off-throttle
 *   wide     two sine pairs at 2·f0 panned hard L/R, ±12 cents — chorus width
 *
 * `output` is mono (it goes through the app's tone stage like every other
 * voice), `wide` is stereo and is sent straight to the output dynamics.
 * No gears, no combustion pulses, no turbo, no pops.
 */

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

import { EV_DEFAULTS } from './profiles.js';

export { EV_DEFAULTS };

export class ElectricDrive {
  constructor(ctx, cfg = {}) {
    this.ctx = ctx;
    this.cfg = { ...EV_DEFAULTS, ...(cfg || {}) };
    this.disposed = false;
    this.started = false;
    this.v = 0; this.vPrev = null; this.decel = 0; this.load = 0; this.regen = 0;
    this.motorRpm = 0; this.f0 = this.cfg.droneHz;
    const t0 = ctx.currentTime;
    const nodes = this._nodes = [];
    const oscs = this._oscs = [];
    const osc = (type, f, detune = 0) => {
      const o = ctx.createOscillator();
      o.type = type; o.frequency.value = f; o.detune.value = detune;
      oscs.push(o); nodes.push(o); return o;
    };
    const gain = (g = 0) => { const n = ctx.createGain(); n.gain.value = g; nodes.push(n); return n; };
    const biquad = (type, f, q) => {
      const n = ctx.createBiquadFilter(); n.type = type; n.frequency.value = f; n.Q.value = q;
      nodes.push(n); return n;
    };

    this.out = gain(1);
    this.trim = gain(0.27 * this.cfg.level);
    this.trim.connect(this.out);
    this.levelG = gain(0);          // overall level, per frame
    this.levelG.connect(this.trim);

    // --- drone: detuned saws through a resonant lowpass --------------------
    const f = this.f0;
    this.sawA = osc('sawtooth', f, -4);
    this.sawB = osc('sawtooth', f, +4);
    this.sawC = osc('sawtooth', 2 * f, +3);
    this.droneLP = biquad('lowpass', 4 * f, 4.5);
    this.droneLP2 = biquad('lowpass', 2400, 0.7);   // keep the saws off the presence band
    const sawMix = gain(1);
    const sawCg = gain(0.45);
    this.sawA.connect(sawMix); this.sawB.connect(sawMix);
    this.sawC.connect(sawCg); sawCg.connect(sawMix);
    // The sub owns the fundamental. Without this highpass the detuned saws'
    // own fundamentals beat against it and the level swells by ~6 dB every
    // few seconds at a steady cruise; their upper harmonics keep the chorus.
    this.droneHP = biquad('highpass', 1.4 * f, 0.7);
    sawMix.connect(this.droneHP); this.droneHP.connect(this.droneLP); this.droneLP.connect(this.droneLP2);
    this.droneG = gain(0.22);
    this.droneLP2.connect(this.droneG); this.droneG.connect(this.levelG);

    // Slow chorus wander on the saws' detune: the two never lock.
    this.lfo = osc('sine', 0.19);
    const lfoA = gain(3), lfoB = gain(-3);
    this.lfo.connect(lfoA); this.lfo.connect(lfoB);
    lfoA.connect(this.sawA.detune); lfoB.connect(this.sawB.detune);

    // --- sub / idle hum ------------------------------------------------------
    this.sub = osc('sine', f);
    this.subG = gain(0.55);
    this.sub.connect(this.subG); this.subG.connect(this.levelG);

    // --- chord: fifth + octave ----------------------------------------------
    this.fifth = osc('sine', 1.5 * f, -4);
    this.oct = osc('triangle', 2 * f, +4);
    this.fifthG = gain(0); this.octG = gain(0);
    this.fifth.connect(this.fifthG); this.oct.connect(this.octG);
    this.fifthG.connect(this.levelG); this.octG.connect(this.levelG);

    // --- motor whine ----------------------------------------------------------
    this.whA = osc('sine', 100);
    this.whB = osc('sine', 100, 9);
    this.wh2 = osc('triangle', 200);
    this.whineG = gain(0);
    const wh2g = gain(0.3);
    this.whA.connect(this.whineG); this.whB.connect(this.whineG);
    this.wh2.connect(wh2g); wh2g.connect(this.whineG);
    this.whineG.connect(this.levelG);

    // --- inverter tone --------------------------------------------------------
    this.inv = osc('sine', 1300);
    this.invG = gain(0);
    this.inv.connect(this.invG); this.invG.connect(this.levelG);

    // --- swoosh: band-passed noise -------------------------------------------
    const len = Math.max(1, Math.floor(ctx.sampleRate * 2));
    const buf = ctx.createBuffer(1, len, ctx.sampleRate);
    const d = buf.getChannelData(0);
    let r = 0x1234567;
    for (let i = 0; i < len; i++) { r ^= r << 13; r ^= r >>> 17; r ^= r << 5; d[i] = ((r >>> 0) / 4294967296) * 2 - 1; }
    this.noise = ctx.createBufferSource();
    this.noise.buffer = buf; this.noise.loop = true;
    oscs.push(this.noise); nodes.push(this.noise);
    this.noiseBP = biquad('bandpass', 400, 1.4);
    this.noiseG = gain(0);
    this.noise.connect(this.noiseBP); this.noiseBP.connect(this.noiseG); this.noiseG.connect(this.levelG);

    // --- regen ------------------------------------------------------------------
    this.reg = osc('triangle', 3 * f);
    this.regLP = biquad('lowpass', 900, 0.8);
    this.regG = gain(0);
    this.reg.connect(this.regLP); this.regLP.connect(this.regG); this.regG.connect(this.levelG);

    // --- stereo width (chorus) ------------------------------------------------
    this.wideOut = gain(1);
    this.wideG = gain(0);
    this.wideG.connect(this.wideOut);
    const merger = ctx.createChannelMerger(2); nodes.push(merger);
    merger.connect(this.wideG);
    this.wL = osc('sine', 2 * f, -12);
    this.wR = osc('sine', 2 * f, +12);
    this.wL3 = osc('sine', 3 * f, +8);
    this.wR3 = osc('sine', 3 * f, -8);
    const wl = gain(1), wr = gain(1), wl3 = gain(0.5), wr3 = gain(0.5);
    this.wL.connect(wl); this.wR.connect(wr); this.wL3.connect(wl3); this.wR3.connect(wr3);
    wl.connect(merger, 0, 0); wl3.connect(merger, 0, 0);
    wr.connect(merger, 0, 1); wr3.connect(merger, 0, 1);

    void t0;
  }

  get output() { return this.out; }
  get wide() { return this.wideOut; }

  start(t) {
    if (this.started) return;
    this.started = true;
    for (const o of this._oscs) { try { o.start(t); } catch (e) { /* already started */ } }
  }

  stop(t) {
    for (const o of this._oscs) { try { o.stop(t); } catch (e) { /* not started */ } }
  }

  /** Per-frame: map speed / pedal / deceleration onto the voices. */
  update(p) {
    if (this.disposed) return;
    const c = this.cfg;
    const now = Number.isFinite(p.now) ? p.now : this.ctx.currentTime;
    const dt = p.dt > 0 ? p.dt : 0;
    const v = Math.max(0, Number(p.speed) || 0);
    // Deceleration from the road speed itself (smoothed), for regen.
    if (this.vPrev != null && dt > 0) {
      const a = (v - this.vPrev) / dt;
      const k = 1 - Math.exp(-dt / 0.35);
      this.decel += (Math.max(0, -a) - this.decel) * k;
    }
    this.vPrev = v;
    const kmh = v * 3.6;
    const thr = clamp(Number(p.throttle) || 0, 0, 1);
    const kl = dt > 0 ? 1 - Math.exp(-dt / 0.12) : 1;
    this.load += (thr - this.load) * kl;
    const load = this.load;
    const moving = clamp(kmh / 8, 0, 1);
    const regenT = thr < 0.06 && kmh > 6 ? clamp((this.decel - 0.25) / 2.5, 0, 1) : 0;
    this.regen += (regenT - this.regen) * kl;
    const regen = this.regen;
    const spd = clamp(kmh / 200, 0, 1);

    // --- pitch -----------------------------------------------------------------
    const f0 = c.droneHz + c.hzPerKmh * kmh;
    this.f0 = f0;
    this.motorRpm = c.maxMotorRpm * clamp(kmh / c.vmaxKmh, 0, 1.15);
    const motorHz = this.motorRpm / 60;
    const fw = clamp(motorHz * c.whineOrder, 20, 3200);
    const tc = 0.04;
    const F = (param, val, tcc = tc) => { if (Number.isFinite(val)) param.setTargetAtTime(val, now, tcc); };
    F(this.droneHP.frequency, 1.4 * f0);
    F(this.sawA.frequency, f0); F(this.sawB.frequency, f0); F(this.sawC.frequency, 2 * f0);
    F(this.sub.frequency, f0);
    F(this.fifth.frequency, 1.5 * f0); F(this.oct.frequency, 2 * f0);
    F(this.whA.frequency, fw); F(this.whB.frequency, fw); F(this.wh2.frequency, 2 * fw);
    F(this.inv.frequency, 1300 + 4.5 * kmh);
    F(this.reg.frequency, 3 * f0);
    F(this.wL.frequency, 2 * f0); F(this.wR.frequency, 2 * f0);
    F(this.wL3.frequency, 3 * f0); F(this.wR3.frequency, 3 * f0);
    F(this.noiseBP.frequency, clamp(3 * f0 + 250 + 900 * load, 100, 4000));

    // --- brightness: the lowpass sweep opens with load ------------------------
    const cut = clamp(f0 * (2.2 + 9 * load + 2 * spd) * (1 - 0.35 * regen), 80, 5000);
    F(this.droneLP.frequency, cut, 0.08);
    F(this.droneLP.Q, 3 + 4 * load, 0.1);

    // --- levels ---------------------------------------------------------------
    // Standing still: a quiet hum. Accelerating: louder and richer. Coasting:
    // softer. Regen: softer still, with its own dark descending tone.
    const coast = thr < 0.08 ? 1 : 0;
    const lev = (0.5 + 0.5 * moving)
              * (0.46 + 0.54 * Math.pow(load, 0.65))
              * (0.85 + 0.25 * spd)
              * (1 - 0.15 * coast * moving);
    F(this.levelG.gain, lev, 0.06);
    F(this.droneG.gain, 0.16 + 0.16 * load, 0.06);
    F(this.subG.gain, 0.55 - 0.15 * spd, 0.06);
    F(this.fifthG.gain, (0.05 + 0.3 * load) * (0.3 + 0.7 * moving), 0.06);
    F(this.octG.gain, (0.08 + 0.22 * load + 0.1 * spd) * (0.3 + 0.7 * moving), 0.06);
    F(this.whineG.gain, moving * (0.025 + 0.11 * load + 0.04 * spd + 0.03 * regen), 0.06);
    F(this.invG.gain, moving * (0.004 + 0.012 * load), 0.08);
    F(this.noiseG.gain, (0.04 + 0.3 * load) * moving * (0.4 + 0.6 * spd), 0.08);
    F(this.regG.gain, 0.22 * regen, 0.08);
    F(this.wideG.gain, 0.27 * this.cfg.level * lev * (0.05 + 0.12 * load + 0.04 * spd), 0.06);
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    const t = this.ctx.currentTime;
    for (const o of this._oscs) { try { o.stop(t); } catch (e) { /* not started */ } }
    for (const n of this._nodes) { try { n.disconnect(); } catch (e) { /* gone */ } }
  }
}
