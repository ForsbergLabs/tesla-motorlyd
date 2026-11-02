/**
 * drive.js — everything between "the car is moving" and "the engine model".
 *
 *   GpsSpeedSource   Geolocation -> outlier-gated Kalman filter (speed +
 *                    acceleration) -> smooth per-frame speed between 1 Hz fixes
 *   SpeedFollower    critically damped 2nd-order smoother (no steps, no lag spikes)
 *   VirtualCar       test mode: a speed slider driven by a simple driver model
 *   DemoRoute        scripted drive fed through the real GPS pipeline as noisy
 *                    1 Hz fixes, for testing without a car
 *   LoadEstimator    road-load physics: how hard the engine would have to work
 *                    to produce the measured acceleration -> throttle / demand
 *   GearSelector     road-car shift map: early upshifts in Normal, held gears in
 *                    Sport, kickdown, downshifts on braking, hysteresis + dwell
 *
 * Pure JS, no DOM except GpsSpeedSource.start(). Units: SI (m, s, m/s) inside.
 */

const RPM_PER_RADS = 60 / (2 * Math.PI);
const RHO_AIR = 1.225;
const G = 9.81;
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const lerp = (a, b, t) => a + (b - a) * t;

/** Great-circle distance in metres. */
export function haversine(lat1, lon1, lat2, lon2) {
  const R = 6371008.8;
  const toRad = Math.PI / 180;
  const dLat = (lat2 - lat1) * toRad;
  const dLon = (lon2 - lon1) * toRad;
  const a = Math.sin(dLat / 2) ** 2
    + Math.cos(lat1 * toRad) * Math.cos(lat2 * toRad) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.min(1, Math.sqrt(a)));
}

/** One-pole low-pass with a time constant, frame-rate independent. */
export function lowpass(prev, target, dt, tau) {
  if (!(tau > 0)) return target;
  return prev + (target - prev) * (1 - Math.exp(-dt / tau));
}

// ---------------------------------------------------------------------------
// Kalman filter on [speed, acceleration] with a constant-acceleration model.
// ---------------------------------------------------------------------------
export class SpeedKalman {
  /** @param {number} jerk  process noise: expected jerk std, m/s^3 */
  constructor(jerk = 1.2) {
    this.q = jerk * jerk;
    this.reset(0);
  }

  reset(v, varV = 4) {
    this.v = v; this.a = 0;
    this.P = [varV, 0, 0, 2];   // [Pvv, Pva, Pav, Paa]
    this.initialised = true;
  }

  predict(dt) {
    if (!(dt > 0)) return;
    dt = Math.min(dt, 5);
    const [p00, p01, p10, p11] = this.P;
    this.v += this.a * dt;
    // Acceleration slowly reverts to 0 when unobserved (cars do not hold a
    // constant acceleration for long); keeps extrapolation sane in tunnels.
    const decay = Math.exp(-dt / 6);
    this.a *= decay;
    const q = this.q, dt2 = dt * dt, dt3 = dt2 * dt;
    // F = [[1, dt], [0, decay]]
    const n00 = p00 + dt * (p10 + p01) + dt2 * p11 + q * dt3 / 3;
    const n01 = decay * (p01 + dt * p11) + q * dt2 / 2;
    const n10 = decay * (p10 + dt * p11) + q * dt2 / 2;
    const n11 = decay * decay * p11 + q * dt;
    this.P = [n00, n01, n10, n11];
    if (this.v < 0) this.v = 0;
  }

  /** Normalised innovation squared for measurement z with variance R. */
  nis(z, R) {
    const y = z - this.v;
    return (y * y) / (this.P[0] + R);
  }

  update(z, R) {
    const [p00, p01, p10, p11] = this.P;
    const S = p00 + R;
    const k0 = p00 / S, k1 = p10 / S;
    const y = z - this.v;
    this.v += k0 * y;
    this.a += k1 * y;
    this.P = [(1 - k0) * p00, (1 - k0) * p01, p10 - k1 * p00, p11 - k1 * p01];
    if (this.v < 0) this.v = 0;
    this.a = clamp(this.a, -12, 12);
  }
}

// ---------------------------------------------------------------------------
// GPS speed source.
// ---------------------------------------------------------------------------
export class GpsSpeedSource {
  constructor(opts = {}) {
    this.kf = new SpeedKalman(opts.jerk ?? 1.5);
    this.maxAccuracy = opts.maxAccuracy ?? 60;     // m, reject worse fixes
    this.onStatus = opts.onStatus || (() => {});
    this.watchId = null;
    this.reset();
  }

  reset() {
    this.kf.reset(0, 25);
    this.haveFix = false;
    this.last = null;          // last accepted position fix {lat, lon, acc, t}
    this.tKf = null;           // perf-clock seconds of the filter state
    this.lastFixWall = 0;
    this.rejects = 0;
    this.fixCount = 0;
    this.rawSpeed = null;      // last measured speed, m/s
    this.rawSource = '';
    this.accuracy = null;
    this.fixRate = 0;          // Hz, smoothed
  }

  get running() { return this.watchId != null; }

  start() {
    if (!('geolocation' in navigator)) {
      this.onStatus({ level: 'warn', text: 'Geolocation støttes ikke i denne nettleseren' });
      return false;
    }
    this.reset();
    this.watchId = navigator.geolocation.watchPosition(
      (pos) => this.ingest(pos),
      (err) => this.onStatus({ level: 'warn', text: 'GPS-feil: ' + (err && err.message || err) }),
      { enableHighAccuracy: true, maximumAge: 0, timeout: 15000 },
    );
    this.onStatus({ level: '', text: 'Venter på GPS…' });
    return true;
  }

  stop() {
    if (this.watchId != null && 'geolocation' in navigator) navigator.geolocation.clearWatch(this.watchId);
    this.watchId = null;
  }

  /**
   * Feed one position (a GeolocationPosition or a look-alike).
   * @param {object} pos  { coords: {latitude, longitude, accuracy, speed, speedAccuracy?}, timestamp }
   * @param {number} [nowMs] performance.now() at arrival
   */
  ingest(pos, nowMs = (typeof performance !== 'undefined' ? performance.now() : Date.now())) {
    const c = pos && pos.coords;
    if (!c) return;
    const t = nowMs / 1000;
    const ts = Number(pos.timestamp) || nowMs;
    const acc = Number.isFinite(c.accuracy) ? c.accuracy : 30;
    this.accuracy = acc;
    if (this.lastFixWall) {
      const dtw = (nowMs - this.lastFixWall) / 1000;
      if (dtw > 0.05 && dtw < 10) this.fixRate = this.fixRate ? lowpass(this.fixRate, 1 / dtw, 1, 3) : 1 / dtw;
    }
    this.lastFixWall = nowMs;

    if (acc > this.maxAccuracy) {
      this.rejects++;
      this.onStatus({ level: 'warn', text: `Svakt GPS-signal (±${Math.round(acc)} m) – venter` });
      return;
    }

    // --- measurement: Doppler speed first, else distance / time -------------
    let z = null, sigma = null, src = '';
    if (c.speed != null && Number.isFinite(c.speed) && c.speed >= 0) {
      z = c.speed;
      const sa = Number(c.speedAccuracy);
      sigma = Number.isFinite(sa) && sa > 0 ? Math.max(0.2, sa) : Math.max(0.3, 0.015 * z + 0.012 * Math.min(acc, 40));
      src = 'doppler';
    } else if (this.last) {
      const dt = (ts - this.last.ts) / 1000;
      if (dt >= 0.4 && dt <= 6) {
        const d = haversine(this.last.lat, this.last.lon, c.latitude, c.longitude);
        z = d / dt;
        // Consecutive fixes share most of their error (it drifts slowly), so
        // the error of the DIFFERENCE is a fraction of the reported accuracy.
        sigma = Math.max(0.4, (0.15 * (acc + this.last.acc)) / dt);
        // A car standing still wanders a few metres; that is not speed.
        if (d < 0.6 * (acc + this.last.acc) * 0.5 && z < 2) z = 0;
        // Distance / time is the AVERAGE speed over the interval, i.e. the
        // speed half an interval ago; bring it forward with the current
        // acceleration estimate so the fallback does not lag by dt/2.
        if (z > 0 && this.haveFix) z = Math.max(0, z + this.kf.a * dt / 2);
        src = 'posisjon';
      }
    }
    this.last = { lat: c.latitude, lon: c.longitude, acc, ts };
    if (z == null) return;
    if (!(z < 95)) { this.rejects++; return; }       // > 340 km/h: nonsense

    const R = sigma * sigma;
    if (!this.haveFix) {
      this.kf.reset(z, R);
      this.tKf = t;
      this.haveFix = true;
    } else {
      this.kf.predict(t - this.tKf);
      this.tKf = t;
      // Outlier gate: 4.5 sigma. Two in a row and we believe the GPS (the car
      // really did change speed, or we were wrong) and re-seed.
      if (this.kf.nis(z, R) > 20 && this.rejects < 2) {
        this.rejects++;
        return;
      }
      if (this.rejects >= 2 && this.kf.nis(z, R) > 20) this.kf.reset(z, R);
      else this.kf.update(z, R);
    }
    this.rejects = 0;
    this.fixCount++;
    this.rawSpeed = z;
    this.rawSource = src;
    this.onStatus({
      level: 'ok',
      text: `GPS ${Math.round(z * 3.6)} km/h · ±${Math.round(acc)} m · ${src}`
        + (this.fixRate ? ` · ${this.fixRate.toFixed(1)} Hz` : ''),
    });
  }

  /**
   * Filter estimate at `nowMs`, extrapolated from the last fix with the
   * estimated acceleration (bounded), for smooth per-frame use.
   */
  sample(nowMs) {
    if (!this.haveFix) return { v: 0, a: 0, stale: true, valid: false };
    const age = Math.max(0, nowMs / 1000 - this.tKf);
    const ext = Math.min(age, 1.6);                  // never extrapolate far
    const fade = age > 1.6 ? Math.exp(-(age - 1.6) / 1.5) : 1;
    const v = Math.max(0, this.kf.v + this.kf.a * ext);
    return { v, a: this.kf.a * fade, stale: age > 4, valid: true };
  }
}

// ---------------------------------------------------------------------------
// Critically damped follower: removes the small corrections each new fix makes
// to the extrapolated speed without adding a step or a long lag.
// ---------------------------------------------------------------------------
export class SpeedFollower {
  constructor(hz = 1.1) { this.w = 2 * Math.PI * hz; this.v = 0; this.dv = 0; }
  reset(v = 0) { this.v = v; this.dv = 0; }
  step(target, dt) {
    if (!(dt > 0)) return this.v;
    const w = this.w;
    // Sub-step for stability at low frame rates.
    const n = Math.max(1, Math.ceil(dt / 0.01));
    const h = dt / n;
    for (let i = 0; i < n; i++) {
      const acc = w * w * (target - this.v) - 2 * w * this.dv;
      this.dv += acc * h;
      this.v += this.dv * h;
    }
    if (this.v < 0) { this.v = 0; if (this.dv < 0) this.dv = 0; }
    return this.v;
  }
}

// ---------------------------------------------------------------------------
// Test mode: virtual car following a speed slider.
// ---------------------------------------------------------------------------
export class VirtualCar {
  constructor() { this.v = 0; this.a = 0; this.target = 0; }
  /** @param {string} mode 'normal' | 'sport' */
  step(dt, mode) {
    const sport = mode === 'sport';
    const err = this.target - this.v;
    // Available acceleration falls with speed (power limited).
    const aMax = (sport ? 5.2 : 3.0) * clamp(1 - this.v / (sport ? 95 : 80), 0.15, 1);
    let aCmd;
    if (err > 0.3) aCmd = Math.min(aMax, 0.9 * err + 0.4);
    else if (err < -0.3) {
      // Small drop: lift off and coast (engine braking). Big drop: brake.
      const brake = clamp((-err - 2) * 0.55, 0, sport ? 7.5 : 6);
      aCmd = -(0.7 + 0.01 * this.v) - brake;
    } else aCmd = err * 0.8;
    // Jerk limit so the virtual driver is smooth like a human foot.
    const jMax = sport ? 9 : 5;
    this.a += clamp(aCmd - this.a, -jMax * dt, jMax * dt);
    this.v = Math.max(0, this.v + this.a * dt);
    if (this.v === 0 && this.a < 0) this.a = 0;
    return this.v;
  }
}

// ---------------------------------------------------------------------------
// Demo route: a scripted drive fed through the GPS pipeline as 1 Hz fixes with
// realistic noise, to test the whole chain (filter, smoothing, gears) at a desk.
// ---------------------------------------------------------------------------
export class DemoRoute {
  constructor() {
    // [duration s, target km/h, label]
    this.legs = [
      [4, 0, 'Står stille'],
      [14, 50, 'Bykjøring 50'],
      [6, 50, 'Jevn 50'],
      [7, 0, 'Bremser til stopp'],
      [3, 0, 'Står stille'],
      [22, 110, 'Påkjøring motorvei'],
      [14, 110, 'Cruise 110'],
      [8, 150, 'Forbikjøring (full gass)'],
      [10, 110, 'Tilbake til 110'],
      [14, 30, 'Avkjøring, motorbrems'],
      [8, 0, 'Stopp'],
    ];
    this.car = new VirtualCar();
    this.t = 0; this.fixT = 0;
    this.lat = 59.91; this.lon = 10.75;
    this.heading = 0.6;
    this.seed = 12345;
    this.ts = 0;
  }
  _rand() { // deterministic gaussian
    this.seed = (this.seed * 1664525 + 1013904223) >>> 0;
    const u = (this.seed + 1) / 4294967297;
    this.seed = (this.seed * 1664525 + 1013904223) >>> 0;
    const w = (this.seed + 1) / 4294967297;
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * w);
  }
  get duration() { return this.legs.reduce((s, l) => s + l[0], 0); }
  leg() {
    let acc = 0;
    for (const l of this.legs) { acc += l[0]; if (this.t < acc) return l; }
    return null;
  }
  /** Advance; returns an array of fake GeolocationPositions that became due. */
  step(dt, mode = 'normal') {
    const leg = this.leg();
    if (!leg) return { fixes: [], done: true, label: 'Ferdig' };
    this.car.target = leg[1] / 3.6;
    this.car.step(dt, mode);
    this.t += dt;
    const d = this.car.v * dt;
    this.heading += 0.02 * dt;
    this.lat += (d * Math.cos(this.heading)) / 111320;
    this.lon += (d * Math.sin(this.heading)) / (111320 * Math.cos(this.lat * Math.PI / 180));
    this.ts += dt * 1000;
    const fixes = [];
    this.fixT += dt;
    if (this.fixT >= 1) {
      this.fixT -= 1;
      const noisyV = Math.max(0, this.car.v + 0.25 * this._rand());
      // Every ~25th fix is a glitch, to exercise the outlier gate.
      const glitch = (Math.round(this.t) % 25) === 12;
      fixes.push({
        timestamp: this.ts,
        coords: {
          latitude: this.lat + 8e-6 * this._rand(),
          longitude: this.lon + 8e-6 * this._rand(),
          accuracy: 6 + 2 * Math.abs(this._rand()),
          speed: glitch ? noisyV + 9 : noisyV,
          speedAccuracy: 0.5,
        },
      });
    }
    return { fixes, done: false, label: leg[2], trueV: this.car.v };
  }
}

// ---------------------------------------------------------------------------
// Load estimation from road physics.
// ---------------------------------------------------------------------------
export class LoadEstimator {
  /**
   * @param {object} vehicle  geared vehicle: mass, wheelRadius, finalDrive,
   *                          gearRatios, dragArea, rollingResistance
   * @param {object} engine   peakTorque, peakTorqueRpm, redlineRpm, idleRpm, turbo?
   */
  constructor(vehicle, engine) { this.set(vehicle, engine); this.thr = 0; this.demand = 0; this.aS = 0; this.pNeed = 0; }

  set(vehicle, engine) {
    this.veh = vehicle; this.eng = engine;
    // Peak power, for the gear-independent "how hard is the driver pushing".
    let best = 0;
    // Monotone "best power available at or below this rpm" table, 100 rpm steps.
    this.pTable = [];
    for (let rpm = 0; rpm <= engine.redlineRpm + 100; rpm += 100) {
      const r = Math.max(rpm, engine.idleRpm);
      best = Math.max(best, this.wot(r) * r / RPM_PER_RADS);
      this.pTable.push(best);
    }
    this.pPeak = Math.max(30000, best);
  }

  wot(rpm) {
    const e = this.eng;
    const k = (rpm - e.peakTorqueRpm) / (e.redlineRpm * 0.75);
    const shape = clamp(1 - 0.85 * k * k, 0.3, 1);
    const bm = e.turbo ? 1 + 0.55 * 0.8 * (e.turbo.maxBoost || 1) : 1;
    return e.peakTorque * shape * bm;
  }

  /** Lowest rpm at which full-throttle power covers `p` watts (Infinity if none). */
  rpmForPower(p) {
    if (!(p > 0)) return 0;
    const t = this.pTable;
    for (let i = 0; i < t.length; i++) if (t[i] >= p) return i * 100;
    return Infinity;
  }

  /** Force the road needs at speed v (m/s) to hold acceleration a (m/s^2). */
  forceNeeded(v, a) {
    const veh = this.veh;
    const m = veh.mass;
    return 1.06 * m * a + 0.5 * RHO_AIR * veh.dragArea * v * v
      + (v > 0.3 ? veh.rollingResistance * m * G : 0);
  }

  /**
   * @param {number} v  m/s (smoothed)
   * @param {number} a  m/s^2 (smoothed)
   * @param {number} gear current gear (1..n)
   * @param {number} dt seconds
   * @returns {{throttle:number, demand:number}}
   */
  step(v, a, gear, dt) {
    const veh = this.veh, eng = this.eng;
    // Strong smoothing on acceleration: GPS acceleration is a derivative of a
    // 1 Hz signal and is the noisiest number in the chain.
    // Deadband: +-0.08 m/s^2 is GPS noise at cruise, not the driver.
    const aDb = Math.abs(a) < 0.08 ? 0 : a - Math.sign(a) * 0.08;
    this.aS = lowpass(this.aS, aDb, dt, 0.6);
    const F = this.forceNeeded(v, this.aS);
    const P = F * v;
    // Slow, for the shift map's power-reserve rule (anti-hunting).
    this.pNeed = lowpass(this.pNeed, Math.max(0, P), dt, this.pNeed < P ? 0.6 : 1.4);
    // Gear-INDEPENDENT demand drives the shift map (prevents hunting through
    // a gear-dependent load estimate).

    // The SOUND throttle is an estimate of the driver's PEDAL, not of how hard
    // the virtual engine has to work: a 600 hp engine moving a car at 2 m/s^2
    // is physically at light load, but the driver of the real car is clearly
    // pushing, and that is what the sound has to follow. Reference: a quick
    // road car's full-pedal capability (4.5 m/s^2 off the line, ~160 kW at the
    // wheels higher up). Gear-independent, so a shift never jolts the load.
    const m = veh.mass;
    const fRef = Math.min(4.5 * m, 160000 / Math.max(v, 1)) + this.forceNeeded(v, 0);
    let thrTarget = F > 0 ? clamp(F / fRef, 0, 1) : 0;
    void gear;
    // Rolling at steady speed always needs a little fuel; do not let cruise
    // noise flicker in and out of fuel cut.
    if (v > 1.5 && this.aS > -0.45) thrTarget = Math.max(thrTarget, 0.07);
    // Standing start: a driver gives it some to pull away.
    if (v < 3 && this.aS > 0.4) thrTarget = Math.max(thrTarget, clamp(0.18 + 0.12 * this.aS, 0, 0.6));
    // Shift-map demand: the same pedal estimate, smoothed a little slower on
    // the way down so a brief lift does not trigger an upshift.
    this.demand = lowpass(this.demand, thrTarget, dt, this.demand < thrTarget ? 0.45 : 0.9);
    // Asymmetric smoothing + slew limit: no jumps, no jerks.
    const tau = thrTarget > this.thr ? 0.6 : 0.45;
    let next = lowpass(this.thr, thrTarget, dt, tau);
    const slew = 1.6 * dt;
    next = clamp(next, this.thr - slew, this.thr + slew);
    this.thr = clamp(next, 0, 1);
    return { throttle: this.thr, demand: this.demand, accel: this.aS, pNeed: this.pNeed };
  }
}

// ---------------------------------------------------------------------------
// Gear selection.
// ---------------------------------------------------------------------------
export class GearSelector {
  constructor() { this.since = 99; this.upHold = 0; this.downHold = 0; this.lastReason = ''; }

  reset() { this.since = 99; this.upHold = 0; this.downHold = 0; }

  rpmAt(g, v, veh) {
    return v / veh.wheelRadius * veh.finalDrive * veh.gearRatios[g - 1] * RPM_PER_RADS;
  }

  /**
   * @returns {number} desired gear
   * opts: { gear, v, accel, demand, mode, veh, eng, shifting, dt }
   */
  choose({ gear, v, accel, demand, mode, veh, eng, shifting, dt, rpmForPower, pNeed }) {
    this.since += dt;
    const n = veh.gearRatios.length;
    if (shifting) return gear;
    if (gear < 1) return 1;
    const sport = mode === 'sport';
    const red = eng.redlineRpm, idle = eng.idleRpm;
    const rpm = (g) => this.rpmAt(g, v, veh);

    // Stopping / crawling: first gear.
    if (v < 2.2 && accel < 0.5) {
      if (gear !== 1 && this.since > 0.4) { this.since = 0; this.lastReason = 'stopp'; return 1; }
      return gear;
    }

    const braking = accel < -1.2;
    const d = clamp(demand, 0, 1);
    // Lowest rpm the box is willing to sit at, as a fraction of redline.
    let floorF = sport ? 0.25 + 0.55 * Math.pow(d, 1.3) : 0.165 + 0.46 * Math.pow(d, 2.2);
    // Braking: hold the revs up for engine braking and so the box is in the
    // right gear to pull away again. Sport rev-matches down aggressively.
    if (braking) {
      const b = clamp((-accel - 1.2) / 3, 0, 1);
      floorF = Math.max(floorF, sport ? 0.40 + 0.15 * b : 0.20 + 0.08 * b);
    }
    let floor = Math.max(idle * 1.45, floorF * red);
    // Power reserve: the gear must deliver what the road is asking for with
    // margin left (Normal uses <= 70 % of what that rpm can give, Sport 50 %).
    // This is what makes the box kick down for a motorway overtake even though
    // the pedal-ish demand is moderate, and stay in top gear on a flat cruise.
    if (rpmForPower && pNeed > 0) {
      const need = rpmForPower(pNeed / (sport ? 0.5 : 0.7));
      // Capped well below the upshift ceiling, or an engine too small for
      // the demand would bounce between "over the ceiling, change up" and
      // "not enough power, kick down" (gear hunting).
      const cap = red * (sport ? 0.70 : 0.60);
      floor = Math.max(floor, Math.min(Number.isFinite(need) ? need : cap, cap));
    }
    // Latest upshift point.
    const ceil = red * (sport ? 0.93 : 0.78 + 0.14 * d);

    // Ideal: the tallest gear that keeps rpm at or above the floor.
    let ideal = 1;
    for (let g = 1; g <= n; g++) if (rpm(g) >= floor) ideal = g;
    while (ideal < n && rpm(ideal) > ceil) ideal++;

    let target = gear;
    if (ideal > gear) {
      const over = rpm(gear) > ceil;
      // Upshift hysteresis: the next gear must clear the floor with margin and
      // the wish must persist; over the ceiling it goes at once.
      const nextOk = rpm(gear + 1) >= floor * 1.1;
      this.upHold = nextOk ? this.upHold + dt : 0;
      // After a downshift, do not change straight back up (hunting).
      const dwell = this.lastUp === false ? 3.0 : (sport ? 0.9 : 1.1);
      if (over || (nextOk && this.upHold > (sport ? 0.35 : 0.25) && this.since > dwell)) {
        // One at a time when pulling; skip-shift when cruising gently (a
        // modern automatic going 3 -> 5 on a light throttle).
        target = (d < 0.25 && !sport && !braking) ? Math.min(ideal, gear + 2) : gear + 1;
        this.lastReason = over ? 'opp (makstur)' : 'opp';
      }
      this.downHold = 0;
    } else if (ideal < gear) {
      this.upHold = 0;
      // Downshift hysteresis: only once rpm has fallen clearly under the floor.
      const under = rpm(gear) < floor * (braking ? 0.93 : 0.86);
      this.downHold = under ? this.downHold + dt : 0;
      const kick = d > 0.45 && rpm(gear) < floor * 0.8;
      // After an upshift the box will not change its mind for a while.
      const dwell = this.lastUp && !braking ? 3.0 : kick ? 0.35 : braking ? 0.55 : 0.8;
      if (under && (kick || this.downHold > 0.2) && this.since > dwell) {
        // One gear per step, sequentially (~0.35 s apart on a kickdown): a
        // two- or three-gear drop in one go slammed the revs up in ~70 ms.
        target = gear - 1;
        this.lastReason = kick ? 'kickdown' : braking ? 'ned (brems)' : 'ned';
      }
    } else {
      this.upHold = 0; this.downHold = 0;
    }
    // Never select a gear that would over-rev.
    while (target > 1 && target < gear && rpm(target) > red * 0.95) target++;
    target = clamp(target, 1, n);
    if (target !== gear) { this.since = 0; this.lastUp = target > gear; }
    return target;
  }
}
