/**
 * controller.js — drives an EngineSim (or a bare Drivetrain host) from the
 * real car's speed (GPS), from the test-mode speed slider, or from the demo
 * route. One pipeline for all three, so what you test at the desk is what
 * runs in the car.
 *
 * host interface (EngineSim satisfies it):
 *   host.physics   Drivetrain  (speedTarget, throttle, brake, gear, setGear, setAutoShift, shift)
 *   host.geared    vehicle with the gearbox actually fitted
 *   host._machine() engine profile incl. turbo
 */
import {
  GpsSpeedSource, SpeedFollower, VirtualCar, DemoRoute,
  LoadEstimator, GearSelector,
} from './drive.js';

export class DriveController {
  constructor(host, opts = {}) {
    this.host = host;
    this.source = 'test';        // 'test' | 'gps' | 'demo'
    this.mode = 'normal';        // 'normal' | 'sport'
    this.gps = new GpsSpeedSource({ onStatus: (s) => { this.gpsStatus = s; if (opts.onGpsStatus) opts.onGpsStatus(s); } });
    this.gpsStatus = { level: '', text: '' };
    this.follower = new SpeedFollower(1.1);
    this.car = new VirtualCar();
    this.demo = null;
    this.selector = new GearSelector();
    this.load = new LoadEstimator(host.geared, host._machine());
    this.v = 0; this.displayV = 0; this.accel = 0; this.demand = 0; this.throttle = 0;
    this.demoLabel = '';
    this._attach();
  }

  /** Call after the host changed engine / vehicle / turbo. */
  refresh() { this.load.set(this.host.geared, this.host._machine()); this._attach(); }

  _attach() {
    const ph = this.host.physics;
    ph.setAutoShift(false);           // we own gear selection; never two shifters
    ph.lashScale = 0.3;               // soft driveline clunks, no knocking
    ph.brake = 0;
  }

  setSource(src) {
    if (src === this.source) return;
    if (this.source === 'gps') this.gps.stop();
    this.source = src;
    if (src === 'gps') {
      this.gps.start();
      this.follower.reset(this.v);
    } else if (src === 'demo') {
      this.demo = new DemoRoute();
      this.gps.reset();
      this.follower.reset(0);
    } else {
      this.car.v = this.v; this.car.a = 0; this.car.target = this.v;
    }
  }

  setMode(m) { this.mode = m === 'sport' ? 'sport' : 'normal'; }
  setTargetSpeedKmh(kmh) { this.car.target = Math.max(0, kmh) / 3.6; }

  /**
   * Advance one animation frame. Must be called BEFORE host.update(dt).
   * @param {number} dt seconds
   * @param {number} nowMs performance.now()
   */
  frame(dt, nowMs) {
    if (!(dt > 0)) return;
    const ph = this.host.physics;
    let v, a;
    if (this.source === 'test') {
      v = this.car.step(dt, this.mode);
      a = this.car.a;
      this.displayV = v;
    } else {
      if (this.source === 'demo' && this.demo) {
        const r = this.demo.step(dt, this.mode);
        for (const fix of r.fixes) this.gps.ingest(fix, nowMs);
        this.demoLabel = r.done ? 'Demo ferdig' : r.label;
        if (r.done) this.demo = new DemoRoute();
      }
      const s = this.gps.sample(nowMs);
      // Before the first fix: hold the current speed instead of diving to 0.
      v = this.follower.step(s.valid ? s.v : this.v, dt);
      a = s.a;
      // The display shows the GPS (filtered) speed of the real car, not the
      // engine model's.
      this.displayV = s.valid ? s.v : 0;
      if (s.stale && this.source === 'gps') {
        // Lost GPS (tunnel, garage): coast the estimate gently, never jump.
        a = Math.min(a, 0);
      }
    }
    this.v = v;
    const est = this.load.step(v, a, ph.gear || 1, dt);
    this.accel = est.accel; this.demand = est.demand; this.throttle = est.throttle;

    const gear = ph.gear || 1;
    const target = this.selector.choose({
      gear, v, accel: est.accel, demand: est.demand, mode: this.mode,
      veh: this.host.geared, eng: this.host._machine(),
      shifting: ph.shift ? ph.shift.busy : false, dt,
      rpmForPower: (p) => this.load.rpmForPower(p), pNeed: est.pNeed,
    });
    if (target !== gear) ph.setGear(target);

    ph.speedTarget = v;
    ph.throttle = est.throttle;
    ph.brake = 0;
  }

  stop() { this.gps.stop(); }
}
