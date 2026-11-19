import test from 'node:test';
import assert from 'node:assert/strict';
import { BicycleVehicle, SteeringController, pacejkaLateral, limitCombinedSlip } from '../src/dynamics.js';

const approx = (actual, expected, tol = 1e-6) => assert.ok(Math.abs(actual - expected) <= tol, `${actual} not within ${tol} of ${expected}`);
const control = (steer = 0, throttle = 0, brake = 0, reverse = false) => ({ steer, throttle, brake, reverse });

test('Pacejka returns zero for zero load and slip', () => {
  approx(pacejkaLateral(0, 4000, 0.9), 0);
  approx(pacejkaLateral(0.2, 0, 0.9), 0);
});
test('Pacejka force is odd in slip and bounded by mu Fz', () => {
  for (let a = -1.4; a <= 1.4; a += 0.013) {
    const force = pacejkaLateral(a, 4000, 0.75);
    approx(force, -pacejkaLateral(-a, 4000, 0.75), 1e-8);
    assert.ok(Math.abs(force) <= 3000 + 1e-7);
  }
});
test('Combined-slip cap never exceeds friction circle', () => {
  for (let fx = -6000; fx <= 6000; fx += 400) {
    for (let fy = -6000; fy <= 6000; fy += 400) {
      const v = limitCombinedSlip(fx, fy, 3500, 0.8);
      assert.ok(Math.hypot(v.fx, v.fy) <= 2800 + 1e-8);
    }
  }
});
test('steering returns to center without oscillation', () => {
  const c = new SteeringController();
  for (let i = 0; i < 25; i++) c.step(1, 4, 0, 0, 0.8, 0.02, false);
  const initial = c.angleRad;
  for (let i = 0; i < 100; i++) c.step(0, 0, 0, 0, 0.8, 0.02, false);
  assert.ok(initial > 0);
  approx(c.angleRad, 0);
});
test('touch steering is less sensitive at high speed', () => {
  const fast = new SteeringController(), slow = new SteeringController();
  for (let i = 0; i < 200; i++) {
    slow.step(0.9, 1, 0, 0, 0.9, 0.02, false);
    fast.step(0.9, 20, 0, 0, 0.9, 0.02, false);
  }
  assert.ok(Math.abs(fast.angleRad) < Math.abs(slow.angleRad));
});
test('bounded countersteer reacts against sideslip', () => {
  const a = new SteeringController(), b = new SteeringController();
  for (let i = 0; i < 80; i++) {
    a.step(0, 12, 0.17, 0.2, 0.9, 0.02, true);
    b.step(0, 12, 0.17, 0.2, 0.9, 0.02, false);
  }
  assert.ok(a.angleRad < b.angleRad);
  assert.ok(Math.abs(a.assistAngleRad) <= 0.18 + 1e-7);
});
test('vehicle remains finite during stop-start-reverse and sharp steering', () => {
  const v = new BicycleVehicle();
  for (let i = 0; i < 2000; i++) {
    const cycle = i % 400;
    v.step(control(Math.sin(i * 0.05), cycle < 180 ? 0.7 : 0, cycle >= 180 && cycle < 250 ? 0.8 : 0, cycle >= 250), 0.02, i < 1000 ? 0.85 : 0.35, true);
    for (const n of [v.x, v.y, v.u, v.v, v.heading, v.yawRate, v.betaRad, v.traction]) assert.ok(Number.isFinite(n));
    assert.ok(v.traction <= 1 + 1e-7);
    assert.ok(Math.abs(v.u) <= 24 + 1e-7);
  }
});
test('straight-line acceleration starts in forward direction', () => {
  const v = new BicycleVehicle();
  for (let i = 0; i < 100; i++) v.step(control(0, 0.6), 0.02);
  assert.ok(v.x > 1 && v.u > 0);
  assert.ok(Math.abs(v.y) < 0.01);
});
test('reverse starts from rest without false high-speed yaw', () => {
  const v = new BicycleVehicle();
  for (let i = 0; i < 80; i++) v.step(control(0.35, 0.4, 0, true), 0.02);
  assert.ok(v.u < 0 && v.x < 0);
  assert.ok(Math.abs(v.yawRate) < 0.5);
});
test('zero pedal at rest stays still', () => {
  const v = new BicycleVehicle();
  for (let i = 0; i < 200; i++) v.step(control(), 0.02);
  approx(v.u, 0); approx(v.x, 0); approx(v.y, 0);
});
