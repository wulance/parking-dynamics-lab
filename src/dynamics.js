/**
 * Planar bicycle demonstrator: input shaping + dynamic lateral tire forces.
 * This is an educational controller/vehicle prototype, NOT a real-car calibration.
 * All units SI (m, s, rad, N, kg), left/right symmetric.
 * Physics update reuses two preallocated force objects; UI may allocate separately.
 */
export const PARAMS = Object.freeze({
  massKg: 1500,
  yawInertiaKgM2: 2600,
  wheelbaseM: 2.7,
  frontComM: 1.15,
  rearComM: 1.55,
  maxSteerRad: 0.58,
  driveForceN: 4200,
  brakeForceN: 10500,
  dragNsPerM: 44,
  tireB: 9.8,
  tireC: 1.35,
  tireE: 0.45,
});
const G = 9.81;
const TWO_PI = Math.PI * 2;
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const sign = (v) => (v < 0 ? -1 : 1);
const toward = (value, target, maxDelta) =>
  value + clamp(target - value, -maxDelta, maxDelta);

/** Magic Formula lateral tire force, with peak bounded by mu * Fz. */
export function pacejkaLateral(slipAngleRad, normalLoadN, mu, tire = PARAMS) {
  if (normalLoadN <= 0 || mu <= 0) return 0;
  const bx = tire.tireB * slipAngleRad;
  const curve = Math.sin(tire.tireC * Math.atan(bx - tire.tireE * (bx - Math.atan(bx))));
  return -mu * normalLoadN * clamp(curve, -1, 1);
}

/** Circle/ellipse limiter; ensures Fx^2+Fy^2 <= (mu*Fz)^2 for equal mu axes. */
export function limitCombinedSlip(fx, fy, normalLoadN, mu) {
  const max = Math.max(0, mu * normalLoadN);
  if (max <= 0) return { fx: 0, fy: 0, utilization: 0 };
  const magnitude = Math.hypot(fx, fy);
  const scale = magnitude > max ? max / magnitude : 1;
  return { fx: fx * scale, fy: fy * scale, utilization: clamp(magnitude / max, 0, 1) };
}

/** Writes the same friction-circle result into preallocated mutable storage. */
function limitCombinedSlipInto(fx, fy, normalLoadN, mu, out) {
  const max = Math.max(0, mu * normalLoadN);
  if (max <= 0) {
    out.fx = 0; out.fy = 0; out.utilization = 0;
    return;
  }
  const magnitude = Math.hypot(fx, fy);
  const scale = magnitude > max ? max / magnitude : 1;
  out.fx = fx * scale;
  out.fy = fy * scale;
  out.utilization = clamp(magnitude / max, 0, 1);
}

/**
 * Driver steering maps a touch position [-1,1] into a speed-dependent steering
 * angle, with optional countersteer/yaw feedback. It changes INPUT only.
 */
export class SteeringController {
  constructor() {
    this.angleRad = 0;
    this.assistAngleRad = 0;
  }
  reset() {
    this.angleRad = 0;
    this.assistAngleRad = 0;
  }
  step(normalizedInput, speedMps, sideslipRad, yawRate, mu, dt, assisted = true, wheelbaseM = 2.7) {
    const speed = Math.abs(speedMps);
    const limit = clamp(0.58 / (1 + 0.012 * speed * speed), 0.12, 0.58);
    const shaped = sign(normalizedInput) * Math.pow(clamp(Math.abs(normalizedInput), 0, 1), 1.55);
    const driverAngle = shaped * limit;
    const referenceYaw = clamp(
      (speedMps * Math.tan(driverAngle)) / wheelbaseM,
      -(mu * G) / Math.max(speed, 2),
      (mu * G) / Math.max(speed, 2),
    );
    const authority = assisted ? clamp((speed - 1.5) / 2, 0, 1) : 0;
    // Smoothed, bounded countersteering. This does not teleport body state.
    const rawAssist = clamp(-0.75 * sideslipRad - 0.22 * (yawRate - referenceYaw), -0.18, 0.18) * authority;
    this.assistAngleRad = toward(this.assistAngleRad, rawAssist, 1.2 * dt);
    const target = clamp(driverAngle + this.assistAngleRad, -limit, limit);
    const steerRate = clamp(2.3 / (1 + speed * 0.07), 0.45, 2.3);
    this.angleRad = toward(this.angleRad, target, steerRate * dt);
    return this.angleRad;
  }
}

/** State is x/y in world metres, heading in radians, local body u/v in m/s. */
export class BicycleVehicle {
  constructor(config = PARAMS) {
    this.cfg = config;
    this.steering = new SteeringController();
    this._frontForce = { fx: 0, fy: 0, utilization: 0 };
    this._rearForce = { fx: 0, fy: 0, utilization: 0 };
    this.reset();
  }
  reset() {
    this.x = 0; this.y = 0; this.heading = 0;
    this.u = 0; this.v = 0; this.yawRate = 0;
    this.steerRad = 0; this.betaRad = 0; this.traction = 0;
    this.lastFyFrontN = 0; this.lastFyRearN = 0;
    this.steering.reset();
  }
  /**
   * Input: {steer:-1..1, throttle:0..1, brake:0..1, reverse:boolean}.
   * The instance is mutated; callers don't have to allocate on each step.
   */
  step(input, dt = 0.02, mu = 0.88, assisted = true) {
    if (!(dt > 0 && dt <= 0.05)) throw new RangeError('Use a fixed step in (0, 0.05] seconds');
    mu = clamp(mu, 0.05, 1.25);
    const p = this.cfg;
    const absU = Math.abs(this.u);
    this.betaRad = absU < 1.5 ? 0 : Math.atan2(this.v, absU);
    this.steerRad = this.steering.step(input.steer, this.u, this.betaRad, this.yawRate, mu, dt, assisted, p.wheelbaseM);

    const frontFz = p.massKg * G * p.rearComM / p.wheelbaseM;
    const rearFz = p.massKg * G * p.frontComM / p.wheelbaseM;
    const vxReg = Math.max(absU, 1.5);
    const frontAlpha = Math.atan2(this.v + p.frontComM * this.yawRate, vxReg) - this.steerRad;
    const rearAlpha = Math.atan2(this.v - p.rearComM * this.yawRate, vxReg);
    let frontFy = pacejkaLateral(frontAlpha, frontFz, mu, p);
    let rearFy = pacejkaLateral(rearAlpha, rearFz, mu, p);

    const reverse = !!input.reverse;
    const accelCmd = clamp(input.throttle, 0, 1);
    const brakeCmd = clamp(input.brake, 0, 1);
    const forwardIntent = reverse ? -1 : 1;
    // Braking always opposes current rolling direction; clamp at zero below.
    const appliedBrake = absU > 0.03 ? sign(this.u) * brakeCmd * p.brakeForceN : 0;
    const totalFx = accelCmd * forwardIntent * p.driveForceN - appliedBrake;
    // Half each axle for the demo. Full 4-wheel load transfer is intentionally omitted.
    const front = this._frontForce, rear = this._rearForce;
    limitCombinedSlipInto(totalFx * 0.5, frontFy, frontFz, mu, front);
    limitCombinedSlipInto(totalFx * 0.5, rearFy, rearFz, mu, rear);
    frontFy = front.fy; rearFy = rear.fy;
    this.lastFyFrontN = frontFy;
    this.lastFyRearN = rearFy;
    this.traction = Math.max(front.utilization, rear.utilization);

    const c = Math.cos(this.steerRad);
    const s = Math.sin(this.steerRad);
    const frontLong = front.fx * c - frontFy * s;
    const frontLat = front.fx * s + frontFy * c;
    const longForce = frontLong + rear.fx - p.dragNsPerM * this.u;
    const latForce = frontLat + rearFy;
    let udot = longForce / p.massKg + this.v * this.yawRate;
    let vdot = latForce / p.massKg - this.u * this.yawRate;
    let rdot = (p.frontComM * frontLat - p.rearComM * rearFy) / p.yawInertiaKgM2;

    // At parking speed, velocity-derived tire slip is ill-conditioned.
    // Blend toward the no-slip bicycle reference to prevent zero-speed yaw jitter.
    const dynWeight = clamp((absU - 0.75) / 2.0, 0, 1);
    const desiredYaw = this.u * Math.tan(this.steerRad) / p.wheelbaseM;
    const prevU = this.u;
    this.u += udot * dt;
    if (brakeCmd > 0 && accelCmd < 0.01 && prevU * this.u < 0) this.u = 0;
    this.u = clamp(this.u, -13, 24);
    const dynV = this.v + vdot * dt;
    const dynYaw = this.yawRate + rdot * dt;
    this.v = dynV * dynWeight;
    this.yawRate = dynYaw * dynWeight + desiredYaw * (1 - dynWeight);
    this.heading += this.yawRate * dt;
    if (this.heading > Math.PI) this.heading -= TWO_PI;
    if (this.heading < -Math.PI) this.heading += TWO_PI;
    const cs = Math.cos(this.heading), sn = Math.sin(this.heading);
    this.x += (cs * this.u - sn * this.v) * dt;
    this.y += (sn * this.u + cs * this.v) * dt;
    if (Math.abs(this.u) < 0.005 && accelCmd === 0) this.u = 0;
    return this;
  }
}
