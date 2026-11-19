using System;

namespace ParkingDynamics
{
    /// <summary>
    /// Pure-C# planar bicycle demonstrator, fixed step 0.02s recommended.
    /// Roll/pitch, load transfer, contact patches and suspension are intentionally
    /// excluded: Unity WheelCollider provides those in the 3D integration path.
    /// </summary>
    public sealed class PlanarBicycleBackend : IVehiclePhysicsBackend
    {
        public float MassKg = 1500f;
        public float YawInertiaKgM2 = 2600f;
        public float FrontComM = 1.15f;
        public float RearComM = 1.55f;
        public float DriveForceN = 4200f;
        public float BrakeForceN = 10500f;
        public float DragNsPerM = 44f;

        public readonly PacejkaTireModel Tires = new PacejkaTireModel();
        public readonly TouchSteeringAssist Steering = new TouchSteeringAssist();
        private VehicleState _state;
        public VehicleState State => _state;
        public void Reset() { _state = default; Steering.Reset(); }

        public void Step(in DriverInput input, float deltaTime, float mu, bool assisted)
        {
            if (deltaTime <= 0f || deltaTime > .05f) throw new ArgumentOutOfRangeException(nameof(deltaTime));
            mu = MathEx.Clamp(mu, .05f, 1.25f);
            float absU = MathEx.Abs(_state.LongitudinalMps);
            _state.SideslipRad = absU < 1.5f ? 0f : MathEx.Atan2(_state.LateralMps, absU);
            Steering.Enabled = assisted;
            float L = FrontComM + RearComM;
            Steering.WheelbaseM = L;
            float delta = Steering.Step(in input, in _state, mu, deltaTime);

            float fzFront = MassKg * MathEx.Gravity * RearComM / L;
            float fzRear = MassKg * MathEx.Gravity * FrontComM / L;
            float vReg = MathEx.Clamp(absU, 1.5f, 100f);
            float alphaFront = MathEx.Atan2(_state.LateralMps + FrontComM * _state.YawRateRadPerSec, vReg) - delta;
            float alphaRear = MathEx.Atan2(_state.LateralMps - RearComM * _state.YawRateRadPerSec, vReg);
            float driveDirection = input.Reverse ? -1f : 1f;
            float brake = absU > .03f ? MathEx.SignPositive(_state.LongitudinalMps) * MathEx.Clamp(input.Brake, 0f, 1f) * BrakeForceN : 0f;
            float fxRequest = driveDirection * MathEx.Clamp(input.Throttle, 0f, 1f) * DriveForceN - brake;
            TireInput tireFront = new TireInput { SlipAngleRad = alphaFront, NormalLoadN = fzFront,
                FrictionCoefficient = mu, LongitudinalDemandN = fxRequest * .5f };
            TireInput tireRear = new TireInput { SlipAngleRad = alphaRear, NormalLoadN = fzRear,
                FrictionCoefficient = mu, LongitudinalDemandN = fxRequest * .5f };
            TireForces front = Tires.Evaluate(in tireFront);
            TireForces rear = Tires.Evaluate(in tireRear);

            float c = MathEx.Cos(delta), s = MathEx.Sin(delta);
            float fxFront = front.LongitudinalN * c - front.LateralN * s;
            float fyFront = front.LongitudinalN * s + front.LateralN * c;
            float du = (fxFront + rear.LongitudinalN - DragNsPerM * _state.LongitudinalMps) / MassKg
                        + _state.LateralMps * _state.YawRateRadPerSec;
            float dv = (fyFront + rear.LateralN) / MassKg - _state.LongitudinalMps * _state.YawRateRadPerSec;
            float dr = (FrontComM * fyFront - RearComM * rear.LateralN) / YawInertiaKgM2;
            float previousU = _state.LongitudinalMps;
            _state.LongitudinalMps = MathEx.Clamp(previousU + du * deltaTime, -13f, 24f);
            if (input.Brake > 0 && input.Throttle < .01f && previousU * _state.LongitudinalMps < 0f)
                _state.LongitudinalMps = 0f;

            // Blend to stable kinematic limit while stopping/reversing.
            float dynamicWeight = MathEx.Clamp((absU - .75f) / 2f, 0f, 1f);
            float desiredYaw = previousU * MathEx.Tan(delta) / L;
            _state.LateralMps = (_state.LateralMps + dv * deltaTime) * dynamicWeight;
            _state.YawRateRadPerSec = (_state.YawRateRadPerSec + dr * deltaTime) * dynamicWeight
                                     + desiredYaw * (1f - dynamicWeight);
            _state.HeadingRad += _state.YawRateRadPerSec * deltaTime;
            if (_state.HeadingRad > Math.PI) _state.HeadingRad -= (float)(2 * Math.PI);
            if (_state.HeadingRad < -Math.PI) _state.HeadingRad += (float)(2 * Math.PI);
            float ch = MathEx.Cos(_state.HeadingRad), sh = MathEx.Sin(_state.HeadingRad);
            _state.XMetres += (ch * _state.LongitudinalMps - sh * _state.LateralMps) * deltaTime;
            _state.YMetres += (sh * _state.LongitudinalMps + ch * _state.LateralMps) * deltaTime;
            _state.SteeringRad = delta;
            _state.FrictionUtilization = Math.Max(front.Utilization, rear.Utilization);
            if (MathEx.Abs(_state.LongitudinalMps) < .005f && input.Throttle <= 0f)
                _state.LongitudinalMps = 0f;
        }
    }
}
