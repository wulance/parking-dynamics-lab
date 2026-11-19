namespace ParkingDynamics
{
    /// <summary>
    /// Bounded yaw/sideslip feedback acts on steering INPUT, never on body velocity.
    /// Helper control only; disable near zero speed, especially when parking.
    /// </summary>
    public sealed class TouchSteeringAssist : ISteeringAssist
    {
        public bool Enabled = true;
        public float MaxAssistRad = 0.18f;
        public float YawGain = 0.22f;
        public float SideslipGain = 0.75f;
        public float WheelbaseM = 2.7f;
        private float _steeringRad;
        private float _assistRad;

        public float AssistAngleRad => _assistRad;
        public float SteeringRad => _steeringRad;

        public void Reset() { _steeringRad = 0f; _assistRad = 0f; }

        public float Step(in DriverInput input, in VehicleState state, float mu, float deltaTime)
        {
            float speed = MathEx.Abs(state.LongitudinalMps);
            float limit = MathEx.Clamp(0.58f / (1f + 0.012f * speed * speed), 0.12f, 0.58f);
            float raw = MathEx.Clamp(input.Steering, -1f, 1f);
            float shaped = MathEx.SignPositive(raw) * MathEx.Pow(MathEx.Abs(raw), 1.55f);
            float driverAngle = shaped * limit;
            float rReference = state.LongitudinalMps * MathEx.Tan(driverAngle) / WheelbaseM;
            float maxYaw = mu * MathEx.Gravity / MathEx.Clamp(speed, 2f, 100f);
            rReference = MathEx.Clamp(rReference, -maxYaw, maxYaw);

            float authority = Enabled ? MathEx.Clamp((speed - 1.5f) / 2f, 0f, 1f) : 0f;
            float assistTarget = MathEx.Clamp(
                -SideslipGain * state.SideslipRad - YawGain * (state.YawRateRadPerSec - rReference),
                -MaxAssistRad, MaxAssistRad) * authority;
            _assistRad = MathEx.Toward(_assistRad, assistTarget, 1.2f * deltaTime);
            float target = MathEx.Clamp(driverAngle + _assistRad, -limit, limit);
            float rate = MathEx.Clamp(2.3f / (1f + speed * .07f), .45f, 2.3f);
            _steeringRad = MathEx.Toward(_steeringRad, target, rate * deltaTime);
            return _steeringRad;
        }
    }
}
