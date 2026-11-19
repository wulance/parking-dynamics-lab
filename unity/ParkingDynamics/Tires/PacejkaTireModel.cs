namespace ParkingDynamics
{
    /// <summary>
    /// Educational simplified Magic Formula; no temperature/pressure/camber effects.
    /// NOT meant to be added to WheelCollider's own tire forces. Use EITHER this
    /// force model with a custom contact backend OR WheelCollider friction.
    /// </summary>
    public sealed class PacejkaTireModel : ITireForceModel
    {
        public float B = 9.8f;
        public float C = 1.35f;
        public float E = 0.45f;

        public TireForces Evaluate(in TireInput input)
        {
            TireForces f = default;
            float mu = MathEx.Clamp(input.FrictionCoefficient, 0f, 1.5f);
            float cap = mu * MathEx.Clamp(input.NormalLoadN, 0f, 1000000f);
            if (cap <= 0f) return f;

            float bx = B * input.SlipAngleRad;
            float curve = MathEx.Sin(C * MathEx.Atan(bx - E * (bx - MathEx.Atan(bx))));
            float lateral = -cap * MathEx.Clamp(curve, -1f, 1f);
            float longitudinal = input.LongitudinalDemandN;
            float norm = MathEx.Hypot(longitudinal, lateral);
            float scale = norm > cap ? cap / norm : 1f;
            f.LongitudinalN = longitudinal * scale;
            f.LateralN = lateral * scale;
            f.Utilization = MathEx.Clamp(norm / cap, 0f, 1f);
            return f;
        }
    }
}
