using System;

namespace ParkingDynamics
{
    public static class MathEx
    {
        public const float Gravity = 9.81f;
        public static float Clamp(float v, float min, float max) => Math.Min(max, Math.Max(min, v));
        public static float Toward(float v, float target, float maxDelta) =>
            v + Clamp(target - v, -maxDelta, maxDelta);
        public static float Abs(float value) => Math.Abs(value);
        public static float Sqrt(float value) => (float)Math.Sqrt(value);
        public static float Sin(float value) => (float)Math.Sin(value);
        public static float Cos(float value) => (float)Math.Cos(value);
        public static float Tan(float value) => (float)Math.Tan(value);
        public static float Atan(float value) => (float)Math.Atan(value);
        public static float Atan2(float y, float x) => (float)Math.Atan2(y, x);
        public static float Pow(float b, float exponent) => (float)Math.Pow(b, exponent);
        public static float Hypot(float x, float y) => Sqrt(x * x + y * y);
        public static float SignPositive(float v) => v < 0 ? -1f : 1f;
    }
}
