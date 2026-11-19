namespace ParkingDynamics
{
    // SI units throughout; structs are passed by value or `in` without GC allocations.
    public struct DriverInput
    {
        public float Steering;   // [-1, 1], negative = left, positive = right
        public float Throttle;   // [0, 1]
        public float Brake;      // [0, 1]
        public bool Reverse;
    }

    public struct VehicleState
    {
        public float XMetres;
        public float YMetres;
        public float HeadingRad;
        public float LongitudinalMps;
        public float LateralMps;
        public float SideslipRad;
        public float YawRateRadPerSec;
        public float SteeringRad;
        public float FrictionUtilization;
    }

    public struct TireInput
    {
        public float SlipAngleRad;
        public float LongitudinalDemandN;
        public float NormalLoadN;
        public float FrictionCoefficient;
    }

    public struct TireForces
    {
        public float LongitudinalN;
        public float LateralN;
        public float Utilization;
    }

    public interface ITireForceModel
    {
        TireForces Evaluate(in TireInput input);
    }

    public interface ISteeringAssist
    {
        float Step(in DriverInput input, in VehicleState state, float mu, float deltaTime);
        void Reset();
    }

    public interface IVehiclePhysicsBackend
    {
        VehicleState State { get; }
        void Step(in DriverInput input, float deltaTime, float mu, bool assisted);
        void Reset();
    }
}
