using UnityEngine;

namespace ParkingDynamics
{
    /// <summary>
    /// Optional sample 3D backend: attach to a car GameObject with Rigidbody,
    /// four WheelColliders and four visual meshes. NO Pacejka AddForceAtPosition
    /// here: WheelCollider already computes its own tire forces.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public sealed class UnityWheelColliderBackend : MonoBehaviour
    {
        [Header("Wheels (FL / FR / RL / RR)")]
        public WheelCollider FrontLeft, FrontRight, RearLeft, RearRight;
        public Transform FrontLeftVisual, FrontRightVisual, RearLeftVisual, RearRightVisual;
        [Header("Input source supplied by your UI component")]
        [Range(-1f, 1f)] public float InputSteering;
        [Range(0f, 1f)] public float InputThrottle;
        [Range(0f, 1f)] public float InputBrake;
        public bool Reverse;
        public bool Assisted = true;
        [Range(.25f, 1.1f)] public float RoadMu = .88f;
        public float MotorTorquePerWheelNm = 280f;
        public float BrakeTorquePerWheelNm = 1700f;
        public float WheelbaseMetres = 2.7f;

        private Rigidbody _body;
        private readonly TouchSteeringAssist _steering = new TouchSteeringAssist();
        private VehicleState _state;

        private void Awake()
        {
            _body = GetComponent<Rigidbody>();
            _body.interpolation = RigidbodyInterpolation.Interpolate;
            // Called only once: settings apply to the vehicle, not once per wheel.
            if (FrontLeft != null) FrontLeft.ConfigureVehicleSubsteps(5f, 3, 1);
        }

        private void FixedUpdate()
        {
            if (FrontLeft == null || FrontRight == null || RearLeft == null || RearRight == null)
                return;

#if UNITY_6000_0_OR_NEWER
            Vector3 worldVel = _body.linearVelocity;
#else
            Vector3 worldVel = _body.velocity;
#endif
            Vector3 localVel = transform.InverseTransformDirection(worldVel);
            _state.LongitudinalMps = localVel.z;
            _state.LateralMps = localVel.x;
            _state.SideslipRad = Mathf.Abs(localVel.z) < 1.5f ? 0f : Mathf.Atan2(localVel.x, Mathf.Abs(localVel.z));
            _state.YawRateRadPerSec = Vector3.Dot(_body.angularVelocity, transform.up);
            DriverInput input = new DriverInput
            {
                Steering = InputSteering,
                Throttle = InputThrottle,
                Brake = InputBrake,
                Reverse = Reverse
            };
            _steering.Enabled = Assisted;
            _steering.WheelbaseM = WheelbaseMetres;
            float steerDegrees = _steering.Step(in input, in _state, RoadMu, Time.fixedDeltaTime) * Mathf.Rad2Deg;
            FrontLeft.steerAngle = steerDegrees;
            FrontRight.steerAngle = steerDegrees;

            // Start with simple FWD and equal per-wheel torque; refine drivetrain later.
            float torque = Mathf.Clamp01(InputThrottle) * MotorTorquePerWheelNm * (Reverse ? -1 : 1);
            FrontLeft.motorTorque = torque;
            FrontRight.motorTorque = torque;
            RearLeft.motorTorque = 0f;
            RearRight.motorTorque = 0f;
            float brakes = Mathf.Clamp01(InputBrake) * BrakeTorquePerWheelNm;
            FrontLeft.brakeTorque = brakes;
            FrontRight.brakeTorque = brakes;
            RearLeft.brakeTorque = brakes;
            RearRight.brakeTorque = brakes;
        }

        private void LateUpdate()
        {
            UpdateVisual(FrontLeft, FrontLeftVisual);
            UpdateVisual(FrontRight, FrontRightVisual);
            UpdateVisual(RearLeft, RearLeftVisual);
            UpdateVisual(RearRight, RearRightVisual);
        }
        private static void UpdateVisual(WheelCollider collider, Transform wheel)
        {
            if (collider == null || wheel == null) return;
            collider.GetWorldPose(out Vector3 pos, out Quaternion rotation);
            wheel.SetPositionAndRotation(pos, rotation);
        }
    }
}
