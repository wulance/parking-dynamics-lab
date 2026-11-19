// Requires Unity Test Framework (NUnit) and test-only assembly definition.
#if UNITY_INCLUDE_TESTS
using System;
using NUnit.Framework;

namespace ParkingDynamics.Tests
{
    public sealed class ParkingDynamicsTests
    {
        [Test]
        public void Pacejka_IsSymmetricAndBounded()
        {
            var model = new PacejkaTireModel();
            for (int i = 0; i <= 200; i++)
            {
                float angle = -1f + i * .01f;
                var a = new TireInput { SlipAngleRad = angle, NormalLoadN = 4000f, FrictionCoefficient = .8f };
                var b = new TireInput { SlipAngleRad = -angle, NormalLoadN = 4000f, FrictionCoefficient = .8f };
                var fa = model.Evaluate(in a);
                var fb = model.Evaluate(in b);
                Assert.That(fa.LateralN, Is.EqualTo(-fb.LateralN).Within(.02f));
                Assert.That(Math.Abs(fa.LateralN), Is.LessThanOrEqualTo(3200.01f));
            }
        }

        [Test]
        public void CounterSteer_IsBounded()
        {
            var steering = new TouchSteeringAssist();
            var state = new VehicleState { LongitudinalMps = 12, SideslipRad = .15f, YawRateRadPerSec = .2f };
            var input = new DriverInput();
            for (int i = 0; i < 200; i++) steering.Step(in input, in state, .8f, .02f);
            Assert.That(steering.AssistAngleRad, Is.LessThan(0));
            Assert.That(Math.Abs(steering.AssistAngleRad), Is.LessThanOrEqualTo(.181f));
        }

        [Test]
        public void SimulatedVehicle_StaysFinite()
        {
            var vehicle = new PlanarBicycleBackend();
            for (int i = 0; i < 1200; i++)
            {
                var input = new DriverInput { Steering = (float)Math.Sin(i*.04), Throttle = i%350 < 160 ? .7f : 0f,
                    Brake = i%350 >= 160 && i%350 < 250 ? .7f : 0f, Reverse = i%350 >= 250 };
                vehicle.Step(in input, .02f, i < 600 ? .9f : .35f, true);
                Assert.That(float.IsNaN(vehicle.State.XMetres), Is.False);
                Assert.That(float.IsInfinity(vehicle.State.YMetres), Is.False);
                Assert.That(vehicle.State.FrictionUtilization, Is.InRange(0f, 1f));
            }
        }

        [Test]
        public void TelemetryRingBuffer_OverwritesOldest()
        {
            var buffer = new NoAllocTelemetryBuffer(2);
            var a = new VehicleState { XMetres = 1f };
            var b = new VehicleState { XMetres = 2f };
            var c = new VehicleState { XMetres = 3f };
            buffer.Record(in a); buffer.Record(in b); buffer.Record(in c);
            Assert.That(buffer.Count, Is.EqualTo(2));
            Assert.That(buffer.At(0).XMetres, Is.EqualTo(2f));
            Assert.That(buffer.At(1).XMetres, Is.EqualTo(3f));
        }
    }
}
#endif
