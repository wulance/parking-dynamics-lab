using System;

namespace ParkingDynamics
{
    /// <summary>Preallocated ring buffer for telemetry on the physics thread.
    /// Export/serialize outside FixedUpdate to avoid GC or I/O stalls.</summary>
    public sealed class NoAllocTelemetryBuffer
    {
        private readonly VehicleState[] _buffer;
        private int _head;
        private int _count;
        public int Count => _count;
        public int Capacity => _buffer.Length;

        public NoAllocTelemetryBuffer(int capacity)
        {
            if (capacity <= 0) throw new ArgumentOutOfRangeException(nameof(capacity));
            _buffer = new VehicleState[capacity];
        }
        public void Record(in VehicleState state)
        {
            _buffer[_head] = state;
            _head = (_head + 1) % _buffer.Length;
            if (_count < _buffer.Length) _count++;
        }
        public VehicleState At(int chronologicalIndex)
        {
            if (chronologicalIndex < 0 || chronologicalIndex >= _count)
                throw new ArgumentOutOfRangeException(nameof(chronologicalIndex));
            int first = (_head - _count + _buffer.Length) % _buffer.Length;
            return _buffer[(first + chronologicalIndex) % _buffer.Length];
        }
    }
}
