using System;
using UnityEngine;
using Utilities;

namespace Kart {
    public class AIInput : MonoBehaviour, IDrive {
        public Circuit circuit;
        public AIDriverData driverData;
        
        public Vector2 Move { get; private set; }
        public bool IsBraking { get; private set; }
        
        public void Enable() {
            // noop
        }

        int currentWaypointIndex;
        int currentCornerIndex;

        CountdownTimer driftTimer;

        float previousYaw; // The yaw of the kart in the previous frame.
        
        public void AddDriverData(AIDriverData data) => driverData = data;
        public void AddCircuit(Circuit circuit) => this.circuit = circuit;

        void Start() {
            if (circuit == null || driverData == null) {
                throw new ArgumentNullException($"AIInput requires a circuit and driver data to be set.");
            }
            previousYaw = transform.eulerAngles.y;
            driftTimer = new CountdownTimer(driverData.timeToDrift);
            driftTimer.OnTimerStart += () => IsBraking = true;
            driftTimer.OnTimerStop += () => IsBraking = false;
        }

        void Update() {
            driftTimer.Tick(Time.deltaTime);
            if (circuit.waypoints.Length == 0) {
                return;
            }

            // Calculate angular velocity
            float currentYaw = transform.eulerAngles.y;
            float deltaYaw = Mathf.DeltaAngle(previousYaw,currentYaw); // Make sure the values are in the right order
            float angularVelocity = deltaYaw / Time.deltaTime;
            previousYaw = currentYaw;

            Vector3 toNextPoint = circuit.waypoints[currentWaypointIndex].position - transform.position;
            Vector3 toNextCorner = circuit.waypoints[currentCornerIndex].position - transform.position;
            var distanceToNextPoint = toNextPoint.magnitude;
            var distanceToNextCorner = toNextCorner.magnitude;

            // If the kart comes within range of the next waypoint, move to the next waypoint.
            if (distanceToNextPoint < driverData.proximityThreshold) {
                currentWaypointIndex = (currentWaypointIndex + 1) % circuit.waypoints.Length;
            }

            // If the kart comes within range of the corner, update the corner waypoint.
            if (distanceToNextCorner < driverData.updateCornerRange) {
                currentCornerIndex = currentWaypointIndex;
            }

            // Drift logic
            if (distanceToNextCorner < driverData.brakeRange && !driftTimer.IsRunning) {
                driftTimer.Start();
            }

            // Adjust speed
            Move = Move.With(y: driftTimer.IsRunning ? driverData.speedWhileDrifting : 1f);

            // Adjust turning
            Vector3 desiredForward = toNextPoint.normalized;
            Vector3 currentForward = transform.forward;
            float turnAngle = Vector3.SignedAngle(currentForward, desiredForward, Vector3.up);

            // Updates 'Move' based on the value of 'turnAngle'
            Move = turnAngle switch {
                > 5f => Move.With(x: 1f),
                < -5f => Move.With(x: -1f),
                _ => Move.With(x: 0f)
            };

            // Counter-steer logic
            if (Mathf.Abs(angularVelocity) > driverData.spinThreshold) {
                Move = Move.With(x: -Mathf.Sign(angularVelocity));
                IsBraking = true;
            } else {
                IsBraking = false;
            }
        }
    }
}