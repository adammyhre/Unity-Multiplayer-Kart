using UnityEngine;

namespace Kart {
    [CreateAssetMenu(fileName = "AIDriverData", menuName = "Kart/AIDriverData")]
    public class AIDriverData : ScriptableObject {
        public float proximityThreshold = 20.0f; // How close we need to get to a waypoint for it to count as visited.
        public float updateCornerRange = 50f; // How close we need to get to a corner before we update the corner waypoint.
        public float brakeRange = 80f; // How close we need to get to a corner before we start braking.
        public float spinThreshold = 100f; // Angular velocity at which the AI will start counter-steering.
        public float speedWhileDrifting = 0.5f;
        public float timeToDrift = 0.5f;
    }
}