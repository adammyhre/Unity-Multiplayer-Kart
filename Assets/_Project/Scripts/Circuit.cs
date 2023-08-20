using UnityEngine;

namespace Kart {
    [CreateAssetMenu(fileName = "CircuitData", menuName = "Kart/CircuitData")]
    public class Circuit : ScriptableObject {
        public Transform[] waypoints;
        public Transform[] spawnPoints;
    }
}