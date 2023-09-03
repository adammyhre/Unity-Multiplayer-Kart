using UnityEngine;

namespace Kart {
    public class FaceCamera : MonoBehaviour {
        [SerializeField] Transform kartCamera; 

        void Update() {
            if (kartCamera) {
                transform.rotation = kartCamera.rotation;
            }
        }
    }
}