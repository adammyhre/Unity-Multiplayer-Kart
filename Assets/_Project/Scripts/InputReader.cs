using UnityEngine;
using UnityEngine.InputSystem;
using static PlayerInputActions;

namespace Kart {
    [CreateAssetMenu(fileName = "InputReader", menuName = "Kart/Input Reader")]
    public class InputReader : ScriptableObject, IPlayerActions, IDrive {
        public Vector2 Move => inputActions.Player.Move.ReadValue<Vector2>();
        public bool IsBraking => inputActions.Player.Brake.ReadValue<float>() > 0;

        PlayerInputActions inputActions;
        
        void OnEnable() {
            if (inputActions == null) {
                inputActions = new PlayerInputActions();
                inputActions.Player.SetCallbacks(this);
            }
        }
        
        public void Enable() {
            inputActions.Enable();
        }
        
        public void OnMove(InputAction.CallbackContext context) {
            // noop
        }

        public void OnLook(InputAction.CallbackContext context) {
            // noop
        }

        public void OnFire(InputAction.CallbackContext context) {
            // noop
        }

        public void OnBrake(InputAction.CallbackContext context) {
            // noop
        }
    }
}
