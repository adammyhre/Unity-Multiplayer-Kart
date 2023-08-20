using UnityEngine;

namespace Kart {
    public interface IDrive {
        Vector2 Move { get; }
        bool IsBraking { get; }
        void Enable();
    }
}