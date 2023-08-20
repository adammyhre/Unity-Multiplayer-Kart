using UnityEngine;

namespace Utilities {
    public static class Vector2Extensions {
        /// <summary>
        /// Sets any values of the Vector2
        /// </summary>
        public static Vector2 With(this Vector2 vector, float? x = null, float? y = null) {
            return new Vector2(x ?? vector.x, y ?? vector.y);
        }
    
        /// <summary>
        /// Adds to any values of the Vector3
        /// </summary>
        public static Vector2 Add(this Vector2 vector, float? x = null, float? y = null) {
            return new Vector2(vector.x + (x ?? 0), vector.y + (y ?? 0));
        }
        
        /// <summary>
        /// Converts a Vector2 to a Vector3 with a y value of 0.
        /// </summary>
        /// <param name="v2">The Vector2 to convert.</param>
        /// <returns>A Vector3 with the x and z values of the Vector2 and a y value of 0.</returns>
        public static Vector3 ToVector3(this Vector2 v2) {
            return new Vector3(v2.x, 0, v2.y);
        }
    }
}