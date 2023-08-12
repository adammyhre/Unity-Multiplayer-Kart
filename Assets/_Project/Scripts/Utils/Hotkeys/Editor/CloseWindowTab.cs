using UnityEditor;
using UnityEngine;

namespace Utilities.Editor {
    public class CloseWindowTab : UnityEditor.Editor {
        [MenuItem("File/Close Window Tab")]
        static void CloseTab() {
            var focusedWindow = EditorWindow.focusedWindow;
            if (focusedWindow != null) {
                CloseTab(focusedWindow);
            } else {
                Debug.LogWarning("Found no focused window to close");
            }
        }

        static void CloseTab(EditorWindow editorWindow) {
            editorWindow.Close();
        }
    }
}
