using UnityEngine;
using UnityEditor;

public class ForceSaveSceneAndProject : MonoBehaviour {
    [MenuItem("File/Save Scene And Project %#&s")]
    static void FunctionForceSaveSceneAndProject() {
        EditorApplication.ExecuteMenuItem("File/Save");
        EditorApplication.ExecuteMenuItem("File/Save Project");
        Debug.Log("Saved scene and project");
    }
}