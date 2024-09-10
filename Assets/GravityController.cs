using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GravityController : MonoBehaviour
{
    public Slider gravitySlider; // Reference to the slider
    public float minGravity = -20f; // Minimum gravity value
    public float maxGravity = 20f;  // Maximum gravity value

    void Start()
    {
        // Initialize slider value and add a listener to handle slider changes
        gravitySlider.onValueChanged.AddListener(UpdateGravity);
        gravitySlider.minValue = 0f;  // Set slider's minimum value
        gravitySlider.maxValue = 1f;  // Set slider's maximum value
        gravitySlider.value = 0.5f;   // Start slider in the middle
    }

    // This method will be called when the slider value changes
    void UpdateGravity(float value)
    {
        // Map slider value (0-1) to gravity range (minGravity-maxGravity)
        float newGravity = Mathf.Lerp(minGravity, maxGravity, value);
        Physics.gravity = new Vector3(0, newGravity, 0); // Update global gravity
        Debug.Log("Gravity changed to: " + newGravity); // Log the value for testing
    }
}