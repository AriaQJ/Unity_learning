using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class JointSliderController : MonoBehaviour
{
    public ArticulationBody joint; // Assign the joint in the Inspector
    public Slider slider;          // Assign the slider in the Inspector
    public float minAngle = -90f;
    public float maxAngle = 90f;
    // Start is called before the first frame update
    void Start()
    {
        // Configure the slider's range
        slider.minValue = minAngle;
        slider.maxValue = maxAngle;
        slider.onValueChanged.AddListener(OnSliderValueChanged);
    }

    void OnSliderValueChanged(float value)
    {
        // Update the joint's target position
        var drive = joint.xDrive;
        drive.target = value;
        joint.xDrive = drive;
    }
}
