using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForwardKinematicsController : MonoBehaviour
{
    public Transform[] joints; // Array of the 6 joint Transforms of UR5
    public LayerMask collisionLayer; // Layer mask for collision detection

    // Denavit-Hartenberg Parameters for UR5
    private readonly float[] d = { 0.1625f, 0, 0, 0.1333f, 0.0997f, 0.0996f };
    private readonly float[] a = { 0, -0.425f, -0.39225f, 0, 0, 0 };
    private readonly float[] alpha = { Mathf.PI / 2, 0, 0, Mathf.PI / 2, -Mathf.PI / 2, 0 };

    //joint angles degree
    [Range(-360f, 360f)] public float jointAngle0;
    [Range(-360f, 360f)] public float jointAngle1;
    [Range(-180f, 180f)] public float jointAngle2;
    [Range(-360f, 360f)] public float jointAngle3;
    [Range(-360f, 360f)] public float jointAngle4;
    [Range(-360f, 360f)] public float jointAngle5;

    private float[] jointAnglesRadians = new float[6];
    public Vector3 endEffectorPosition;

    // speed for the trajectory movement
    public float movementSpeed = 1.0f;

    // Start is called before the first frame update
    void Start()
    {
        if (jointAnglesRadians == null || jointAnglesRadians.Length != 6)
        {
            jointAnglesRadians = new float[6];
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Convert joint angles from degrees to radians
        jointAnglesRadians[0] = jointAngle0 * Mathf.Deg2Rad;
        jointAnglesRadians[1] = jointAngle1 * Mathf.Deg2Rad;
        jointAnglesRadians[2] = jointAngle2 * Mathf.Deg2Rad;
        jointAnglesRadians[3] = jointAngle3 * Mathf.Deg2Rad;
        jointAnglesRadians[4] = jointAngle4 * Mathf.Deg2Rad;
        jointAnglesRadians[5] = jointAngle5 * Mathf.Deg2Rad;

        endEffectorPosition = ComputeEndEffectorPosition(jointAnglesRadians);
        Debug.Log("End-Effector Position: " + endEffectorPosition);

        // Check for collisions
        if (IsInCollision(endEffectorPosition))
        {
            Debug.LogWarning("Computed pose is in collision bounds. Aborting trajectory.");
        }
        else
        {
            StartCoroutine(MoveToTargetPose(jointAnglesRadians));
        }
    }

    public Vector3 ComputeEndEffectorPosition(float[] jointAngles)
    {
        Matrix4x4 T = Matrix4x4.identity;

        for (int i = 0; i < jointAngles.Length; i++)
        {
            T *= DHMatrix(jointAngles[i], d[i], a[i], alpha[i]);
        }

        // Extract the position from the transformation matrix
        Vector3 position = T.GetColumn(3); // Get the translation vector (fourth column)
        return position;
    }

    private Matrix4x4 DHMatrix(float theta, float d, float a, float alpha)
    {
        float cosTheta = Mathf.Cos(theta);
        float sinTheta = Mathf.Sin(theta);
        float cosAlpha = Mathf.Cos(alpha);
        float sinAlpha = Mathf.Sin(alpha);

        Matrix4x4 T = new Matrix4x4();

        T.SetRow(0, new Vector4(cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta));
        T.SetRow(1, new Vector4(sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta));
        T.SetRow(2, new Vector4(0, sinAlpha, cosAlpha, d));
        T.SetRow(3, new Vector4(0, 0, 0, 1));

        return T;
    }

    // Check if the computed end-effector position is in collision
    private bool IsInCollision(Vector3 position)
    {
        // Use a small sphere cast to check for collisions at the target position
        float radius = 0.05f; // Radius for collision detection
        return Physics.CheckSphere(position, radius, collisionLayer);
    }

    private IEnumerator MoveToTargetPose(float[] targetJointAngles)
    {
        float[] currentAngles = new float[jointAnglesRadians.Length];
        for (int i = 0; i < jointAnglesRadians.Length; i++)
        {
            currentAngles[i] = jointAnglesRadians[i];
        }

        float t = 0;
        while (t < 1)
        {
            t += Time.deltaTime * movementSpeed;
            for (int i = 0; i < joints.Length; i++)
            {
                float interpolatedAngle = Mathf.Lerp(currentAngles[i], targetJointAngles[i], t);
                joints[i].localRotation = Quaternion.Euler(0, interpolatedAngle * Mathf.Rad2Deg, 0);
            }
            yield return null;
        }
    }
}
