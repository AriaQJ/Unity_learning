using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FKRobotController : MonoBehaviour
{
    // Denavit-Hartenberg Parameters for UR5e
    private readonly float[] d = { 0.1625f, 0, 0, 0.1333f, 0.0997f, 0.0996f };
    private readonly float[] a = { 0, -0.425f, -0.39225f, 0, 0, 0 };
    private readonly float[] alpha = { Mathf.PI / 2, 0, 0, Mathf.PI / 2, -Mathf.PI / 2, 0 };

    //joint angles 
    [Range(-360f, 360f)] public float jointAngle0;
    [Range(-360f, 360f)] public float jointAngle1;
    [Range(-180f, 180f)] public float jointAngle2;
    [Range(-360f, 360f)] public float jointAngle3;
    [Range(-360f, 360f)] public float jointAngle4;
    [Range(-360f, 360f)] public float jointAngle5;
    private float[] jointAnglesRadians = new float[6];

    //articulation bodies
    public List<ArticulationBody> articulationBodiesWithXDrive = new List<ArticulationBody>();

    //end effector position
    public Vector3 exepectedPosition;
    public Vector3 resultingPosition;


    void Start()
    {
        
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

        // Change joint angles in articulation bodies
        for (int i = 0; i < articulationBodiesWithXDrive.Count; i++)
        {
            SetTarget(articulationBodiesWithXDrive[i], jointAnglesRadians[i]);
        }

        exepectedPosition = ComputeEndEffectorPosition(jointAnglesRadians);
        //float x = exepectedPosition.y;
        //float y = exepectedPosition.z;
        //float z = -exepectedPosition.x;
        //exepectedPosition = new Vector3(x, y, z);
        Debug.Log("exepectedPosition: " + exepectedPosition);


        
    }

    void SetTarget(ArticulationBody ab, float angle)
    {
        ArticulationDrive xDrive = ab.xDrive;
        xDrive.target = angle * Mathf.Rad2Deg;
        ab.xDrive = xDrive;
    }

    public Vector3 ComputeEndEffectorPosition(float[] jointAngles)
    {
        Matrix4x4 dhMatrix = Matrix4x4.identity;
        Matrix4x4 unityMatrix = Matrix4x4.identity;

        for (int i = 0; i < jointAngles.Length; i++)
        {
            dhMatrix *= DHMatrix(jointAngles[i], d[i], a[i], alpha[i]);
        }
        Vector4 position = dhMatrix.GetColumn(3);
        float x = position.y;
        float y = position.z;
        float z = -position.x;

        //dhMatrix = new Vector3(x, y, z);

        //dhMatrix = dhMatrix * T;

        //unityMatrix.SetColumn(0, new Vector4(dhMatrix.m00, dhMatrix.m10, -dhMatrix.m20, 0)); 
        //unityMatrix.SetColumn(1, new Vector4(dhMatrix.m01, dhMatrix.m11, -dhMatrix.m21, 0)); 
        //unityMatrix.SetColumn(2, new Vector4(-dhMatrix.m02, -dhMatrix.m12, dhMatrix.m22, 0)); 
        //unityMatrix.SetColumn(3, new Vector4(dhMatrix.m03, dhMatrix.m13, -dhMatrix.m23, 1)); 

        // Extract the position from the transformation matrix
        //Vector3 exepectedPosition = dhMatrix.GetColumn(3); // Get the translation vector (fourth column)
        return new Vector3 (x,y,z);
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
}
