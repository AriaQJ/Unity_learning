using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKController : MonoBehaviour
{
    // Start is called before the first frame update

    // Denavit-Hartenberg Parameters for UR5e
    private readonly float[] d = { 0.1625f, 0, 0, 0.1333f, 0.0997f, 0.0996f };
    private readonly float[] a = { 0, -0.425f, -0.39225f, 0, 0, 0 };
    private readonly float[] alpha = { Mathf.PI / 2, 0, 0, Mathf.PI / 2, -Mathf.PI / 2, 0 };

    public float[] jointAnglesRadians = new float[6];

    //articulation bodies
    public List<ArticulationBody> articulationBodiesWithXDrive = new List<ArticulationBody>();

    public Vector3 targetPosition;
    public Vector3 resultingPosition;
    //learning parameters
    public float learningRate = 0.1f;    
    public int maxIterations = 100;      
    public float threshold = 0.01f;      
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        GameObject ball = GameObject.Find("IK_Sephere");
        targetPosition = ball.transform.position;
        //jointAnglesRadians = ReadAngelDegree(articulationBodiesWithXDrive);
        //    resultingPosition = ComputeEndEffectorPosition(jointAnglesRadians);
        //    float x = resultingPosition.y;
        //    float y = resultingPosition.z;
        //    float z = -resultingPosition.x;
        //    resultingPosition = new Vector3(x, y, z);
        //IK to get angels
        float[] jointAngels = InverseKinematics(targetPosition);
        //apply to articulation body
        ApplyJointAngles(jointAngels);

    }

    float[] InverseKinematics(Vector3 targetPosition)
    {
        float[] theta = new float[6];
        theta = ReadAngelDegree(articulationBodiesWithXDrive);

        for (int iter = 0; iter < maxIterations; iter++)
        {
            Vector3 currentEndEffectorPos = ComputeEndEffectorPosition(theta);

            Vector3 deltaPos =  targetPosition - currentEndEffectorPos;

            if (deltaPos.magnitude < threshold)
                break;

            float[,] jacobian = ComputeJacobian(theta);
            float[] deltaTheta = JacobianPseudoInverse(jacobian, deltaPos);

            for (int i = 0; i < theta.Length; i++)
            {
                theta[i] += learningRate * deltaTheta[i];
            }

        }

        return theta;
    }

    void ApplyJointAngles(float[] jointAngles)
    {
        for (int i = 0; i < articulationBodiesWithXDrive.Count; i++)
        {
            ArticulationDrive drive = articulationBodiesWithXDrive[i].xDrive;
            drive.target = jointAngles[i] * Mathf.Rad2Deg;
            articulationBodiesWithXDrive[i].xDrive = drive;
        }
    }



    float[] ReadAngelDegree(List<ArticulationBody> ablist)
    {
        float[] jointAnglesRadians = new float[ablist.Count];
        for (int i = 0; i < ablist.Count; i++)
        {
            jointAnglesRadians[i] = ablist[i].xDrive.target;
            //Debug.Log(ablist[i].xDrive.target);
        }
        return jointAnglesRadians;
    }

    //float[] InverseKinematics(Vector3 exepectedPosition)
    //{


    //    //return float[] {0,0,0,0,0,0};
    //}

    //void ApplyJointAngels(float[] jointAngels)
    //{

    //}

    public Vector3 ComputeEndEffectorPosition(float[] jointAngles)
    {
        Matrix4x4 dhMatrix = Matrix4x4.identity;
        Matrix4x4 unityMatrix = Matrix4x4.identity;

        for (int i = 0; i < jointAngles.Length; i++)
        {
            dhMatrix *= DHMatrix(jointAngles[i], d[i], a[i], alpha[i]);
        }

        //unityMatrix.SetColumn(0, new Vector4(dhMatrix.m00, dhMatrix.m10, -dhMatrix.m20, 0)); 
        //unityMatrix.SetColumn(1, new Vector4(dhMatrix.m01, dhMatrix.m11, -dhMatrix.m21, 0)); 
        //unityMatrix.SetColumn(2, new Vector4(-dhMatrix.m02, -dhMatrix.m12, dhMatrix.m22, 0)); 
        //unityMatrix.SetColumn(3, new Vector4(dhMatrix.m03, dhMatrix.m13, -dhMatrix.m23, 1)); 

        // Extract the position from the transformation matrix
        Vector3 exepectedPosition = dhMatrix.GetColumn(3); // Get the translation vector (fourth column)
        float x = exepectedPosition.y;
        float y = exepectedPosition.z;
        float z = -exepectedPosition.x;
        exepectedPosition = new Vector3(x, y, z);
        return exepectedPosition;
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
    private float[,] ComputeJacobian(float[] jointAngles)
    {
        float delta = 0.001f;
        float[,] jacobian = new float[3, jointAngles.Length];

        Vector3 originalPosition = ComputeEndEffectorPosition(jointAngles);

        for (int i = 0; i < jointAngles.Length; i++)
        {
            float originalAngle = jointAngles[i];
            jointAngles[i] += delta;
            Vector3 newPosition = ComputeEndEffectorPosition(jointAngles);
            Vector3 diff = (newPosition - originalPosition) / delta;

            jacobian[0, i] = diff.x;
            jacobian[1, i] = diff.y;
            jacobian[2, i] = diff.z;

            jointAngles[i] = originalAngle;
        }

        return jacobian;
    }

    private float[] JacobianPseudoInverse(float[,] jacobian, Vector3 deltaPosition)
    {
        int rows = jacobian.GetLength(0);
        int cols = jacobian.GetLength(1);
        float[] deltaTheta = new float[cols];

        for (int i = 0; i < cols; i++)
        {
            float sum = 0;
            for (int j = 0; j < rows; j++)
            {
                sum += jacobian[j, i] * deltaPosition[j];
            }
            deltaTheta[i] = sum;
        }

        return deltaTheta;
    }
}
