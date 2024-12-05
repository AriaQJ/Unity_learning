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

    //public float[] jointAnglesRadians = new float[6];

    //articulation bodies
    public List<ArticulationBody> articulationBodiesWithXDrive = new List<ArticulationBody>();

    public Vector3 targetPosition;
    public Vector3 resultingPosition;
    //learning parameters
    public float learningRate = 0.01f;
    public int maxIterations = 100;
    public float threshold = 0.01f;
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        //GameObject ball = GameObject.Find("IK_Sephere");
        //targetPosition = ball.transform.position;
        //
        //jointAnglesRadians = ReadAngelDegree(articulationBodiesWithXDrive);
        //    resultingPosition = ComputeEndEffectorPosition(jointAnglesRadians);
        //    float x = resultingPosition.y;
        //    float y = resultingPosition.z;
        //    float z = -resultingPosition.x;
        //    resultingPosition = new Vector3(x, y, z);
        //IK to get angels
        //InverseKinematics(targetPosition);
        //apply to articulation body
        //ApplyJointAngles(jointAngels);
    }

    public void InverseKinematics(Vector3 targetPosition)
    {
        float step = 0.001f;
        float[] theta = new float[6];
        theta = ReadAngelDegree(articulationBodiesWithXDrive);
        float[] theta_i = theta;
        float[] thetaRadians = new float[6];
        float[] theta_iRadians = new float[6];

        for (int iter = 0; iter < maxIterations; iter++)
        {
            for (int i = 0; i < theta.Length; i++)
            {
                //degree to rad conversion
                for (int j = 0; j < theta.Length; j++)
                {
                    thetaRadians[j] = theta[j] * Mathf.Deg2Rad;
                }

                Vector3 currentEndEffectorPos = ComputeEndEffectorPosition(thetaRadians);
                float distance = Vector3.Distance(targetPosition, currentEndEffectorPos);

                if (distance < threshold)
                    break;
                theta_i = theta;
                theta_i[i] += step;

                //degree to rad conversion
                for (int j = 0; j < theta.Length; j++)
                {
                    theta_iRadians[j] = theta_i[j] * Mathf.Deg2Rad;
                }

                float newDistance = Vector3.Distance(targetPosition, ComputeEndEffectorPosition(theta_iRadians));

                thetaRadians[i] -= ((newDistance - distance) / step) * learningRate;


                for (int j = 0; j < theta.Length; j++)
                {
                    theta[j] = thetaRadians[j] * Mathf.Rad2Deg;
                }


            }
        }
        ApplyJointAngles(theta);
    }

    public void ApplyJointAngles(float[] jointAngles)
    {
        for (int i = 0; i < articulationBodiesWithXDrive.Count; i++)
        {
            ArticulationDrive drive = articulationBodiesWithXDrive[i].xDrive;
            drive.target = jointAngles[i];
            articulationBodiesWithXDrive[i].xDrive = drive;
        }
    }



    public float[] ReadAngelDegree(List<ArticulationBody> ablist)
    {
        float[] jointAngles = new float[ablist.Count];
        for (int i = 0; i < ablist.Count; i++)
        {
            jointAngles[i] = ablist[i].xDrive.target;
            //Debug.Log(ablist[i].xDrive.target);
        }
        return jointAngles;
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
        Vector4 position = dhMatrix.GetColumn(3);
        float x = position.y;
        float y = position.z;
        float z = -position.x;
        return new Vector3(x, y, z);
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

    //// Start is called before the first frame update

    //// Denavit-Hartenberg Parameters for UR5e
    //private readonly float[] d = { 0.1625f, 0, 0, 0.1333f, 0.0997f, 0.0996f };
    //private readonly float[] a = { 0, -0.425f, -0.39225f, 0, 0, 0 };
    //private readonly float[] alpha = { Mathf.PI / 2, 0, 0, Mathf.PI / 2, -Mathf.PI / 2, 0 };

    ////public float[] jointAnglesRadians = new float[6];

    ////articulation bodies
    //public List<ArticulationBody> articulationBodiesWithXDrive = new List<ArticulationBody>();

    //public Vector3 targetPosition;
    //public Vector3 resultingPosition;
    //public Transform endEffectorTransform;

    ////learning parameters
    //public float learningRate = 0.001f;    
    //public int maxIterations = 1000;      
    //public float threshold = 0.01f;


    //void Start()
    //{
    //    //articulationBodiesWithXDrive = new List<ArticulationBody>();
    //    //articulationBodiesWithXDrive.Add(GameObject.Find("Shoulder_Link").GetComponent<ArticulationBody>());
    //    //articulationBodiesWithXDrive.Add(GameObject.Find("UpperArm_Link").GetComponent<ArticulationBody>());
    //    //articulationBodiesWithXDrive.Add(GameObject.Find("Forearm_Link").GetComponent<ArticulationBody>());
    //    //articulationBodiesWithXDrive.Add(GameObject.Find("Wrist1_Link").GetComponent<ArticulationBody>());
    //    //articulationBodiesWithXDrive.Add(GameObject.Find("Wrist2_Link").GetComponent<ArticulationBody>());
    //    //articulationBodiesWithXDrive.Add(GameObject.Find("Wrist3_Link").GetComponent<ArticulationBody>());
    //}

    //// Update is called once per frame
    //void Update()
    //{
    //    //InverseKinematics(targetPosition);
    //    //Debug.Log($"")
    //}

    //public IEnumerator MoveToTargetPosition()
    //{
    //    for (int i = 0; i < maxIterations; i++)
    //    {
    //        InverseKinematics(targetPosition);
    //        // 判断机械臂末端是否到达目标位置
    //        Vector3 currentPos = ComputeEndEffectorPosition(ReadAngelRadians(articulationBodiesWithXDrive));
    //        //Vector3 currentPos = endEffectorTransform.position;
    //        //Debug.Log($"End Effector Position: {currentPos}");
    //        //Debug.Log($"Target Position: {targetPosition}");
    //        float distance = Vector3.Distance(currentPos, targetPosition);
    //        //Debug.Log($"distance: {distance}");
    //        if (distance < threshold)
    //        {
    //            Debug.Log("Inverse Kinematics converged.");
    //            yield break;
    //        }
    //        yield return null; // 等待下一帧
    //    }
    //}

    //void InverseKinematics(Vector3 targetPosition)
    //{
    //    float step = 0.001f;
    //    float[] theta = new float[6];
    //    theta = ReadAngelDegree(articulationBodiesWithXDrive);
    //    float[] theta_i = theta;
    //    float[] thetaRadians = new float[6];
    //    float[] theta_iRadians = new float[6];

    //    for (int iter = 0; iter < maxIterations; iter++)
    //    {
    //        for (int i = 0; i < theta.Length; i++)
    //        {   
    //            //degree to rad conversion
    //            for (int j = 0; j < theta.Length; j++)
    //            {
    //                thetaRadians[j] = theta[j] * Mathf.Deg2Rad;
    //            }

    //            Vector3 currentEndEffectorPos = ComputeEndEffectorPosition(thetaRadians);
    //            //Vector3 currentPos = endEffectorTransform.position;
    //            //Debug.Log($"Computed End Effector Position: {currentEndEffectorPos}");
    //            //Debug.Log($"real Position: {currentPos}");
    //            float distance = Vector3.Distance(targetPosition, currentEndEffectorPos);

    //            if (distance < threshold)
    //                break;
    //            theta_i = theta;
    //            theta_i[i] += step;

    //            //degree to rad conversion
    //            for (int j = 0; j < theta.Length; j++)
    //            {
    //                theta_iRadians[j] = theta_i[j] * Mathf.Deg2Rad;
    //            }

    //            float newDistance = Vector3.Distance(targetPosition, ComputeEndEffectorPosition(theta_iRadians));

    //            thetaRadians[i] -= ((newDistance - distance) / step) * learningRate;


    //            for (int j = 0; j < theta.Length; j++)
    //            {
    //                theta[j] = thetaRadians[j] * Mathf.Rad2Deg;
    //            }


    //        }
    //    }
    //    ApplyJointAngles(theta);
    //}

    //void ApplyJointAngles(float[] jointAngles)
    //{
    //    for (int i = 0; i < articulationBodiesWithXDrive.Count; i++)
    //    {
    //        ArticulationDrive drive = articulationBodiesWithXDrive[i].xDrive;
    //        drive.target = jointAngles[i];
    //        articulationBodiesWithXDrive[i].xDrive = drive;
    //    }
    //}



    //float[] ReadAngelDegree(List<ArticulationBody> ablist)
    //{
    //    float[] jointAngles = new float[ablist.Count];
    //    for (int i = 0; i < ablist.Count; i++)
    //    {
    //        jointAngles[i] = ablist[i].xDrive.target;
    //        //Debug.Log(ablist[i].xDrive.target);
    //    }
    //    return jointAngles;
    //}

    //float[] ReadAngelRadians(List<ArticulationBody> ablist)
    //{
    //    float[] jointAngles = new float[ablist.Count];
    //    for (int i = 0; i < ablist.Count; i++)
    //    {
    //        jointAngles[i] = ablist[i].xDrive.target * Mathf.Deg2Rad;
    //    }
    //    return jointAngles;
    //}
    ////float[] InverseKinematics(Vector3 exepectedPosition)
    ////{


    ////    //return float[] {0,0,0,0,0,0};
    ////}

    ////void ApplyJointAngels(float[] jointAngels)
    ////{

    ////}

    //public Vector3 ComputeEndEffectorPosition(float[] jointAngles)
    //{
    //    Matrix4x4 dhMatrix = Matrix4x4.identity;

    //    for (int i = 0; i < jointAngles.Length; i++)
    //    {
    //        dhMatrix *= DHMatrix(jointAngles[i], d[i], a[i], alpha[i]);
    //    }

    //    //unityMatrix.SetColumn(0, new Vector4(dhMatrix.m00, dhMatrix.m10, -dhMatrix.m20, 0)); 
    //    //unityMatrix.SetColumn(1, new Vector4(dhMatrix.m01, dhMatrix.m11, -dhMatrix.m21, 0)); 
    //    //unityMatrix.SetColumn(2, new Vector4(-dhMatrix.m02, -dhMatrix.m12, dhMatrix.m22, 0)); 
    //    //unityMatrix.SetColumn(3, new Vector4(dhMatrix.m03, dhMatrix.m13, -dhMatrix.m23, 1)); 

    //    // Extract the position from the transformation matrix
    //    //Vector4 position = dhMatrix.GetColumn(3);
    //    //float x = position.y;
    //    //float y = position.z;
    //    //float z = -position.x;

    //    //return new Vector3(x,y,z);
    //    // 提取位置
    //    Vector3 position = new Vector3(dhMatrix.m03, dhMatrix.m13, dhMatrix.m23);

    //    // 根据坐标系调整
    //    // 例如，如果需要交换 Y 和 Z 轴
    //    float temp = position.y;
    //    position.y = position.z;
    //    position.z = temp;
    //    position.x = -position.x;

    //    return position;
    //}

    //private Matrix4x4 DHMatrix(float theta, float d, float a, float alpha)
    //{
    //    float cosTheta = Mathf.Cos(theta);
    //    float sinTheta = Mathf.Sin(theta);
    //    float cosAlpha = Mathf.Cos(alpha);
    //    float sinAlpha = Mathf.Sin(alpha);

    //    Matrix4x4 T = new Matrix4x4();

    //    T.SetRow(0, new Vector4(cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta));
    //    T.SetRow(1, new Vector4(sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta));
    //    T.SetRow(2, new Vector4(0, sinAlpha, cosAlpha, d));
    //    T.SetRow(3, new Vector4(0, 0, 0, 1));

    //    return T;
    //}
}
