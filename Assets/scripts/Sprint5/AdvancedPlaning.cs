using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AdvancedPlaning : MonoBehaviour
{
    
     // Denavit-Hartenberg Parameters for UR5e
        private readonly float[] d = { 0.1625f, 0, 0, 0.1333f, 0.0997f, 0.0996f };
        private readonly float[] a = { 0, -0.425f, -0.39225f, 0, 0, 0 };
        private readonly float[] alpha = { Mathf.PI / 2, 0, 0, Mathf.PI / 2, -Mathf.PI / 2, 0 };

        public List<ArticulationBody> articulationBodiesWithXDrive = new List<ArticulationBody>();

        // 路径节点列表
        public List<Vector3> pathNodes = new List<Vector3>();
        private int currentNodeIndex = 0;

        // 学习参数
        public float learningRate = 0.01f;
        public int maxIterations = 100;
        public float threshold = 0.01f;

        // 标志位
        private bool isMoving = false;

        void Start()
        {
            // 初始化，如有必要
        }

        void Update()
        {
            // 可以在这里添加实时更新的逻辑
        }

        // 执行一个节点
        public void MoveToNextNode()
        {
        if (pathNodes == null || pathNodes.Count == 0)
        {
            Debug.LogError("Path nodes are empty or not assigned.");
            return;
        }
        if (isMoving || currentNodeIndex >= pathNodes.Count)
                return;

            StartCoroutine(MoveToNodeCoroutine(pathNodes[currentNodeIndex]));
            currentNodeIndex++;
        }

        // 执行所有节点
        public void MoveThroughAllNodes()
        {
            if (isMoving)
                return;

            StartCoroutine(MoveThroughAllNodesCoroutine());
        }

        IEnumerator MoveToNodeCoroutine(Vector3 targetPos)
        {
            isMoving = true;
            bool reached = false;

            while (!reached)
            {
                reached = InverseKinematics(targetPos);
                yield return null; // 等待下一帧
            }

            isMoving = false;
        }

        IEnumerator MoveThroughAllNodesCoroutine()
        {
            isMoving = true;

            while (currentNodeIndex < pathNodes.Count)
            {
                Vector3 targetPos = pathNodes[currentNodeIndex];
                bool reached = false;

                while (!reached)
                {
                    reached = InverseKinematics(targetPos);
                    yield return null; // 等待下一帧
                }

                currentNodeIndex++;
            }

            Debug.Log("Reached the target position.");
            isMoving = false;
        }

        // 修改 InverseKinematics 方法，返回是否到达目标
        bool InverseKinematics(Vector3 targetPosition)
        {
            float step = 0.001f;
            float[] theta = ReadAngelDegree(articulationBodiesWithXDrive);
            float[] thetaRadians = new float[6];
            float[] theta_i = new float[6];
            float[] theta_iRadians = new float[6];

            for (int iter = 0; iter < maxIterations; iter++)
            {
                for (int i = 0; i < theta.Length; i++)
                {
                    // Convert degrees to radians
                    for (int j = 0; j < theta.Length; j++)
                    {
                        thetaRadians[j] = theta[j] * Mathf.Deg2Rad;
                    }

                    Vector3 currentEndEffectorPos = ComputeEndEffectorPosition(thetaRadians);
                    float distance = Vector3.Distance(targetPosition, currentEndEffectorPos);
                    //Debug.Log("Current distance to target: " + distance);

                    if (distance < threshold)
                    {
                        // Apply the final joint angles
                        ApplyJointAngles(theta);
                        return true; // Target reached
                    }

                    // Perturb theta_i
                    theta_i = (float[])theta.Clone();
                    theta_i[i] += step;

                    // Convert degrees to radians
                    for (int j = 0; j < theta.Length; j++)
                    {
                        theta_iRadians[j] = theta_i[j] * Mathf.Deg2Rad;
                    }

                    float newDistance = Vector3.Distance(targetPosition, ComputeEndEffectorPosition(theta_iRadians));

                    // Gradient descent update
                    thetaRadians[i] -= ((newDistance - distance) / step) * learningRate;

                    // Convert radians back to degrees
                    for (int j = 0; j < theta.Length; j++)
                    {
                        theta[j] = thetaRadians[j] * Mathf.Rad2Deg;
                    }
                }
            }

            // Apply the joint angles even if threshold not met
            ApplyJointAngles(theta);
            return false; // Target not reached within maxIterations
        }

        void ApplyJointAngles(float[] jointAngles)
        {
            for (int i = 0; i < articulationBodiesWithXDrive.Count; i++)
            {
                ArticulationDrive drive = articulationBodiesWithXDrive[i].xDrive;
                drive.target = jointAngles[i];
                articulationBodiesWithXDrive[i].xDrive = drive;
            }
        }

        float[] ReadAngelDegree(List<ArticulationBody> ablist)
        {
            float[] jointAngles = new float[ablist.Count];
            for (int i = 0; i < ablist.Count; i++)
            {
                jointAngles[i] = ablist[i].xDrive.target;
            }
            return jointAngles;
        }

        public Vector3 ComputeEndEffectorPosition(float[] jointAngles)
        {
            Matrix4x4 dhMatrix = Matrix4x4.identity;

            for (int i = 0; i < jointAngles.Length; i++)
            {
                dhMatrix *= DHMatrix(jointAngles[i], d[i], a[i], alpha[i]);
            }

            // Extract the position from the transformation matrix
            Vector4 position = dhMatrix.GetColumn(3);
            float x = position.y;
            float y = position.z;
            float z = -position.x;

            //float x = -position.z;
            //float y = position.y;
            //float z = position.x;  
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
    }