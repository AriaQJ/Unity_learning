using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RRTPathPlanner : MonoBehaviour
{
    public VoxelGridDrawer voxelGrid; // Reference to the voxel grid script
    public Transform startTransform; // Starting Transform (end effector)
    public Transform goalTransform; // Goal Transform (target object)
    public float stepSize = 0.5f; // Step size for tree expansion
    public int maxIterations = 10000; // Maximum number of iterations
    public float goalThreshold = 1.0f; // Distance threshold to consider goal reached

    public Material lineMaterial; // Material for the path lines

    private List<Node> path; // Final path from start to goal
    private bool pathDrawn = false; // Flag to check if path is drawn

    public AdvancedPlaning advancedPlaning;

    void Start()
    {
        if (voxelGrid == null)
        {
            Debug.LogError("VoxelGridDrawer reference is missing.");
            return;
        }

        if (startTransform == null || goalTransform == null)
        {
            Debug.LogError("Start or Goal Transform is missing.");
            return;
        }

        voxelGrid.GenerateVoxelData(); // Ensure voxel data is up-to-date
    }

    // Method to be called by the button to run RRT
    public void RunRRT()
    {
        path = RRTAlgorithm();

        if (path != null)
        {
            Debug.Log("Path found with " + path.Count + " nodes.");
            DrawPath(); // Draw the path in the Game view

            // Pass the path nodes to the IKController
            List<Vector3> pathPositions = new List<Vector3>();
            foreach (Node node in path)
            {
                pathPositions.Add(node.position);
            }
            advancedPlaning.pathNodes = pathPositions;
        }
        else
        {
            Debug.Log("Failed to find a path.");
        }
    }

    // Node class for RRT
    class Node
    {
        public Vector3 position;
        public Node parent;

        public Node(Vector3 pos, Node parent = null)
        {
            this.position = pos;
            this.parent = parent;
        }
    }

    List<Node> RRTAlgorithm()
    {
        List<Node> tree = new List<Node>();
        Node startNode = new Node(startTransform.position);
        tree.Add(startNode);

        for (int i = 0; i < maxIterations; i++)
        {
            // Sample random point
            Vector3 randomPoint = SampleRandomPoint();

            // Find nearest node in the tree
            Node nearestNode = FindNearestNode(tree, randomPoint);

            // Generate new node towards random point
            Vector3 direction = (randomPoint - nearestNode.position).normalized;
            Vector3 newPoint = nearestNode.position + direction * stepSize;

            // Check if new point is in free space
            if (IsInFreeSpace(newPoint))
            {
                // Check if path from nearestNode to newPoint is collision-free
                if (IsCollisionFree(nearestNode.position, newPoint))
                {
                    Node newNode = new Node(newPoint, nearestNode);
                    tree.Add(newNode);

                    // Check if goal is reached
                    if (Vector3.Distance(newNode.position, goalTransform.position) <= goalThreshold)
                    {
                        // Goal reached, construct path
                        Node goalNode = new Node(goalTransform.position, newNode);
                        return ConstructPath(goalNode);
                    }
                }
            }
        }

        // Failed to find a path within maxIterations
        return null;
    }

    Vector3 SampleRandomPoint()
    {
        // Sample random point within grid bounds
        Vector3 min = voxelGrid.gridCenter - voxelGrid.gridSize / 2f;
        Vector3 max = voxelGrid.gridCenter + voxelGrid.gridSize / 2f;

        float x = Random.Range(min.x, max.x);
        float y = Random.Range(voxelGrid.gridCenter.y, max.y); // Only sample y >= gridCenter.y
        float z = Random.Range(min.z, max.z);

        return new Vector3(x, y, z);
    }

    Node FindNearestNode(List<Node> tree, Vector3 point)
    {
        Node nearest = null;
        float minDist = Mathf.Infinity;

        foreach (Node node in tree)
        {
            float dist = Vector3.Distance(node.position, point);
            if (dist < minDist)
            {
                minDist = dist;
                nearest = node;
            }
        }

        return nearest;
    }

    bool IsInFreeSpace(Vector3 point)
    {
        // Convert world position to voxel indices
        int x, y, z;
        if (voxelGrid.WorldToGridIndex(point, out x, out y, out z))
        {
            int occupancy = voxelGrid.voxelData[x, y, z];
            return occupancy == 0;
        }
        else
        {
            // Point is outside the grid
            return false;
        }
    }

    bool IsCollisionFree(Vector3 start, Vector3 end)
    {
        // Perform collision checking along the line from start to end
        //int numSamples = Mathf.CeilToInt(Vector3.Distance(start, end) / voxelGrid.cellSize);
        //for (int i = 0; i <= numSamples; i++)
        //{
        //    float t = (float)i / numSamples;
        //    Vector3 point = Vector3.Lerp(start, end, t);
        //    if (!IsInFreeSpace(point))
        //    {
        //        return false; // Collision detected
        //    }
        //}
        //return true; // No collision

            Vector3 direction = end - start;
            float distance = direction.magnitude;
            direction.Normalize();

            // 定义一个 LayerMask，只检测障碍物层
            int obstacleLayerMask = LayerMask.GetMask("Obstacle");

            // 检查从 start 到 end 之间是否有障碍物
            if (Physics.Raycast(start, direction, distance, obstacleLayerMask))
            {
                // 检测到障碍物，路径不可行
                return false;
            }
            else
            {
                // 没有检测到障碍物，路径可行
                return true;
            }
    }

    List<Node> ConstructPath(Node goalNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = goalNode;
        while (currentNode != null)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse(); // Reverse the path to start from the starting node
        return path;
    }

    // Draw the path using LineRenderer
    void DrawPath()
    {
        if (pathDrawn)
        {
            // Remove existing path
            Destroy(transform.Find("RRTPath").gameObject);
        }

        GameObject pathObject = new GameObject("RRTPath");
        pathObject.transform.parent = this.transform;

        LineRenderer lr = pathObject.AddComponent<LineRenderer>();
        lr.material = lineMaterial;
        lr.startWidth = 0.05f;
        lr.endWidth = 0.05f;
        lr.positionCount = path.Count;

        Vector3[] positions = new Vector3[path.Count];
        for (int i = 0; i < path.Count; i++)
        {
            positions[i] = path[i].position;
        }
        lr.SetPositions(positions);

        pathDrawn = true;
    }
}

// Extension methods for VoxelGridDrawer
public static class VoxelGridExtensions
{
    public static bool WorldToGridIndex(this VoxelGridDrawer voxelGrid, Vector3 position, out int x, out int y, out int z)
    {
        Vector3 localPos = position - voxelGrid.gridCenter + voxelGrid.gridSize / 2f;

        x = Mathf.FloorToInt(localPos.x / voxelGrid.cellSize);
        y = Mathf.FloorToInt(localPos.y / voxelGrid.cellSize);
        z = Mathf.FloorToInt(localPos.z / voxelGrid.cellSize);

        // Check if indices are within bounds
        if (x >= 0 && x < voxelGrid.voxelData.GetLength(0) &&
            y >= 0 && y < voxelGrid.voxelData.GetLength(1) &&
            z >= 0 && z < voxelGrid.voxelData.GetLength(2))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
