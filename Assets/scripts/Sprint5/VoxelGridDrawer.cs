using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class VoxelGridDrawer : MonoBehaviour
{
    public Vector3 gridCenter = Vector3.zero; // Grid center at (0,0,0)
    public Vector3 gridSize = new Vector3(3, 2, 3); // Grid dimensions (in units)
    public float cellSize = 1f; // Size of each voxel cell (in units)

    public Color emptyColor = Color.green; // Color for empty cells
    public Color occupiedColor = Color.red; // Color for occupied cells

    public bool showGrid = true; // Option to show or hide the grid

    public int[,,] voxelData; // 3D matrix to store voxel occupancy data

    private void OnDrawGizmos()
    {
        if (showGrid)
        {
            GenerateVoxelData(); // Ensure voxel data is up-to-date
            DrawVoxelGrid();
        }
    }

    public void GenerateVoxelData()
    {
        // Calculate the number of cells in each dimension
        int cellsX = Mathf.RoundToInt(gridSize.x / cellSize);
        int cellsY = Mathf.RoundToInt(gridSize.y / cellSize);
        int cellsZ = Mathf.RoundToInt(gridSize.z / cellSize);

        // Initialize the voxel data matrix
        voxelData = new int[cellsX, cellsY, cellsZ];

        // Calculate the starting point of the grid so that gridCenter is at the center
        Vector3 gridStart = gridCenter - new Vector3((cellsX * cellSize) / 2f, 0f, (cellsZ * cellSize) / 2f);

        for (int x = 0; x < cellsX; x++)
        {
            for (int y = 0; y < cellsY; y++)
            {
                // Only generate cells where y >= gridCenter.y
                float yPosition = y * cellSize + cellSize / 2f + gridCenter.y;
                if (yPosition < gridCenter.y)
                    continue;

                for (int z = 0; z < cellsZ; z++)
                {
                    // Calculate the center position of the current cell
                    Vector3 cellCenter = gridStart + new Vector3(x * cellSize + cellSize / 2f, yPosition, z * cellSize + cellSize / 2f);

                    // Check for objects within the cell
                    Collider[] colliders = Physics.OverlapBox(cellCenter, Vector3.one * cellSize / 2f);

                    if (colliders.Length == 0)
                    {
                        // No objects, mark as empty
                        voxelData[x, y, z] = 0;
                    }
                    else
                    {
                        // Occupied
                        voxelData[x, y, z] = 1;
                    }
                }
            }
        }
    }

    void DrawVoxelGrid()
    {
        if (voxelData == null)
            GenerateVoxelData();

        int cellsX = voxelData.GetLength(0);
        int cellsY = voxelData.GetLength(1);
        int cellsZ = voxelData.GetLength(2);

        // Calculate the starting point of the grid
        Vector3 gridStart = gridCenter - new Vector3((cellsX * cellSize) / 2f, 0f, (cellsZ * cellSize) / 2f);

        for (int x = 0; x < cellsX; x++)
        {
            for (int y = 0; y < cellsY; y++)
            {
                float yPosition = y * cellSize + cellSize / 2f + gridCenter.y;
                if (yPosition < gridCenter.y)
                    continue;

                for (int z = 0; z < cellsZ; z++)
                {
                    Vector3 cellCenter = gridStart + new Vector3(x * cellSize + cellSize / 2f, yPosition, z * cellSize + cellSize / 2f);

                    // Determine cell color based on occupancy
                    Color cellColor;
                    int occupancy = voxelData[x, y, z];

                    if (occupancy == 0)
                    {
                        cellColor = emptyColor;
                    }
                    else // occupancy == 1
                    {
                        cellColor = occupiedColor;
                    }

                    // Draw the wireframe cube using Gizmos
                    DrawWireCube(cellCenter, Vector3.one * cellSize, cellColor);
                }
            }
        }
    }

    void DrawWireCube(Vector3 center, Vector3 size, Color color)
    {
        Gizmos.color = color;

        Vector3 halfSize = size / 2f;

        // 8 vertices of the cube
        Vector3[] vertices = new Vector3[8]
        {
            center + new Vector3(-halfSize.x, -halfSize.y, -halfSize.z),
            center + new Vector3( halfSize.x, -halfSize.y, -halfSize.z),
            center + new Vector3( halfSize.x, -halfSize.y,  halfSize.z),
            center + new Vector3(-halfSize.x, -halfSize.y,  halfSize.z),
            center + new Vector3(-halfSize.x,  halfSize.y, -halfSize.z),
            center + new Vector3( halfSize.x,  halfSize.y, -halfSize.z),
            center + new Vector3( halfSize.x,  halfSize.y,  halfSize.z),
            center + new Vector3(-halfSize.x,  halfSize.y,  halfSize.z),
        };

        // Draw edges of the cube
        int[,] edges = new int[12, 2]
        {
            {0,1}, {1,2}, {2,3}, {3,0}, // Bottom face
            {4,5}, {5,6}, {6,7}, {7,4}, // Top face
            {0,4}, {1,5}, {2,6}, {3,7}  // Vertical edges
        };

        for (int i = 0; i < edges.GetLength(0); i++)
        {
            Gizmos.DrawLine(vertices[edges[i, 0]], vertices[edges[i, 1]]);
        }
    }
}