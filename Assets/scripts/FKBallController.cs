using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FKBallController : MonoBehaviour
{   
    private FKRobotController fkRobotController;

    // Start is called before the first frame update
    void Start()
    {
        // Find FKRobotController component in the scene
        fkRobotController = FindObjectOfType<FKRobotController>();
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 targetPosition = fkRobotController.exepectedPosition;
        transform.position = Vector3.MoveTowards(transform.position, targetPosition, 5f);
    }
}
