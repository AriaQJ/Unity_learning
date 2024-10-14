using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PDController : MonoBehaviour
{
    //articulation bodies
    public List<ArticulationBody> articulationBodiesWithXDrive = new List<ArticulationBody>();
    public ArticulationDriveType driveType;
    //public enum DriveType {Force, Acceleration, Target, TargetVelocity};

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        for (int i = 0; i < articulationBodiesWithXDrive.Count; i++)
        {
            SetDriveType(articulationBodiesWithXDrive[i], driveType);
        }
    }

    void SetDriveType(ArticulationBody ab, ArticulationDriveType driveType)
    {
        ArticulationDrive xDrive = ab.xDrive;
        xDrive.driveType = driveType;
        ab.xDrive = xDrive;
    }

}
