using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectID : MonoBehaviour
{
    public string objectID;

    void Start()
    {
    
        objectID = System.Guid.NewGuid().ToString();
        Debug.Log("Object ID: " + objectID);
    }

    private void Update()
    {
        
    }
}

