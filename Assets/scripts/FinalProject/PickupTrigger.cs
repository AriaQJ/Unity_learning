using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PickupTrigger : MonoBehaviour
{
    private bool itemInTrigger = false;

    public string[] itemTags = { "GreenItem", "RedItem", "BlueItem" };

    void OnTriggerEnter(Collider other)
    {
        if (IsItemTag(other.tag))
        {
            //Debug.Log("objects enters the trigger：" + other.name);
            itemInTrigger = true;
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (IsItemTag(other.tag))
        {
            //Debug.Log("objects leaves the trigger：" + other.name);
            itemInTrigger = false;
        }
    }

    public bool IsItemInTrigger()
    {
        return itemInTrigger;
    }

    private bool IsItemTag(string tag)
    {
        foreach (string itemTag in itemTags)
        {
            if (tag == itemTag)
                return true;
        }
        return false;
    }
}
