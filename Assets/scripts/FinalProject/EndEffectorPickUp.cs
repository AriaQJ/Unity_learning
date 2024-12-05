using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EndEffectorPickUp : MonoBehaviour
{
    private GameObject heldItem = null;

    public string[] itemTags = { "RedItem", "BlueItem", "GreenItem" };

    void OnTriggerEnter(Collider other)
    {
        if (heldItem == null && IsItemTag(other.tag))
        {
            heldItem = other.gameObject;
            heldItem.transform.parent = this.transform;
            Rigidbody rb = heldItem.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = true;
            }
            Collider col = heldItem.GetComponent<Collider>();
            if (col != null)
            {
                col.enabled = false;
            }
        }
    }

    public void ReleaseItem(Vector3 dropOffPosition)
    {
        if (heldItem != null)
        {
            // 解除父子关系
            heldItem.transform.SetParent(null);

            // 强制设置物品的位置和旋转到放置位置
            heldItem.transform.position = dropOffPosition;
            heldItem.transform.rotation = Quaternion.identity; // 根据需要设置旋转

            // 重置物理属性
            Rigidbody rb = heldItem.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = false; // 防止物品受物理引擎影响
                rb.velocity = Vector3.zero; // 清除物品的速度
                rb.angularVelocity = Vector3.zero; // 清除物品的角速度
            }

            // 启用物品的碰撞器
            Collider col = heldItem.GetComponent<Collider>();
            if (col != null)
            {
                col.enabled = true;
            }

            // 清空引用
            heldItem = null;
        }
    }


    public bool IsHoldingItem()
    {
        return heldItem != null;
    }

    public GameObject GetHeldItem()
    {
        return heldItem;
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