using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotArmController : MonoBehaviour
{
    public IKController ikController;            // 引用 IKController 脚本
    public EndEffectorPickUp endEffectorPickup;  // 引用 EndEffectorPickup 脚本
    public PickupTrigger pickupTrigger;          // 引用 PickupTrigger 脚本

    public Transform pickupPosition;             // 物品拾取位置
    public Transform standbyPosition;            // 待机位置
    public Transform dropOffPosition_Red;        // 红色物品放置位置
    public Transform dropOffPosition_Blue;       // 蓝色物品放置位置
    public Transform dropOffPosition_Green;      // 绿色物品放置位置

    void Start()
    {
        StartCoroutine(AutomationRoutine());
    }

    IEnumerator AutomationRoutine()
    {
        while (true)
        {
            // 移动到待机位置
            ikController.targetPosition = standbyPosition.position;
            yield return null; // 等待一帧，让目标位置更新
            yield return StartCoroutine(MoveArmToTarget());

            // 等待物品到达拾取位置
            yield return new WaitUntil(() => IsItemAtPickupPosition());

            // 移动到拾取位置
            ikController.targetPosition = pickupPosition.position;
            yield return null;
            yield return StartCoroutine(MoveArmToTarget());

            // 等待物品被拾取
            yield return new WaitUntil(() => endEffectorPickup.IsHoldingItem());

            // 获取物品类别
            string itemType = GetCurrentItemType();

            // 根据类别选择放置位置
            Vector3 dropOffPosition = GetDropOffPosition(itemType);
            ikController.targetPosition = dropOffPosition;
            yield return null;
            yield return StartCoroutine(MoveArmToTarget());

            // 释放物品
            //Debug.Log("release item");
            endEffectorPickup.ReleaseItem(dropOffPosition);
            yield return new WaitForSeconds(0.5f);
        }
    }

    IEnumerator MoveArmToTarget()
    {
        while (true)
        {
            // 调用 IKController 的 InverseKinematics 方法
            ikController.InverseKinematics(ikController.targetPosition);

            // 计算当前末端执行器的位置
            float[] currentJointAngles = ikController.ReadAngelDegree(ikController.articulationBodiesWithXDrive);
            float[] currentJointAnglesRadians = new float[currentJointAngles.Length];
            for (int i = 0; i < currentJointAngles.Length; i++)
            {
                currentJointAnglesRadians[i] = currentJointAngles[i] * Mathf.Deg2Rad;
            }
            Vector3 currentEndEffectorPos = ikController.ComputeEndEffectorPosition(currentJointAnglesRadians);

            // 判断是否到达目标位置
            float distance = Vector3.Distance(currentEndEffectorPos, ikController.targetPosition);
            if (distance < ikController.threshold)
            {
                yield break; // 达到目标位置，结束协程
            }
            yield return null; // 等待下一帧
        }
    }

    bool IsItemAtPickupPosition()
    {
        return pickupTrigger.IsItemInTrigger();
    }

    string GetCurrentItemType()
    {
        GameObject heldItem = endEffectorPickup.GetHeldItem();
        if (heldItem != null)
        {
            ItemProperties properties = heldItem.GetComponent<ItemProperties>();
            if (properties != null)
            {
                return properties.itemType;
            }
        }
        return null;
    }

    Vector3 GetDropOffPosition(string itemType)
    {
        switch (itemType)
        {
            case "Red":
                return dropOffPosition_Red.position;
            case "Blue":
                return dropOffPosition_Blue.position;
            case "Green":
                return dropOffPosition_Green.position;
            default:
                return standbyPosition.position;
        }
    }
}
