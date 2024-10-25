using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Newtonsoft.Json.Linq;

public class UDPReceiver : MonoBehaviour
{
    // Start is called before the first frame update
    Thread receiveThread;
    UdpClient client;
    int port = 65402; // Must match the destination_port in your Python script
    bool dataReceived = false;
    string receivedData = "";

    public Vector3 targetPosition;

    void Start()
    {
        StartReceiver();
    }

    void StartReceiver()
    {
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void ReceiveData()
    {
        client = new UdpClient(port);
        while (true)
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = client.Receive(ref anyIP);
                receivedData = Encoding.UTF8.GetString(data);
                dataReceived = true;
            }
            catch (Exception err)
            {
                Debug.LogError(err.ToString());
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (dataReceived)
        {
            dataReceived = false;
            ParseData(receivedData);
        }
    }

    void ParseData(string jsonData)
    {
        try
        {
            // Parse the JSON data
            JArray detectedObjects = JArray.Parse(jsonData);

            // Assuming you only care about the first detected object
            if (detectedObjects.Count > 0)
            {
                JObject obj = (JObject)detectedObjects[0];

                float X = obj["world"][0].ToObject<float>();
                float Y = obj["world"][1].ToObject<float>();
                float Z = obj["world"][2].ToObject<float>();

                // Unity's coordinate system may differ; adjust if necessary
                targetPosition = new Vector3(X, Y, Z);

                // Optionally, you can send this targetPosition to your IKController
                IKController ikController = GetComponent<IKController>();
                if (ikController != null)
                {
                    ikController.targetPosition = targetPosition;
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error parsing data: {e.Message}");
        }
    }

    void OnApplicationQuit()
    {
        if (receiveThread != null)
            receiveThread.Abort();
        if (client != null)
            client.Close();
    }
}
