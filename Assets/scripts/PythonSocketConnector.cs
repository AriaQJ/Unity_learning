using System.Threading;
using System.Collections.Concurrent;
using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Text;

public class PythonSocketConnector : MonoBehaviour
{
    public CameraCapture CaptureObject;

    bool IsInterrupted = false;
    readonly int ThreadTimeout = 100;
    readonly int MaxPacketSize = 1400;
    readonly string PacketSeparator = "<<EOM>>";

    ConcurrentQueue<byte[]> rgbQueue;
    ConcurrentQueue<byte[]> depthQueue;
    string LastIncomingMessage;

    Thread RGBThread;
    Thread DepthThread;
    Thread ReceivingThread;

    void Awake()
    {
        RGBThread = new Thread(RGBPublishingFunction) { IsBackground = true };
        DepthThread = new Thread(DepthPublishingFunction) { IsBackground = true };
        ReceivingThread = new Thread(ReceivingFunction) { IsBackground = true };
    }

    void Start()
    {
        // initialize queues
        rgbQueue = new();
        depthQueue = new();

        // start threads
        RGBThread.Start();
        DepthThread.Start();
        ReceivingThread.Start();
    }

    void Update()
    {
        if (CaptureObject.IsReadyToCapture())
        {
            rgbQueue.Enqueue(CaptureObject.GetRGBTextureBytes());
            depthQueue.Enqueue(CaptureObject.GetDepthTextureBytes());
        }
        Debug.Log(GetLastIncomingMessage());
    }

    void RGBPublishingFunction()
    {
        IPEndPoint endpoint = new(IPAddress.Loopback, 65400);
        UdpClient sender = new();

        while (IsInterrupted == false)
        {
            while (rgbQueue.TryDequeue(out byte[] frame))
            {
                frame = AppendEOM(frame);
                int numPacket = (int)Math.Ceiling((double)frame.Length / MaxPacketSize);
                for (int i = 0; i < numPacket; i++)
                {
                    int start = i * MaxPacketSize;
                    int end = Math.Min((i + 1) * MaxPacketSize, frame.Length);
                    byte[] packet = new byte[end - start];
                    Array.Copy(frame, start, packet, 0, end - start);
                    sender.Send(packet, packet.Length, endpoint);
                }
            }
        }
    }

    void DepthPublishingFunction()
    {
        IPEndPoint endpoint = new(IPAddress.Loopback, 65401);
        UdpClient sender = new();

        while (IsInterrupted == false)
        {
            while (depthQueue.TryDequeue(out byte[] frame))
            {
                frame = AppendEOM(frame);
                int numPacket = (int)Math.Ceiling((double)frame.Length / MaxPacketSize);
                for (int i = 0; i < numPacket; i++)
                {
                    int start = i * MaxPacketSize;
                    int end = Math.Min((i + 1) * MaxPacketSize, frame.Length);
                    byte[] packet = new byte[end - start];
                    Array.Copy(frame, start, packet, 0, end - start);
                    sender.Send(packet, packet.Length, endpoint);
                }
            }
        }
    }

    void ReceivingFunction()
    {
        IPEndPoint endpoint = new(IPAddress.Loopback, 65403);
        using (var client = new UdpClient(65402))
        {
            while(IsInterrupted == false)
            {
                byte[] data = client.Receive(ref endpoint);
                LastIncomingMessage = Encoding.UTF8.GetString(data);
            }
        }
    }

    void OnApplicationQuit() 
    {
        IsInterrupted = true;
        RGBThread.Interrupt();
        RGBThread.Join(ThreadTimeout);
        DepthThread.Interrupt();
        DepthThread.Join(ThreadTimeout);
        ReceivingThread.Interrupt();
        ReceivingThread.Join(ThreadTimeout);
    }

    byte[] AppendEOM(byte[] frame)
    {
        byte[] eom = Encoding.ASCII.GetBytes(PacketSeparator);
        byte[] eomd = new byte[frame.Length + eom.Length];
        Array.Copy(frame, eomd, frame.Length);
        Array.Copy(eom, 0, eomd, frame.Length, eom.Length);
        return eomd;
    }

    public string GetLastIncomingMessage() => LastIncomingMessage;
}
