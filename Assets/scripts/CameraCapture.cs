using System;
using UnityEngine;
using UnityEngine.Perception.GroundTruth;
using UnityEngine.Perception.GroundTruth.Sensors.Channels;

public class CameraCapture : MonoBehaviour
{
    public PerceptionCamera RGBDCamera;
    public RenderTexture RGBRenderTexture;
    public RenderTexture DepthRenderTexture;
    [Range(1, 95)] public int JPEGQuality = 80;

    Texture2D RGBTexture;
    Texture2D DepthTexture;
    bool IsReady = false;

    void Start() { }

    void Update()
    {
        JPEGQuality = Math.Min(JPEGQuality, 95);
        if (UpdateReadyStatus() == false)
        {
            try
            {
                RGBRenderTexture = RGBDCamera.GetChannel<RGBChannel>().outputTexture;
                RGBTexture = new Texture2D(RGBRenderTexture.width, RGBRenderTexture.height, TextureFormat.RGBA32, false);
                
                DepthRenderTexture = RGBDCamera.GetChannel<DepthChannel>().outputTexture;
                DepthTexture = new Texture2D(DepthRenderTexture.width, DepthRenderTexture.height, TextureFormat.RGBAFloat, false);

                UpdateRGBTexture();
                UpdateDepthTexture();
            }
            catch (InvalidOperationException) { }
        }
        else
        {
            UpdateRGBTexture();
            UpdateDepthTexture();
        }
    }

    bool UpdateReadyStatus()
    {
        if (RGBTexture != null && DepthTexture != null)
            IsReady = true;
        else 
            IsReady = false;
        
        return IsReady;
    }

    void UpdateRGBTexture()
    {
        RenderTexture.active = RGBRenderTexture;
        RGBTexture.ReadPixels(new Rect(0, 0, RGBRenderTexture.width, RGBRenderTexture.height), 0, 0);
        RGBTexture.Apply();
    }

    void UpdateDepthTexture()
    {
        RenderTexture.active = DepthRenderTexture;
        DepthTexture.ReadPixels(new Rect(0, 0, DepthRenderTexture.width, DepthRenderTexture.height), 0, 0);
        DepthTexture.Apply();
    }

    public byte[] GetRGBTextureBytes() => RGBTexture.EncodeToJPG(Math.Min(JPEGQuality, 95));

    public byte[] GetDepthTextureBytes() => DepthTexture.EncodeToEXR(Texture2D.EXRFlags.CompressZIP);

    public bool IsReadyToCapture() => IsReady;
}
