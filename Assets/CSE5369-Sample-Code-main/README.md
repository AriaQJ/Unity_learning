# Unity-Python RGBD Simulator

This package provide the ability to capture Unity's `PerceptionCamera` RGB and
depth image, and publish it to a Python endpoint via UDP for image processing.

> ⚠️ ***Please read the setup instructions carefully!***

## Setup Instructions

### 1. Install Unity High Definition Render Pipeline (HDRP)

Pick one of the following options:

![](/images/pacman.gif)
*Option 1: Importing Unity HDRP to existing projects*

![](/images/newproj.gif)
*Option 2: Creating new project with HDRP preinstalled*

### 2. Convert render materials to HDRP materials

This is necessary for `PerceptionCamera` to be able to see GameObjects. If there
is any `Fix All` option, select it. All checkmarks should be green on this
window.

![](/images/convert_materials.gif)
*Converting all project materials to HDRP compliant*

### 3. (Optional) Create and assign new materials as necessary

If your robots and objects does not render correctly i.e., it looks purple, it
means the shader for that material is wrong. The correct shader material needs
to be `HDRP/Lit`.

![](/images/create_material.gif)
*Creating new material and assign correct shader*

### 4. Install Unity Perception package

Add package via Unity package naming convention i.e., **`com.unity.perception`**

![](/images/import_perception.gif)
*Importing Unity Perception via Package Manager*

### 5. Import Unity-Python RGBD Capture Package

In this repository, the file
[CameraStreamer.unitypackage](./CameraStreamer.unitypackage) contains all the
necessary code and prefabs necessary to complete assignments. To import, go to
`Assets -> Import Package -> Custom Packages...`, which will open a file
selection dialog. Navigate to where you download the `.unitypackage` and select
it.

![](/images/import_package.gif)

### 6. Add RGBD camera into scene

The `RGBDCamera` prefab has all the necessary scripts pre-attached to it and can
simply be put into the scene. You can now move and rotate it as you would any
other `GameObject` in the scene to adjust your camera POV.

![](/images/add_camera.gif)

### 7. Verify Python communication

At this point, the camera will begin capture and send data to Python, while also
listening in for message from Python. You may uncomment:
1. line 53-54 of
the script to view the RGB image
2. line 90-91 to view the depth image

If everything is done correctly, you should see Unity console display  "test
message".

***You are to modify the code after line 120 in `opencv_receiver.py`.***

Additionally, you may chose to disable the debug prints in
`PythonSocketCommunicator.cs`, it is only there for testing purposes.

![](/images/test.gif)