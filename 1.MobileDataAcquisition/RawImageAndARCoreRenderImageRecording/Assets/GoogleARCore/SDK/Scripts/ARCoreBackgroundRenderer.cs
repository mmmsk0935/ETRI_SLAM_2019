//-----------------------------------------------------------------------
// <copyright file="ARCoreBackgroundRenderer.cs" company="Google">
//
// Copyright 2017 Google Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// </copyright>
//-----------------------------------------------------------------------

namespace GoogleARCore
{
    using System.Collections;
    using System.Collections.Generic;
    using GoogleARCoreInternal;
    using UnityEngine;
    using UnityEngine.Rendering;

    // YSKWAK-S
    using System;
    using System.IO;
    //using System.IO.Ports;

    //using IMU_INS;
    // YSKWAK

    /// <summary>
    /// Renders the device's camera as a background to the attached Unity camera component.
    /// When using the front-facing (selfie) camera, this temporarily inverts culling when
    /// rendering.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    [HelpURL("https://developers.google.com/ar/reference/unity/class/GoogleARCore/" +
             "ARCoreBackgroundRenderer")]
    public class ARCoreBackgroundRenderer : MonoBehaviour
    {
        /// <summary>
        /// A material used to render the AR background image.
        /// </summary>
        [Tooltip("A material used to render the AR background image.")]
        public Material BackgroundMaterial;

        private static readonly float k_BlackScreenDuration = 0.5f;

        private static readonly float k_FadingInDuration = 0.5f;

        private Camera m_Camera;

        private Texture m_TransitionImageTexture;

        private BackgroundTransitionState m_TransitionState = BackgroundTransitionState.BlackScreen;

        private float m_CurrentStateElapsed = 0.0f;

        private bool m_SessionEnabled = false;

        private bool m_UserInvertCullingValue = false;

        private CameraClearFlags m_CameraClearFlags = CameraClearFlags.Skybox;

        private CommandBuffer m_CommandBuffer = null;


        // YSKWAK-S
        private static int siRenderImageCount= 0;
        private static int siRawImageCount = 0;
        private StreamWriter sw = null;
        private StreamWriter swIMUdata = null;
        private StreamWriter swLog = null;
        private StreamWriter swLog2 = null;

        Texture2D rawTex;

        int bufferSize;
        int s_ImageBufferSize = 0;
        byte[] s_ImageBuffer = null;

        int width;
        int height;
        int rowStride;
        CameraImageBytes image;

        byte[] BitmapBytes = null;
        byte[] bImageBuffer24 = null;
        byte[] bytesJPG = null;

        Camera Cam = null;
        RenderTexture rtex = null;
        RenderTexture currentRT = null;
        Texture2D Image;
        byte[] bytes = null;

        System.DateTime dtCurrentTime;
        // YSKWAK

        private enum BackgroundTransitionState
        {
            BlackScreen = 0,
            FadingIn = 1,
            CameraImage = 2,
        }

        // YSKWAK-S
        private void Start()
        {
            if (sw == null)
            {
                dtCurrentTime = DateTime.Now;
                
                Directory.CreateDirectory(Application.persistentDataPath + "/Recording/RenderImage/");
                sw = new StreamWriter(Application.persistentDataPath + "/Recording/ARCore_Pose_"+
                    dtCurrentTime.Month + "_" +
                dtCurrentTime.Day + "_" +
                dtCurrentTime.Hour + "_" +
                dtCurrentTime.Minute + "_" +
                dtCurrentTime.Second + "_" +
                dtCurrentTime.Millisecond + ".txt"
                    );
            }

            //if( swIMUdata == null )
            //{
            //    Directory.CreateDirectory(Application.persistentDataPath + "/Recording/");
            //    swIMUdata = new StreamWriter(Application.persistentDataPath + "/Recording/IMUpose.txt");
            //}

            
            

        }
        // YSKWAK

        private void OnEnable()
        {
            if (BackgroundMaterial == null)
            {
                Debug.LogError("ArCameraBackground:: No material assigned.");
                return;
            }

            LifecycleManager.Instance.OnSessionSetEnabled += _OnSessionSetEnabled;

            m_Camera = GetComponent<Camera>();

            m_TransitionImageTexture = Resources.Load<Texture2D>("ViewInARIcon");
            BackgroundMaterial.SetTexture("_TransitionIconTex", m_TransitionImageTexture);

            EnableARBackgroundRendering();
        }

        private void OnDisable()
        {
            LifecycleManager.Instance.OnSessionSetEnabled -= _OnSessionSetEnabled;
            m_TransitionState = BackgroundTransitionState.BlackScreen;
            m_CurrentStateElapsed = 0.0f;

            m_Camera.ResetProjectionMatrix();

            DisableARBackgroundRendering();
        }

        private void OnPreRender()
        {
            m_UserInvertCullingValue = GL.invertCulling;
            var sessionComponent = LifecycleManager.Instance.SessionComponent;
            if (sessionComponent != null &&
                sessionComponent.DeviceCameraDirection == DeviceCameraDirection.FrontFacing)
            {
                GL.invertCulling = true;
            }
        }

        private void OnPostRender()
        {
            GL.invertCulling = m_UserInvertCullingValue;
        }

        private void Update()
        {
            m_CurrentStateElapsed += Time.deltaTime;
            _UpdateState();
            _UpdateShaderVariables();
        }

        private void _UpdateState()
        {
            if (!m_SessionEnabled && m_TransitionState != BackgroundTransitionState.BlackScreen)
            {
                m_TransitionState = BackgroundTransitionState.BlackScreen;
                m_CurrentStateElapsed = 0.0f;
            }
            else if (m_SessionEnabled &&
                     m_TransitionState == BackgroundTransitionState.BlackScreen &&
                     m_CurrentStateElapsed > k_BlackScreenDuration)
            {
                m_TransitionState = BackgroundTransitionState.FadingIn;
                m_CurrentStateElapsed = 0.0f;
            }
            else if (m_SessionEnabled &&
                     m_TransitionState == BackgroundTransitionState.FadingIn &&
                     m_CurrentStateElapsed > k_FadingInDuration)
            {
                m_TransitionState = BackgroundTransitionState.CameraImage;
                m_CurrentStateElapsed = 0.0f;
            }
        }

        private void _UpdateShaderVariables()
        {
            const string brightnessVar = "_Brightness";
            if (m_TransitionState == BackgroundTransitionState.BlackScreen)
            {
                BackgroundMaterial.SetFloat(brightnessVar, 0.0f);
            }
            else if (m_TransitionState == BackgroundTransitionState.FadingIn)
            {
                BackgroundMaterial.SetFloat(
                    brightnessVar,
                    _CosineLerp(m_CurrentStateElapsed, k_FadingInDuration));
            }
            else
            {
                BackgroundMaterial.SetFloat(brightnessVar, 1.0f);
            }

            // Set transform of the transition image texture, it may be visible or invisible based
            // on lerp value.
            const string transformVar = "_TransitionIconTexTransform";
            BackgroundMaterial.SetVector(transformVar, _TextureTransform());

            // Background texture should not be rendered when the session is disabled or
            // there is no camera image texture available.
            if (m_TransitionState == BackgroundTransitionState.BlackScreen ||
                Frame.CameraImage.Texture == null)
            {
                return;
            }

            const string mainTexVar = "_MainTex";
            const string topLeftRightVar = "_UvTopLeftRight";
            const string bottomLeftRightVar = "_UvBottomLeftRight";

            // YSKWAK-S
            bool bDev = true;
            bool bDev2 = true;

            if (bDev)
            {                
                //sw.WriteLine("Phase 1");
                //sw.Flush();

                try
                {
                    //sw.WriteLine("Phase 1A");
                    //sw.Flush();
                    using (image = Frame.CameraImage.AcquireCameraImageBytes())
                    {
                        if (!image.IsAvailable)
                        {
                            return;
                        }
                        //sw.WriteLine("Phase 1B");
                        //sw.Flush();

                        width = image.Width;
                        height = image.Height;
                        rowStride = image.YRowStride;

                        bufferSize = image.YRowStride * image.Height;
                        //bfferSizeUV = image.UVRowStride * image.Height;

                        if (bufferSize != s_ImageBufferSize || s_ImageBuffer.Length == 0)
                        {
                            s_ImageBufferSize = bufferSize;
                            s_ImageBuffer = new byte[bufferSize];
                            //s_ImageBufferU = new byte[bufferSize];
                            //s_ImageBufferV = new byte[bufferSize];
                            BitmapBytes = new byte[bufferSize * 3 + 54];
                            bImageBuffer24 = new byte[width * height * 3];
                            rawTex = new Texture2D(width, height, TextureFormat.RGB24, false);
                        }

                        //sw.WriteLine("Phase 1C");
                        //sw.Flush();

                        // Move raw data into managed buffer.
                        System.Runtime.InteropServices.Marshal.Copy(image.Y, s_ImageBuffer, 0, bufferSize);
                        //System.Runtime.InteropServices.Marshal.Copy(image.U, s_ImageBufferU, 0, bfferSizeUV);
                        //System.Runtime.InteropServices.Marshal.Copy(image.V, s_ImageBufferV, 0, bfferSizeUV);

                        //sw.WriteLine("Phase 1D");
                        //sw.Flush();

                        //if ( BitmapBytes == null)
                        //BitmapBytes = new byte[bufferSize * 3 + 54];
                        BitmapBytes[0] = 0x42;
                        BitmapBytes[1] = 0x4d;

                        //sw.WriteLine("Phase 1E");
                        //sw.Flush();

                        // 6~8 Application Specific : normally, set zero
                        Array.Copy(BitConverter.GetBytes(0), 0, BitmapBytes, 6, 2);

                        // 8~10 Application Specific : normally, set zero
                        Array.Copy(BitConverter.GetBytes(0), 0, BitmapBytes, 8, 2);

                        // 10~14 Offset where the pixel array can be found - 24bit bitmap data always starts at 54 offset.
                        Array.Copy(BitConverter.GetBytes(54), 0, BitmapBytes, 10, 4);

                        // DIB Header
                        // 14~18 Number of bytes in the DIB header. 40 bytes constant.
                        Array.Copy(BitConverter.GetBytes(40), 0, BitmapBytes, 14, 4);

                        // 18~22 Width of the bitmap.
                        Array.Copy(BitConverter.GetBytes(width), 0, BitmapBytes, 18, 4);

                        // 22~26 Height of the bitmap.
                        // 16:9
                        Array.Copy(BitConverter.GetBytes(height), 0, BitmapBytes, 22, 4);

                        // 26~28 Number of color planes being used
                        Array.Copy(BitConverter.GetBytes(0), 0, BitmapBytes, 26, 2);

                        // 28~30 Number of bits. If you don't know the pixel format, trying to calculate it with the quality of the video/image source.
                        //if (image.PixelFormat == System.Drawing.Imaging.PixelFormat.Format24bppRgb)
                        //{
                        //    Array.Copy(BitConverter.GetBytes(24), 0, BitmapBytes, 28, 2);
                        //
                        //}

                        // Image
                        Array.Copy(BitConverter.GetBytes(24), 0, BitmapBytes, 28, 2);

                        // 30~34 BI_RGB no pixel array compression used : most of the time, just set zero if it is raw data.
                        Array.Copy(BitConverter.GetBytes(0), 0, BitmapBytes, 30, 4);

                        // 34~38 Size of the raw bitmap data ( including padding )
                        Array.Copy(BitConverter.GetBytes(bufferSize), 0, BitmapBytes, 34, 4);

                        // 38~46 Print resolution of the image, 72 DPI x 39.3701 inches per meter yields
                        //if (image.PixelFormat == System.Drawing.Imaging.PixelFormat.Format24bppRgb)
                        {
                            Array.Copy(BitConverter.GetBytes(0), 0, BitmapBytes, 38, 4);
                            Array.Copy(BitConverter.GetBytes(0), 0, BitmapBytes, 42, 4);
                        }

                        // 46~50 Number of colors in the palette
                        Array.Copy(BitConverter.GetBytes(0), 0, BitmapBytes, 46, 4);

                        // 50~54 means all colors are important
                        Array.Copy(BitConverter.GetBytes(0), 0, BitmapBytes, 50, 4);

                        //sw.WriteLine("Phase 1F");
                        //sw.Flush();

                        //if (bImageBuffer24 == null)
                        //bImageBuffer24 = new byte[width * height * 3];
                        //byte[] bImageBuffer32 = new byte[width * height * 4];

                        // 8 bits -> 24bits
                        for (int y = 0; y < height ; y++)
                        {
                            for (int x = 0; x < width; x++)
                            {
                                bImageBuffer24[((height - y - 1) * width + x) * 3 + 0] = s_ImageBuffer[y * width + x];
                                bImageBuffer24[((height - y - 1) * width + x) * 3 + 1] = s_ImageBuffer[y * width + x];
                                bImageBuffer24[((height - y - 1) * width + x) * 3 + 2] = s_ImageBuffer[y * width + x];

                                //bImageBuffer32[((height - y - 1) * width + x) * 4 + 0] = s_ImageBuffer[y * width + x];
                                //bImageBuffer32[((height - y - 1) * width + x) * 4 + 1] = s_ImageBuffer[y * width + x];
                                //bImageBuffer32[((height - y - 1) * width + x) * 4 + 2] = s_ImageBuffer[y * width + x];
                                //bImageBuffer32[((height - y - 1) * width + x) * 4 + 3] = 1;
                            }
                        }

                        //sw.WriteLine("Phase 1G");
                        //sw.Flush();

                        Array.Copy(bImageBuffer24 as Array, 0, BitmapBytes, 54, width * height * 3);

                        //sw.WriteLine("Phase 1H");
                        //sw.Flush();

                        //if (siRawImageCount == 0)
                        //rawTex = new Texture2D(width, height, TextureFormat.RGB24, false);

                        //sw.WriteLine("Phase 1I");
                        //sw.Flush();

                        rawTex.LoadRawTextureData(bImageBuffer24);
                        rawTex.Apply();

                        //sw.WriteLine("Phase 1J");
                        //sw.Flush();

                        bytesJPG = rawTex.EncodeToJPG();

                        //sw.WriteLine("Phase 1K");
                        //sw.Flush();

                        //System.IO.StreamWriter sw = new System.IO.StreamWriter();
                        System.IO.Directory.CreateDirectory(Application.persistentDataPath + "/Recording/RawImage/");
                        File.WriteAllBytes(Application.persistentDataPath + "/Recording/RawImage/" + siRawImageCount + ".jpg", bytesJPG);

                        //sw.WriteLine("Phase 1L");
                        //sw.Flush();

                        //System.IO.BinaryWriter bw = new System.IO.BinaryWriter(System.IO.File.Open(Application.persistentDataPath + "/Recording/RawImage/" + siRawImageCount + ".bmp", System.IO.FileMode.Create));
                        //bw.Write(BitmapBytes);
                        //bw.Flush();
                        //bw.Close();

                        //bytesJPG.Initialize();
                        //delete bytesJPG;

                        //image.Dispose();
                        //sw.WriteLine("Phase 1M");
                    }
                    
                    siRawImageCount++;

                    if (sw == null)
                    {
                        Directory.CreateDirectory(Application.persistentDataPath + "/Recording/RenderImage/");
                        sw = new StreamWriter(Application.persistentDataPath + "/Recording/ARCorePose.txt");
                    }
                    //sw.WriteLine("Phase 2");
                    //sw.Flush();
                }
                catch(System.Exception e)
                {
                    if (swLog == null)
                        swLog = new StreamWriter(Application.persistentDataPath + "/error1.txt");
                    if (rawTex != null)
                    {
                        byte[] bImageBuffer32 = new byte[640 * 480 * 4];
                        swLog.WriteLine(rawTex.format);
                        swLog.WriteLine(rawTex.width * rawTex.height * 4);
                        swLog.WriteLine(bImageBuffer32.Length);
                        swLog.WriteLine();
                    }
                    swLog.WriteLine(e);
                    swLog.Flush();
                }
            }
            // YSKWAK

            BackgroundMaterial.SetTexture(mainTexVar, Frame.CameraImage.Texture);

            var uvQuad = Frame.CameraImage.TextureDisplayUvs;
            BackgroundMaterial.SetVector(
                topLeftRightVar,
                new Vector4(
                    uvQuad.TopLeft.x, uvQuad.TopLeft.y, uvQuad.TopRight.x, uvQuad.TopRight.y));
            BackgroundMaterial.SetVector(
                bottomLeftRightVar,
                new Vector4(uvQuad.BottomLeft.x, uvQuad.BottomLeft.y, uvQuad.BottomRight.x,
                    uvQuad.BottomRight.y));

            m_Camera.projectionMatrix = Frame.CameraImage.GetCameraProjectionMatrix(
                m_Camera.nearClipPlane, m_Camera.farClipPlane);


            // YSKWAK-S
            // Rendering sequence

            if (bDev2)
            {
                //if (sw == null)
                //{
                //    Directory.CreateDirectory(Application.persistentDataPath + "/Recording/RenderImage/");
                //    sw = new StreamWriter(Application.persistentDataPath + "/Recording/ARCorePose.txt");
                //}
                //sw.WriteLine("Phase 3");
                //sw.Flush();
                //sw.WriteLine("Phase 3AAAA");

                try
                {
                    if( Cam == null)
                        Cam = GetComponent<Camera>();
                    //sw.WriteLine("Phase 3A");

                    if (Cam.targetTexture == null)
                    {
                        if ( rtex == null )
                            rtex = new RenderTexture(640, 320, 32, RenderTextureFormat.ARGB32);
                        Cam.targetTexture = rtex;
                    }                    

                    currentRT = RenderTexture.active;
                    RenderTexture.active = Cam.targetTexture;

                    //sw.WriteLine("Phase 3B");

                    Cam.Render();

                    if(siRenderImageCount==0)
                        Image = new Texture2D(Cam.targetTexture.width, Cam.targetTexture.height);
                    Image.ReadPixels(new Rect(0, 0, Cam.targetTexture.width, Cam.targetTexture.height), 0, 0);
                    Image.Apply();

                    //Texture2D tex = (Texture2D)Frame.CameraImage.Texture;

                    //sw.WriteLine("Phase 3C");

                    //byte[] bytes = tex.GetRawTextureData();
                    bytes = Image.EncodeToJPG(100);
                    //Color[] pix = Frame.CameraImage.Texture..GetPixels();
                    //byte[] bytes = Image.EncodeToJPG(100);
                    //byte[] bytes2 = Image.GetRawTextureData();
                    //Image.Apply();

                    RenderTexture.active = currentRT;

                    Directory.CreateDirectory(Application.persistentDataPath + "/Recording/RenderImage/");
                    File.WriteAllBytes(Application.persistentDataPath + "/Recording/RenderImage/" + siRenderImageCount + ".jpg", bytes);

                    //sw.WriteLine("Phase 3D");
                    //sw.Write(siRenderImageCount + "\t");

                    dtCurrentTime = DateTime.Now;

                    sw.Write(
                        dtCurrentTime.Month + "\t" +
                    dtCurrentTime.Day + "\t" +
                    dtCurrentTime.Hour + "\t" +
                    dtCurrentTime.Minute + "\t" +
                    dtCurrentTime.Second + "\t" +
                    dtCurrentTime.Millisecond + "\t");    

                    sw.Write(Cam.transform.rotation.x + "\t");
                    sw.Write(Cam.transform.rotation.y + "\t");
                    sw.Write(Cam.transform.rotation.z + "\t");
                    sw.Write(Cam.transform.rotation.w + "\t");
                    sw.Write(Cam.transform.position.x + "\t");
                    sw.Write(Cam.transform.position.y + "\t");
                    sw.WriteLine(Cam.transform.position.z);
                    sw.Flush();

                    //swIMUdata.Write(imu.qCurrentE2Q.x);
                    //swIMUdata.Write(imu.qCurrentE2Q.y);
                    //swIMUdata.Write(imu.qCurrentE2Q.z);
                    //swIMUdata.WriteLine(imu.qCurrentE2Q.w);
                    //swIMUdata.Flush();

                    //sw.WriteLine("Phase 3E");

                    Cam.targetTexture = null;

                    //image.Release();

                    siRenderImageCount++;
                    if (sw == null)
                    {
                        Directory.CreateDirectory(Application.persistentDataPath + "/Recording/RenderImage/");
                        sw = new StreamWriter(Application.persistentDataPath + "/Recording/ARCorePose.txt");
                    }
                    //sw.WriteLine("Phase 4");
                    //sw.Flush();
                }
                catch(System.Exception e)
                {
                    if (swLog2 == null)
                        swLog2 = new StreamWriter(Application.persistentDataPath + "/error2.txt");
                    swLog2.WriteLine(e);
                    swLog2.Flush();
                }

            }
            System.GC.Collect();
            // YSKWAK

        }

        private void _OnSessionSetEnabled(bool sessionEnabled)
        {
            m_SessionEnabled = sessionEnabled;
            if (!m_SessionEnabled)
            {
                _UpdateState();
                _UpdateShaderVariables();
            }
        }

        private float _CosineLerp(float elapsed, float duration)
        {
            float clampedElapsed = Mathf.Clamp(elapsed, 0.0f, duration);
            return Mathf.Cos(((clampedElapsed / duration) - 1) * (Mathf.PI / 2));
        }

        /// <summary>
        /// Textures transform used in background shader to get texture uv coordinates based on
        /// screen uv.
        /// The transformation follows these equations:
        /// textureUv.x = transform[0] * screenUv.x + transform[1],
        /// textureUv.y = transform[2] * screenUv.y + transform[3].
        /// </summary>
        /// <returns>The transform.</returns>
        private Vector4 _TextureTransform()
        {
            float transitionWidthTransform = (m_TransitionImageTexture.width - Screen.width) /
                (2.0f * m_TransitionImageTexture.width);
            float transitionHeightTransform = (m_TransitionImageTexture.height - Screen.height) /
                (2.0f * m_TransitionImageTexture.height);
            return new Vector4(
                (float)Screen.width / m_TransitionImageTexture.width,
                transitionWidthTransform,
                (float)Screen.height / m_TransitionImageTexture.height,
                transitionHeightTransform);
        }

        private void EnableARBackgroundRendering()
        {
            if (BackgroundMaterial == null || m_Camera == null)
            {
                return;
            }

            m_CameraClearFlags = m_Camera.clearFlags;
            m_Camera.clearFlags = CameraClearFlags.Depth;

            m_CommandBuffer = new CommandBuffer();

            m_CommandBuffer.Blit(BackgroundMaterial.mainTexture,
                BuiltinRenderTextureType.CameraTarget, BackgroundMaterial);
                       
            m_Camera.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, m_CommandBuffer);
            m_Camera.AddCommandBuffer(CameraEvent.BeforeGBuffer, m_CommandBuffer);
        }

        private void DisableARBackgroundRendering()
        {
            if (m_CommandBuffer == null || m_Camera == null)
            {
                return;
            }

            m_Camera.clearFlags = m_CameraClearFlags;

            m_Camera.RemoveCommandBuffer(CameraEvent.BeforeForwardOpaque, m_CommandBuffer);
            m_Camera.RemoveCommandBuffer(CameraEvent.BeforeGBuffer, m_CommandBuffer);
        }
    }
}
