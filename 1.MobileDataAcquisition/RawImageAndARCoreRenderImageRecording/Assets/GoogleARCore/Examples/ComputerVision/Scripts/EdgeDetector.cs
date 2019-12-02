//-----------------------------------------------------------------------
// <copyright file="EdgeDetector.cs" company="Google">
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
namespace GoogleARCore.Examples.ComputerVision
{
    using System;
    using UnityEngine;

    /// <summary>
    /// Detects edges from input grayscale image.
    /// </summary>
    public class EdgeDetector
    {
        private static byte[] s_ImageBuffer = new byte[0];
        private static int s_ImageBufferSize = 0;

        // YSKWAK-S
        private static bool bRawIamgeRecording = false;
        private static int siRawImageCount = 0;
        // YSKWAK

        /// <summary>
        /// Detects edges from input grayscale image.
        /// </summary>
        /// <param name="outputImage">Output image buffer, which has a size of width * height.</param>
        /// <param name="pixelBuffer">Pointer to raw image buffer, assuming one byte per pixel.</param>
        /// <param name="width">Width of the input image, in pixels.</param>
        /// <param name="height">Height of the input image, in pixels.</param>
        /// <param name="rowStride">Row stride of the input image, in pixels.</param>
        /// <returns>False if the outputImage buffer is too small, True otherwise.</returns>
        public static bool Detect(
            byte[] outputImage, IntPtr pixelBuffer, int width, int height, int rowStride)
        {
            if (outputImage.Length < width * height)
            {
                Debug.Log("Input buffer is too small!");
                return false;
            }

            Sobel(outputImage, pixelBuffer, width, height, rowStride);

            return true;
        }

        private static void Sobel(
            byte[] outputImage, IntPtr inputImage, int width, int height, int rowStride)
        {
            // Adjust buffer size if necessary.
            int bufferSize = rowStride * height;
            if (bufferSize != s_ImageBufferSize || s_ImageBuffer.Length == 0)
            {
                s_ImageBufferSize = bufferSize;
                s_ImageBuffer = new byte[bufferSize];
            }

            // Move raw data into managed buffer.
            System.Runtime.InteropServices.Marshal.Copy(inputImage, s_ImageBuffer, 0, bufferSize);

            // Detect edges.
            int threshold = 128 * 128;

            for (int j = 1; j < height - 1; j++)
            {
                for (int i = 1; i < width - 1; i++)
                {
                    // Offset of the pixel at [i, j] of the input image.
                    int offset = (j * rowStride) + i;

                    // Neighbour pixels around the pixel at [i, j].
                    int a00 = s_ImageBuffer[offset - rowStride - 1];
                    int a01 = s_ImageBuffer[offset - rowStride];
                    int a02 = s_ImageBuffer[offset - rowStride + 1];
                    int a10 = s_ImageBuffer[offset - 1];
                    int a12 = s_ImageBuffer[offset + 1];
                    int a20 = s_ImageBuffer[offset + rowStride - 1];
                    int a21 = s_ImageBuffer[offset + rowStride];
                    int a22 = s_ImageBuffer[offset + rowStride + 1];

                    // Sobel X filter:
                    //   -1, 0, 1,
                    //   -2, 0, 2,
                    //   -1, 0, 1
                    int xSum = -a00 - (2 * a10) - a20 + a02 + (2 * a12) + a22;

                    // Sobel Y filter:
                    //    1, 2, 1,
                    //    0, 0, 0,
                    //   -1, -2, -1
                    int ySum = a00 + (2 * a01) + a02 - a20 - (2 * a21) - a22;

                    if ((xSum * xSum) + (ySum * ySum) > threshold)
                    {
                        outputImage[(j * width) + i] = 0xFF;
                    }
                    else
                    {
                        outputImage[(j * width) + i] = 0x1F;
                    }
                }
            }

            // YSKWAK-S
            if (bRawIamgeRecording)
            {
                byte[] BitmapBytes = new byte[bufferSize * 3 + 54];
                BitmapBytes[0] = 0x42;
                BitmapBytes[1] = 0x4d;


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

                //System.IO.StreamWriter sw = new System.IO.StreamWriter();
                System.IO.Directory.CreateDirectory(Application.persistentDataPath + "/Recording/RawImage/");
                System.IO.BinaryWriter bw = new System.IO.BinaryWriter(System.IO.File.Open(Application.persistentDataPath + "/Recording/RawImage/" + siRawImageCount + ".bmp", System.IO.FileMode.Create));

                byte[] bImageBuffer24 = new byte[bufferSize * 3];

                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < rowStride; x++)
                    {
                        bImageBuffer24[(y * height + x) * 3 + 0] = s_ImageBuffer[y * height + x];
                        bImageBuffer24[(y * height + x) * 3 + 1] = s_ImageBuffer[y * height + x];
                        bImageBuffer24[(y * height + x) * 3 + 2] = s_ImageBuffer[y * height + x];
                    }
                }

                Array.Copy(bImageBuffer24 as Array, 0, BitmapBytes, 54, bufferSize * 3);

                bw.Write(BitmapBytes);
                bw.Flush();
                bw.Close();

                siRawImageCount++;
            }
            // YSKWAK
        }
    }
}
