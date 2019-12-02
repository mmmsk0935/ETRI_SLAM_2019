/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

void Viewer::LinkWithTracking(Tracking *tTrack)
{
	tTracking = tTrack;

}

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=15;
	fps = 10;
    mT = 1e3/fps;
	
    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
	// YSKWAK-S
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,false);
	//pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
	// YSKWAK
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;

	//YSKWAK-S
	static int iRawImageFrameCount = 1;
	//tempTracking = new Tracking;
	//YSKWAK

	while (1)
	{
		// YSKWAK-S
		//ifsIMU >> dIMUqX >> dIMUqY >> dIMUqZ >> dIMUqW;
		//iImageGrabCount++;

		////cout << "Current quaterion of IMU" << endl;
		////cout << "Grab Image Count = " << iImageGrabCount << endl;
		////cout << dIMUqX << "\t" << dIMUqY << "\t" << dIMUqZ << "\t" << dIMUqW << endl << endl;

		//float xx = dIMUqX * dIMUqX;
		//float xy = dIMUqX * dIMUqY;
		//float xz = dIMUqX * dIMUqZ;
		//float xw = dIMUqX * dIMUqW;

		//float yy = dIMUqY * dIMUqY;
		//float yz = dIMUqY * dIMUqZ;
		//float yw = dIMUqY * dIMUqW;

		//float zz = dIMUqZ * dIMUqZ;
		//float zw = dIMUqZ * dIMUqW;

		//float mat[16];

		//mat[0] = 1 - 2 * (yy + zz);
		//mat[1] = 2 * (xy - zw);
		//mat[2] = 2 * (xz + yw);

		//mat[4] = 2 * (xy + zw);
		//mat[5] = 1 - 2 * (xx + zz);
		//mat[6] = 2 * (yz - xw);

		//mat[8] = 2 * (xz - yw);
		//mat[9] = 2 * (yz + xw);
		//mat[10] = 1 - 2 * (xx + yy);

		//mat[3] = mat[7] = mat[11] = mat[12] = mat[13] = mat[14] = 0;
		//mat[15] = 1;
		// YSKWAK

		// Transpose of Transformation for opengl
		// Rotation only

		//cout << "Origin Transformation Matrix" << endl;
		//cout << Tcw << endl << endl;

		//cout << "Current Transformation Matrix of IMU" << endl;
		//cout << mat[0] << "\t" << mat[4] << "\t" << mat[8] << "\t" << mat[12] << endl;
		//cout << mat[1] << "\t" << mat[5] << "\t" << mat[9] << "\t" << mat[13] << endl;
		//cout << mat[2] << "\t" << mat[6] << "\t" << mat[10] << "\t" << mat[14] << endl;
		//cout << mat[3] << "\t" << mat[7] << "\t" << mat[11] << "\t" << mat[15] << endl << endl;

		/*cv::Mat initialIMUrcw = cv::Mat::eye(3, 3, CV_32F);
		initialIMUrcw.at<float>(0, 0) = mat[0];
		initialIMUrcw.at<float>(0, 1) = mat[1];
		initialIMUrcw.at<float>(0, 2) = mat[2];

		initialIMUrcw.at<float>(1, 0) = mat[4];
		initialIMUrcw.at<float>(1, 1) = mat[5];
		initialIMUrcw.at<float>(1, 2) = mat[6];

		initialIMUrcw.at<float>(2, 0) = mat[7];
		initialIMUrcw.at<float>(2, 1) = mat[8];
		initialIMUrcw.at<float>(2, 2) = mat[9];*/

		/*initialIMUrcw.at<float>(0, 0) = 1;
		initialIMUrcw.at<float>(0, 1) = 0;
		initialIMUrcw.at<float>(0, 2) = 0;

		initialIMUrcw.at<float>(1, 0) = 0;
		initialIMUrcw.at<float>(1, 1) = cos(1.5708);
		initialIMUrcw.at<float>(1, 2) = -sin(1.5708);

		initialIMUrcw.at<float>(2, 0) = 0;
		initialIMUrcw.at<float>(2, 1) = sin(1.5708);
		initialIMUrcw.at<float>(2, 2) = cos(1.5708);*/


		//cout << "Transpose matrix of IMU transformation" << endl;
		//cout << initialIMUrcw << endl << endl;

		/*cv::Mat inverseInitialIMUrcw = initialIMUrcw.inv();

		float mat_gl[16];
		mat_gl[0] = inverseInitialIMUrcw.at<float>(0, 0);
		mat_gl[4] = inverseInitialIMUrcw.at<float>(0, 1);
		mat_gl[8] = inverseInitialIMUrcw.at<float>(0, 2);

		mat_gl[1] = inverseInitialIMUrcw.at<float>(1, 0);
		mat_gl[5] = inverseInitialIMUrcw.at<float>(1, 1);
		mat_gl[9] = inverseInitialIMUrcw.at<float>(1, 2);

		mat_gl[2] = inverseInitialIMUrcw.at<float>(2, 0);
		mat_gl[6] = inverseInitialIMUrcw.at<float>(2, 1);
		mat_gl[10] = inverseInitialIMUrcw.at<float>(2, 2);

		mat_gl[3] = mat_gl[7] = mat_gl[11] = mat_gl[12] = mat_gl[13] = mat_gl[14] = 0;
		mat_gl[15] = 1;*/

		//inverseInitialIMUrcw.mul()

		//cout << "Inverse matrix of IMU transformation" << endl;
		//cout << inverseInitialIMUrcw << endl << endl;

		//cv::normalize(inverseInitialIMUrcw, inverseInitialIMUrcw);

		//inverseInitialIMUrcw.n

		//cout << "Normalized Inverse matrix of IMU transformation" << endl;
		//cout << inverseInitialIMUrcw << endl << endl;

		//cv::Mat mulRotation = inverseInitialIMUrcw.mul(Rcw);
		//cv::Mat mulRotation = inverseInitialIMUrcw.mul(Rcw);
		//inverseInitialIMUrcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
		//initialIMUrcw.copyTo(Rcw.rowRange(0, 3).colRange(0, 3));

		//cout << "Result matrix of IMU transformation" << endl; 
		//cout << mulRotation << endl << endl;

		//cout << "Result Tcw" << endl;
		//cout << Tcw << endl << endl;


		//cv::Mat test = cv::Mat::eye(3, 3, CV_32F);

		//cout << "Test matrix of transformation" << endl;
		//cout << test << endl << endl;

		//test = test * 2;
		//cout << "Test Scaled matrix of transformation" << endl;
		//cout << test << endl << endl;

		////cv::normalize()
		//
		//cout << "Normalized Test Scaled matrix of transformation" << endl;
		//cout << test << endl << endl;


		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

		if (menuFollowCamera && bFollow)
		{
			s_cam.Follow(Twc);
		}
		else if (menuFollowCamera && !bFollow)
		{
			s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
			//s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
			s_cam.Follow(Twc);
			bFollow = true;
		}
		else if (!menuFollowCamera && bFollow)
		{
			bFollow = false;
		}

		if (menuLocalizationMode && !bLocalizationMode)
		{
			mpSystem->ActivateLocalizationMode();
			bLocalizationMode = true;
		}
		else if (!menuLocalizationMode && bLocalizationMode)
		{
			mpSystem->DeactivateLocalizationMode();
			bLocalizationMode = false;
		}

		d_cam.Activate(s_cam);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

		// YSKWAK-S

		// Axis vector drawing of world coordinate
		glLineWidth(3.0);
		glBegin(GL_LINES);

			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(10.0, 0.0, 0.0);

			glColor3f(0.0, 1.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 10.0, 0.0);

			glColor3f(0.0, 0.0, 1.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 10.0);
		glEnd();

		// Ground drawing of world coordinate
		// X-Z plane
		glColor3f(0.5, 0.5, 0.5);
		glLineWidth(1.0);
		glBegin(GL_LINES);
		for (float x = -100; x <= 100; x += 1)
		{
			glVertex3f(x, 0.0, -100.0);
			glVertex3f(x, 0.0, 100.0);

			glVertex3f(-100.0, 0.0, x);
			glVertex3f(100.0, 0.0, x);
		}
		glEnd();


		// Rotation matrix calculation between ORB pose and IMU(Smart phone) pose;
		float* f_mat = tTracking->mat_gl;
		float f_mat_inv[16];

		// IMU rotation log (float[])

		/*cout << endl << tTracking->iImageGrabCount << endl;
		cout << f_mat[0] << "\t" << f_mat[1] << "\t" << f_mat[2] << "\t" << f_mat[3] << endl;
		cout << f_mat[4] << "\t" << f_mat[5] << "\t" << f_mat[6] << "\t" << f_mat[7] << endl;
		cout << f_mat[8] << "\t" << f_mat[9] << "\t" << f_mat[10] << "\t" << f_mat[11] << endl;
		cout << f_mat[12] << "\t" << f_mat[13] << "\t" << f_mat[14] << "\t" << f_mat[15] << endl;*/

		cv::Mat mIMUrotation = cv::Mat(3, 3, CV_32F);
		cv::Mat mIMUrotationInv;

		mIMUrotation.at<float>(0, 0) = f_mat[0];
		mIMUrotation.at<float>(0, 1) = f_mat[1];
		mIMUrotation.at<float>(0, 2) = f_mat[2];

		mIMUrotation.at<float>(1, 0) = f_mat[4];
		mIMUrotation.at<float>(1, 1) = f_mat[5];
		mIMUrotation.at<float>(1, 2) = f_mat[6];

		mIMUrotation.at<float>(2, 0) = f_mat[8];
		mIMUrotation.at<float>(2, 1) = f_mat[9];
		mIMUrotation.at<float>(2, 2) = f_mat[10];

		// IMU orientation log (cv::Mat)

		//cout << endl << mTemp << endl << endl;
		//cout << mTemp.at<float>(0, 0) << "\t" << mTemp.at<float>(0, 1) << "\t" << mTemp.at<float>(0, 2) << endl;
		//cout << mTemp.at<float>(1, 0) << "\t" << mTemp.at<float>(1, 1) << "\t" << mTemp.at<float>(1, 2) << endl;
		//cout << mTemp.at<float>(2, 0) << "\t" << mTemp.at<float>(2, 1) << "\t" << mTemp.at<float>(2, 2) << endl;

		// IMU inverse rotation

		cv::invert(mIMUrotation, mIMUrotationInv);

		// IMU inverse rotation log

		//cout << endl << mTempInv << endl << endl;
		//cout << mTempInv.at<float>(0, 0) << "\t" << mTempInv.at<float>(0, 1) << "\t" << mTempInv.at<float>(0, 2) << endl;
		//cout << mTempInv.at<float>(1, 0) << "\t" << mTempInv.at<float>(1, 1) << "\t" << mTempInv.at<float>(1, 2) << endl;
		//cout << mTempInv.at<float>(2, 0) << "\t" << mTempInv.at<float>(2, 1) << "\t" << mTempInv.at<float>(2, 2) << endl;

		f_mat_inv[0] = mIMUrotationInv.at<float>(0, 0);
		f_mat_inv[1] = mIMUrotationInv.at<float>(0, 1);
		f_mat_inv[2] = mIMUrotationInv.at<float>(0, 2);

		f_mat_inv[4] = mIMUrotationInv.at<float>(1, 0);
		f_mat_inv[5] = mIMUrotationInv.at<float>(1, 1);
		f_mat_inv[6] = mIMUrotationInv.at<float>(1, 2);

		f_mat_inv[8] = mIMUrotationInv.at<float>(2, 0);
		f_mat_inv[9] = mIMUrotationInv.at<float>(2, 1);
		f_mat_inv[10] = mIMUrotationInv.at<float>(2, 2);

		f_mat_inv[3] = f_mat_inv[7] = f_mat_inv[11] = f_mat_inv[12] = f_mat_inv[13] = f_mat_inv[14] = 0;
		f_mat_inv[15] = 1;

		// World coordinate
		// X-Z plane : ground
		//  Z    Y(down)
		//  |  /
		//  | /
		//  |/
		//  -------- X

		// IMU coordinate
		// X-Y plane : ground
		//    Y    
		//    |  
		//    | 
		//    |
		//    -------- X
		//   /
		//  /
		// Z (Up)


		glPushMatrix();
		// World coordiate to IMU coordinate
		// ☞ x-axis +90 degree rotation in world coordinate
		
		glRotatef(90, 1.0, 0.0, 0.0);
		
		// Draw axis vector of IMU
		/*glPushMatrix();
		glLineWidth(10.0);
		glBegin(GL_LINES);
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(5.0, 0.0, 0.0);

			glColor3f(0.0, 1.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 5.0, 0.0);

			glColor3f(0.0, 0.0, 1.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 5.0);
		glEnd();
		glPopMatrix();*/

		// IMU to ORB rotation calculation
		cv::Mat mORBrotation = cv::Mat(3, 3, CV_32F);

		mORBrotation.at<float>(0, 0) = Twc.m[0];
		mORBrotation.at<float>(0, 1) = Twc.m[1];
		mORBrotation.at<float>(0, 2) = Twc.m[2];

		mORBrotation.at<float>(1, 0) = Twc.m[4];
		mORBrotation.at<float>(1, 1) = Twc.m[5];
		mORBrotation.at<float>(1, 2) = Twc.m[6];

		mORBrotation.at<float>(2, 0) = Twc.m[8];
		mORBrotation.at<float>(2, 1) = Twc.m[9];
		mORBrotation.at<float>(2, 2) = Twc.m[10];

		cv::Mat mDiff = mORBrotation * mIMUrotationInv;
		double bAngleDistance = acos(((double)(mDiff.at<float>(0, 0) + mDiff.at<float>(1, 1) + mDiff.at<float>(2, 2) - 1.0) / 2.0) * 180.0 / 3.1459);

		// IMU to ORB rotation log
		//cout << mORBrotation << endl;

		float f_diff[16];

		f_diff[0] = mDiff.at<float>(0, 0);
		f_diff[1] = mDiff.at<float>(0, 1);
		f_diff[2] = mDiff.at<float>(0, 2);

		f_diff[4] = mDiff.at<float>(1, 0);
		f_diff[5] = mDiff.at<float>(1, 1);
		f_diff[6] = mDiff.at<float>(1, 2);

		f_diff[8] = mDiff.at<float>(2, 0);
		f_diff[9] = mDiff.at<float>(2, 1);
		f_diff[10] = mDiff.at<float>(2, 2);

		f_diff[3] = f_diff[7] = f_diff[11] = f_diff[12] = f_diff[13] = f_diff[14] = 0;
		f_diff[15] = 1;

		// Angle distance of IMU to ORB rotation
		/*cout << "Angle distance : " << endl
			<< "tr(R) : " << (double)(mDiff.at<float>(0, 0) + mDiff.at<float>(1, 1) + mDiff.at<float>(2, 2)) << endl
			<< "tr(R)-1 : " << (double)(mDiff.at<float>(0, 0) + mDiff.at<float>(1, 1) + mDiff.at<float>(2, 2) - 1.0) << endl
			<< "(tr(R)-1)/2 : " << ((double)(mDiff.at<float>(0, 0) + mDiff.at<float>(1, 1) + mDiff.at<float>(2, 2) - 1.0) / 2.0) << endl
			<< "arccos( (tr(R)-1)/2 ) : " << acos((double)(mDiff.at<float>(0, 0) + mDiff.at<float>(1, 1) + mDiff.at<float>(2, 2) - 1.0) / 2.0) * 180.0 / 3.1459 << endl
			<< bAngleDistance << endl
			<< "TEST " << acos(1.0)<< endl;*/

		// Euler

		float sy = sqrt(mDiff.at<float>(0, 0) * mDiff.at<float>(0, 0));
		bool singular = sy < 1e-6;

		float dx, dy, dz;
		if (!singular)
		{
			dx = atan2(mDiff.at<float>(2, 1), mDiff.at<float>(2, 2) );
			dy = atan2(mDiff.at<float>(2, 0), sy);
			dz = atan2(mDiff.at<float>(1, 0), mDiff.at<float>(0, 0));
		}
		else
		{
			dx = atan2(-mDiff.at<float>(1, 2), mDiff.at<float>(1, 1));
			dy = atan2(-mDiff.at<float>(2, 0), sy);
			dz = 0;
		}
		
		cout << "Euler angle, singular : " << singular << endl;
		cout << dx * 180 / 3.1459 << "\t" << dy * 180 / 3.1459 << "\t" << dz * 180 / 3.1459 << endl << endl;
		
		// IMU rotation drawing

		// Rotation test
		/*float fTemp[16];
		fTemp[0] = cos(45 * 3.1459 / 180);
		fTemp[4] = -sin(45 * 3.1459 / 180);
		fTemp[8] = 0;

		fTemp[1] = sin(45 * 3.1459 / 180);
		fTemp[5] = cos(45 * 3.1459 / 180);
		fTemp[9] = 0;

		fTemp[2] = 0;
		fTemp[6] = 0;
		fTemp[10] = 1;

		fTemp[3] = fTemp[7] = fTemp[11] = fTemp[12] = fTemp[13] = fTemp[14] = 0;
		fTemp[15] = 1;
		glMultMatrixf(fTemp); */

		glMultMatrixf(f_mat); // IMU	
		//glRotatef(-90dz, 0.0, 0.0, 1.0);
		//glMultMatrixf(f_diff); // IMU to ORB

		// Axis view of IMU

		// Local coordinate of smart phone
		// Y : Up
		// X : Right
		// Z : Normal direction of display panel
		// Y:X ratio 
		// 18:9 (Samsung Note 9)

		//    Y    
		//    |  
		//    |  
		//  +-|-+ 
		//  | | |
		//  | --+------- X
		//  |/  |
		//  /---+
		// /
		// Z (Up)

		// Draw
		glColor3f(1.0, 1.0, 0.0);
		glPushMatrix();
		glLineWidth(5.0);
		glBegin(GL_LINES);
			glColor3f(0.7, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.1, 0.0, 0.0);

			glColor3f(0.0, 0.7, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 0.2, 0.0);

			glColor3f(0.0, 0.0, 0.7);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 0.02);
		glEnd();

		const float& w = 0.1;
		const float h = w * 1.33;
		const float z = -w * 0.6; // Camera : oposite direction

		glLineWidth(1.0);
		
		glColor3f(1.0f, 0.0f, 0.0f);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(w, h, z);

		glVertex3f(0, 0, 0);
		glVertex3f(w, -h, z);

		glVertex3f(0, 0, 0);
		glVertex3f(-w, -h, z);

		glVertex3f(0, 0, 0);
		glVertex3f(-w, h, z);

		glVertex3f(w, h, z);
		glVertex3f(w, -h, z);

		glVertex3f(-w, h, z);
		glVertex3f(-w, -h, z);

		glVertex3f(-w, h, z);
		glVertex3f(w, h, z);

		glVertex3f(-w, -h, z);
		glVertex3f(w, -h, z);
		glEnd();

		glPopMatrix();

		//glMultMatrixf(mat_gl);
		//glRotatef(90, 1.0, 0.0, 0.0);
		//glMultMatrixf(f_mat_inv);		
		//glMultMatrixf(f_mat_inv);

		//glRotatef(-90, 1.0, 0.0, 0.0);
		//glMultMatrixf(f_mat_inv);

		// Difference between f_mat to Twc
		//cosh( trace( Twc*f_mat_inv)/ 2)

		//f_diff

		//glPopMatrix();

		// YSKWAK

        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();

		// YSKWAK-S
		/*std::string filename = "D:/Temp2/Frame/";
		filename = filename + std::to_string(iRawImageFrameCount);
		filename = filename + +".png";*/
		
		
		//cout << filename << endl;;
		// YSKWAK

        cv::imshow("ORB-SLAM2: Current Frame",im);

		// YSKWAK-S
		/*cv::imwrite(filename,im);		
		filename = "D:/Temp2/MapView/"+ std::to_string(iRawImageFrameCount);
		d_cam.SaveOnRender(filename);
		iRawImageFrameCount++;*/
		// YSKWAK

        cv::waitKey(mT);
		//cv::waitKey(100);
		

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;

		// YSKWAK-S
		//glPopMatrix();
		// YSKWAK
    }


    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
