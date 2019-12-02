// Quaternion.h: interface for the Quaternion class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_QUATERNION_H__83C30DA4_0090_11D5_9C75_0000B4A67F6E__INCLUDED_)
#define AFX_QUATERNION_H__83C30DA4_0090_11D5_9C75_0000B4A67F6E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <afxwin.h>
#include <math.h>
#include <gl\GL.h>
#include <gl\GLU.h>
#include "Point3Dt.h"

#ifndef M_PI
#define M_PI 3.1415926535
#endif

class  Quaternion  
{
public:
	Quaternion();
	Quaternion(double, double, double, double);
	Quaternion(const Quaternion &);						// copy constructor
	Quaternion(const double matrix[4][4]);
	virtual ~Quaternion();
	void normalize();
	double& operator [] (int);							// indexing
	const double operator [] (int) const;				// indexing
	const Quaternion& operator = (const Quaternion&);	// assignment
	const Quaternion& operator = (const double[]);		// assignment
	const Quaternion& operator *= (const Quaternion &);
	void ToGLRotate(GLdouble []);
	void ToMatrix(double m[][4]);
	Quaternion inverse();
	void from_axis(double x, double y, double z, double angle_in_degree);
	double cells[4];
};

// prototypes for nonmember functions that operate on Quaternion

// multiply operation
Quaternion operator * (const Quaternion & lhs, const Quaternion & rhs);
Point3D<double> operator * (const Quaternion & lhs, const Point3D<double> &rhs);

// external variable
extern Quaternion   zeroQ;
extern Quaternion   rotXQ;
extern Quaternion   rotYQ;
extern Quaternion   rotZQ;

#endif // !defined(AFX_QUATERNION_H__83C30DA4_0090_11D5_9C75_0000B4A67F6E__INCLUDED_)
