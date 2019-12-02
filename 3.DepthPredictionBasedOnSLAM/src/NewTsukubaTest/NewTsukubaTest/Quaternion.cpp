// Quaternion.cpp: implementation of the Quaternion class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Quaternion.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

#ifndef M_PI
#define M_PI	3.14159265358979323846
#endif

// Macro functions
#define D2R(degree)	(degree/180.0*M_PI)
#define	R2D(radian) (radian/M_PI*180.0)

Quaternion   zeroQ(1.0, 0.0, 0.0, 0.0);
Quaternion   rotXQ(0.995, 0.09987, 0.0, 0.0);
Quaternion   rotYQ(0.995, 0.0, 0.09987, 0.0);
Quaternion   rotZQ(0.995, 0.0, 0.0, 0.09987);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Quaternion::Quaternion()
{
	cells[0] = 1.0;
	cells[1] = 0.0;
	cells[2] = 0.0;
	cells[3] = 0.0;
}

Quaternion::~Quaternion()
{

}

Quaternion::Quaternion(double w, double x, double y, double z)
{
	cells[0] = w;
	cells[1] = x;
	cells[2] = y;
	cells[3] = z;
}

Quaternion::Quaternion(const Quaternion &q)
{
	cells[0] = q[0];
	cells[1] = q[1];
	cells[2] = q[2];
	cells[3] = q[3];
}


Quaternion::Quaternion(const double matrix[4][4])
{
	double tr, s, q[4];
	int i, j, k;
	int nxt[3] = {1, 2, 0};

	tr = matrix[0][0] + matrix[1][1] + matrix[2][2];	// trace

	// check the diagonal
	if( tr > 0.0 )
	{
		s = (float)sqrt( tr + 1.0 );
		cells[0] = s / 2.0;
		s = float(0.5 / s);
		cells[1] = (matrix[1][2] - matrix[2][1]) * s;	// x
		cells[2] = (matrix[2][0] - matrix[0][2]) * s;	// y
		cells[3] = (matrix[0][1] - matrix[1][0]) * s;	// z
	}
	else
	{
		//	diagonal is negative
		i = 0;
		if( matrix[1][1] > matrix[0][0] )
			i = 1;
		if( matrix[2][2] > matrix[i][i] )
			i = 2;
		j = nxt[i];
		k = nxt[j];

		s = (float)sqrt( ( matrix[i][i] - (matrix[j][j] + matrix[k][k]) ) + 1.0);
		q[i] = float(s * 0.5);
		if( s != 0.0 )
			s = float(0.5 / s);
		q[3] = (matrix[j][k] - matrix[k][j]) * s;
		q[j] = (matrix[i][j] - matrix[j][i]) * s;
		q[k] = (matrix[i][k] - matrix[k][i]) * s;

		cells[0] = q[3];	// w
		cells[1] = q[0];	// x
		cells[2] = q[1];	// y
		cells[3] = q[2];	// z
	}
}

void Quaternion::normalize()
{
	double norm;

	norm = sqrt(cells[0]*cells[0]+cells[1]*cells[1]+cells[2]*cells[2]+cells[3]*cells[3]);
	cells[0] /= norm;
	cells[1] /= norm;
	cells[2] /= norm;
	cells[3] /= norm;
}

double& Quaternion::operator [] (int k)
{
	return cells[k];
}

const double Quaternion::operator [] (int k) const
{
	return cells[k];
}

const Quaternion& Quaternion::operator = (const Quaternion& q)
{
	cells[0] = q[0];
	cells[1] = q[1];
	cells[2] = q[2];
	cells[3] = q[3];

	return (*this);
}

const Quaternion& Quaternion::operator = (const double q[])
{
	cells[0] = q[0];
	cells[1] = q[1];
	cells[2] = q[2];
	cells[3] = q[3];

	return (*this);
}

// q1.q2 = [s1,v1].[s2,v2] = [(s1 s2 - v1.v2), (s1v2 + s2v1 + v1xv2)]
const Quaternion& Quaternion::operator *= (const Quaternion &q2)
{
	Quaternion q1(*this);

	cells[0] = q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
	cells[1] = q1[1]*q2[0]+q1[0]*q2[1]-q1[3]*q2[2]+q1[2]*q2[3];
	cells[2] = q1[2]*q2[0]+q1[3]*q2[1]+q1[0]*q2[2]-q1[1]*q2[3];
	cells[3] = q1[3]*q2[0]-q1[2]*q2[1]+q1[1]*q2[2]+q1[0]*q2[3];

	normalize();

	return (*this);
}

// quat2glrotate(Quat qin, Quat qout)
void Quaternion::ToGLRotate(double q[])
{
	double cosdh, sindh; // cosdh = cos(d/2)

	cosdh = cells[0];
	sindh = sqrt(cells[1]*cells[1]
				+ cells[2]*cells[2]
				+ cells[3]*cells[3]);

	q[0] = (2.0 * acos(cosdh) * 180)/ M_PI;
	if(sindh < 0.00001) {
		q[1] = q[2] = 0.0;
		q[3] = 1.0;
	}
	else {
		q[1] = cells[1]/sindh;
		q[2] = cells[2]/sindh;
		q[3] = cells[3]/sindh;
	}
}

void	Quaternion::ToMatrix(double m[][4])
{
	double wx, wy, wz, xx, xy, xz, yy, yz, zz, x2, y2, z2;

	//	calculate coefficients
	x2 = cells[1] + cells[1];
	y2 = cells[2] + cells[2];
	z2 = cells[3] + cells[3];

	xx = cells[1] * x2;
	xy = cells[1] * y2;
	xz = cells[1] * z2;

	yy = cells[2] * y2;
	yz = cells[2] * z2;
	zz = cells[3] * z2;

	wx = cells[0] * x2;
	wy = cells[0] * y2;
	wz = cells[0] * z2;

	m[0][0] = 1 - (yy + zz);
	m[1][0] = xy - wz;
	m[2][0] = xz + wy;
	m[3][0] = 0.0;

	m[0][1] = xy + wz;
	m[1][1] = 1 - (xx +zz);
	m[2][1] = yz - wx;
	m[3][1] = 0.0;

	m[0][2] = xz - wy;
	m[1][2] = yz + wx;
	m[2][2] = 1 - (xx + yy);
	m[3][2] = 0.0;

	m[0][3] = 0;
	m[1][3] = 0;
	m[2][3] = 0;
	m[3][3] = 1;
}

Quaternion Quaternion::inverse()
{
	Quaternion qtmp(-cells[0], cells[1], cells[2], cells[3]);

	return qtmp;
}


Quaternion operator * (const Quaternion & q1, const Quaternion & q2)
{
	Quaternion result;

	result[0] =	q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
	result[1] = q1[1]*q2[0]+q1[0]*q2[1]-q1[3]*q2[2]+q1[2]*q2[3];
	result[2] = q1[2]*q2[0]+q1[3]*q2[1]+q1[0]*q2[2]-q1[1]*q2[3];
	result[3] = q1[3]*q2[0]-q1[2]*q2[1]+q1[1]*q2[2]+q1[0]*q2[3];

	result.normalize();
	return (result);
}

// vrotate(Quat q3, Vector vin, Vector vout)
 Point3D<double> operator * (const Quaternion & q3, const Point3D<double> &vin)
{
	Point3D<double> vout;
	double  q3W, q3X, q3Y, q3Z;

	// q3 * vin * (q3)^*
 
	q3W = q3[1]*vin[0] +q3[2]*vin[1] +q3[3]*vin[2];
	q3X = q3[0]*vin[0] -q3[3]*vin[1] +q3[2]*vin[2];
	q3Y = q3[3]*vin[0] +q3[0]*vin[1] -q3[1]*vin[2];
	q3Z = -q3[2]*vin[0] +q3[1]*vin[1] +q3[0]*vin[2];
 
	vout[0] = q3[1]*q3W +q3[0]*q3X -q3[3]*q3Y +q3[2]*q3Z;
	vout[1] = q3[2]*q3W +q3[3]*q3X +q3[0]*q3Y -q3[1]*q3Z;
	vout[2] = q3[3]*q3W -q3[2]*q3X +q3[1]*q3Y +q3[0]*q3Z;

	return (vout);
}

 void Quaternion::from_axis(double x, double y, double z, double angle_in_degree)
 {
	 // First we want to convert the degrees to radians 
	 // since the angle is assumed to be in radians
	 //GLfloat angle = GLfloat((angle_in_degree / 180.0f) * PI);
	 double angle = double(D2R(angle_in_degree));

	 // Here we calculate the sin( theta / 2) once for optimization
	 double result = sin( angle / 2.0f );

	 // Calculate the w value by cos( theta / 2 )
	 cells[0] = cos( angle / 2.0f );

	 // Calculate the x, y and z of the quaternion
	 cells[1] = (x * result);
	 cells[2] = (y * result);
	 cells[3] = (z * result);
 }