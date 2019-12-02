// Point3D.h: interface for the Point3D class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_POINT3D_H__83C30DA1_0090_11D5_9C75_0000B4A67F6E__INCLUDED_)
#define AFX_POINT3D_H__83C30DA1_0090_11D5_9C75_0000B4A67F6E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <stdlib.h>
#include <math.h>

template < class T >
class Point3D  
{
public:
	Point3D();
	Point3D(T, T, T);
	virtual ~Point3D();
	T& operator [] (int);					// indexing
	const T operator [] (int) const;		// indexing
	inline void SetValues(T x, T y, T z) {
		cells[0] = x; cells[1] = y; cells[2] = z; }
	const Point3D& operator = (const Point3D&);	// assignment
	const Point3D& operator = (const T[]); // assignment double array
	const Point3D& operator = (const T);
	const Point3D& operator += (const Point3D &);
	const Point3D& operator -= (const Point3D &);
	const Point3D& operator *= (const T);
	const Point3D& operator /= (const T);
	const bool operator == (const T);
	const bool operator != (const T);
	double DistanceTo(Point3D&);
	Point3D crossProduct(Point3D&);
	void normalize();	// it's only for Point3D<double>

protected:
	T cells[3];
};

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

template <class T>
Point3D<T>::Point3D()
{

}

template <class T>
Point3D<T>::Point3D(T x, T y, T z)
{
	cells[0] = x;
	cells[1] = y;
	cells[2] = z;
}

template <class T>
Point3D<T>::~Point3D()
{

}

template <class T>
const T Point3D<T>::operator [] (int k) const
{
	return cells[k];
}

template <class T>
T & Point3D<T>::operator [] (int k)
{
	return cells[k];
}

template <class T>
const Point3D<T> & Point3D<T>::operator = (const Point3D& p)
{
	cells[0] = p[0];
	cells[1] = p[1];
	cells[2] = p[2];

	return (*this);
}

template <class T>
const Point3D<T>& Point3D<T>::operator = (const T p[])
{
	cells[0] = p[0];
	cells[1] = p[1];
	cells[2] = p[2];

	return (*this);
}

template <class T>
const Point3D<T>& Point3D<T>::operator = (const T p)
{
	cells[0] = p;
	cells[1] = p;
	cells[2] = p;

	return (*this);
}

template <class T>
const Point3D<T>& Point3D<T>::operator += (const Point3D& p)
{
	cells[0] += p[0];
	cells[1] += p[1];
	cells[2] += p[2];

	return (*this);
}

template <class T>
const Point3D<T>& Point3D<T>::operator -= (const Point3D& p)
{
	cells[0] -= p[0];
	cells[1] -= p[1];
	cells[2] -= p[2];

	return (*this);
}

template <class T>
const Point3D<T>& Point3D<T>::operator *= (const T p)
{
	cells[0] *= p;
	cells[1] *= p;
	cells[2] *= p;

	return (*this);
}

template <class T>
const Point3D<T>& Point3D<T>::operator /= (const T p)
{
	cells[0] /= p;
	cells[1] /= p;
	cells[2] /= p;

	return (*this);
}

template <class T>
const bool Point3D<T>::operator == (const T p)
{
	if (cells[0] != p) return false;
	if (cells[1] != p) return false;
	if (cells[2] != p) return false;

	return true;
}

template <class T>
const bool Point3D<T>::operator != (const T p)
{
	if (cells[0] != p) return true;
	if (cells[1] != p) return true;
	if (cells[2] != p) return true;

	return false;
}

template <class T>
double Point3D<T>::DistanceTo(Point3D& its)
{
	double d1 = cells[0] - its[0];
	double d2 = cells[1] - its[1];
	double d3 = cells[2] - its[2];

	return sqrt(d1*d1 + d2*d2 + d3*d3);
}

template <class T>
Point3D<T> Point3D<T>::crossProduct(Point3D& its)
{
	Point3D<T> result;

	result[0] = cells[1]*its[2] - cells[2]*its[1];
	result[1] = cells[2]*its[0] - cells[0]*its[2];
	result[2] = cells[0]*its[1] - cells[1]*its[0];

	return result;
}

template <class T>
void Point3D<T>::normalize()
{	// it's only for Point3D<double>
	double norm;

	norm = sqrt(cells[0]*cells[0] + cells[1]*cells[1] + cells[2]*cells[2]);
	cells[0] /= norm;
	cells[1] /= norm;
	cells[2] /= norm;
}

// prototypes for nonmember functions that operate on Point3D

// I/O operations
// overload << operator
// overload + addition of two Point3D
template <class T>
Point3D<T> operator + (const Point3D<T> & lhs, const Point3D<T> & rhs)
{
	Point3D<T> result;

	result[0] = lhs[0]+rhs[0];
	result[1] = lhs[1]+rhs[1];
	result[2] = lhs[2]+rhs[2];

	return result;
}

// overload - subtraction of two Point3D
template <class T>
Point3D<T> operator - (const Point3D<T> & lhs, const Point3D<T> & rhs)
{
	Point3D<T> result;

	result[0] = lhs[0]-rhs[0];
	result[1] = lhs[1]-rhs[1];
	result[2] = lhs[2]-rhs[2];

	return result;
}

// overload * inner product of two Point3D
template <class T>
double operator * (const Point3D<T> & lhs, const Point3D<T> & rhs)
{

	return lhs[0]*rhs[0] + lhs[1]*rhs[1] + lhs[2]*rhs[2];
}

// overload * scalar multiplication of Point3D
template <class T>
Point3D<T> operator * (const Point3D<T> & lhs, const T rhs)
{
	Point3D<T> result;

	result[0] = lhs[0]*rhs;
	result[1] = lhs[1]*rhs;
	result[2] = lhs[2]*rhs;

	return result;
}

// overload * scalar multiplication of Point3D
template <class T>
Point3D<T> operator * (const T lhs, const Point3D<T> & rhs)
{
	Point3D<T> result;

	result[0] = lhs*rhs[0];
	result[1] = lhs*rhs[1];
	result[2] = lhs*rhs[2];

	return result;
}

// overload * scalar division of Point3D
template <class T>
Point3D<T> operator / (const Point3D<T> & lhs, const T rhs)
{
	Point3D<T> result;

	result[0] = lhs[0]/rhs;
	result[1] = lhs[1]/rhs;
	result[2] = lhs[2]/rhs;

	return result;
}

// overload == equality of two Point3D
template <class T>
bool operator == (const Point3D<T> & lhs, const Point3D<T> & rhs)
{
	return (lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2]);
}

// external variable
extern Point3D<double> zeroV;
extern Point3D<double> xAxis;
extern Point3D<double> yAxis;
extern Point3D<double> zAxis;

#endif // !defined(AFX_POINT3D_H__83C30DA1_0090_11D5_9C75_0000B4A67F6E__INCLUDED_)
