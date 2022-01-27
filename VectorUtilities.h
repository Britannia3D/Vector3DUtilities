#pragma once

/*
 * Author: David McHale, <mchale.d@yahoo.com>
 *
 * (C) Copyright 2022
 *
 * MIT License
 *
 * All algorithms untested, use at your own risk!
 *
*/

//Self-contained 3D vector class. Additional utility functions in ::Vector3DUtils
class Vector3D
{
public:

	double x, y, z;//The 3 VEC3 floats

	Vector3D(double x, double y, double z)//Constructor
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3D()//Overloaded Constructor
	{
		x = 0;
		y = 0;
		z = 0;
	}

	//[Mathematics operations]

	//Add two vectors
	Vector3D operator+(const Vector3D& inputVector) const
	{
		return Vector3D(x + inputVector.x, y + inputVector.y, z + inputVector.z);
	}

	//Subtract two vectors
	Vector3D operator-(const Vector3D& inputVector) const
	{
		return Vector3D(x - inputVector.x, y - inputVector.y, z - inputVector.z);
	}

	//Multiply two vectors
	Vector3D operator*(const Vector3D& inputVector) const
	{
		return Vector3D(x * inputVector.x, y * inputVector.y, z * inputVector.z);
	}

	//Divide two vectors
	Vector3D operator/(const Vector3D& inputVector) const
	{
		return Vector3D(x / inputVector.x, y / inputVector.y, z / inputVector.z);
	}


	//Add floating point to vector
	Vector3D operator+(double f) const
	{
		return Vector3D(f + x, f + y, f + z);
	}

	//Subtract floating point from vector
	Vector3D operator-(double f) const
	{
		return Vector3D(f - x, f - y, f - z);
	}

	//Multiply vector by floating point
	Vector3D operator*(double f) const
	{
		return Vector3D(f*x, f*y, f*z);
	}

	//Divide vector by floating point
	Vector3D operator/(double f) const
	{
		return Vector3D(f / x, f / y, f / z);
	}


	//[Cumulative operations]

	void operator+=(const Vector3D& inputVector)
	{
		this->x = inputVector.x + this->x;
		this->y = inputVector.y + this->y;
		this->z = inputVector.z + this->z;
	}

	void operator-=(const Vector3D& inputVector)
	{
		this->x -= inputVector.x + this->x;
		this->y -= inputVector.y + this->y;
		this->z -= inputVector.z + this->z;
	}

	void operator*=(const Vector3D& inputVector)
	{
		this->x = inputVector.x * this->x;
		this->y = inputVector.y * this->y;
		this->z = inputVector.z * this->z;
	}

	void operator/=(const Vector3D& inputVector)
	{
		this->x = inputVector.x / this->x;
		this->y = inputVector.y / this->y;
		this->z = inputVector.z / this->z;
	}

	void operator*=(double f)
	{
		this->x = f * this->x;
		this->y = f * this->y;
		this->z = f * this->z;
	}

	void operator/=(double f)
	{
		this->x = f / this->x;
		this->y = f / this->y;
		this->z = f / this->z;
	}
};


class Vector3DUtils
{
public:

	//Euclidean distance
	float dist(float x1, float y1, float z1, float x2, float y2, float z2);

	//Cross product of two Vector3D's.
	Vector3D cross(const Vector3D& A, const Vector3D& B);

	//Dot product
	double dot(Vector3D input1, const Vector3D& input2);

	//Vector plane intersect
	Vector3D intersectPoint(Vector3D rayVector, Vector3D rayPoint, Vector3D planeNormal, Vector3D planePoint);

	//Get arbitrary 3d vector that is perpendicular to the parameter vector	
	//There are infinite such vectors, return one such.
	Vector3D arbitraryOrthogonal(Vector3D vec);

	//Use spherical coordinates to get a position
	Vector3D OrbitalPosition(float angle1, float angle2, Vector3D centroid);

	//Set the length (magnitude) of a given vector
	Vector3D setVectorMagitude(Vector3D input, float newMag);

	//Get magnitude of vector
	double length(Vector3D vec);

	//Linear interpolation
	Vector3D lerp(Vector3D a, Vector3D b, float scale);

	//Move vector towards another vector by a set amount
	Vector3D displaceVectorTowards(Vector3D a, Vector3D b, float ammount);

	Vector3D normalize(Vector3D vec);
};