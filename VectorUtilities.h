#pragma once

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

	//Mathematics operations
	Vector3D operator-(const Vector3D& rhs) const
	{
		return Vector3D(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	Vector3D operator-(double rhs) const
	{
		return Vector3D(rhs - x, rhs - y, rhs - z);
	}

	Vector3D operator+(double rhs) const
	{
		return Vector3D(rhs + x, rhs + y, rhs + z);
	}

	Vector3D operator+(const Vector3D& rhs) const
	{
		return Vector3D(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	Vector3D operator*(double rhs) const
	{
		return Vector3D(rhs*x, rhs*y, rhs*z);
	}

	Vector3D operator/(double rhs) const
	{
		return Vector3D(rhs / x, rhs / y, rhs / z);
	}

	void operator+=(const Vector3D& rhs)
	{
		//return Vector3D(rhs.x + x, rhs.y + y, rhs.z + z);

		this->x = rhs.x + this->x;
		this->y = rhs.y + this->y;
		this->z = rhs.z + this->z;
	}

	void operator*=(double rhs)
	{
		this->x = rhs * this->x;
		this->y = rhs * this->y;
		this->z = rhs * this->z;
	}

	void operator/=(double rhs)
	{
		this->x = rhs / this->x;
		this->y = rhs / this->y;
		this->z = rhs / this->z;
	}

	void operator-=(const Vector3D& rhs)
	{
		this->x -= rhs.x + this->x;
		this->y -= rhs.y + this->y;
		this->z -= rhs.z + this->z;
	}

};


class Vector3DUtils
{
public:

	//Euclidean distance
	float dist(float x1, float y1, float z1, float x2, float y2, float z2);

	//Cross product of two vector array.
	Vector3D cross(const Vector3D& A, const Vector3D& B);

	double dot(Vector3D input1, const Vector3D& rhs);

	Vector3D intersectPoint(Vector3D rayVector, Vector3D rayPoint, Vector3D planeNormal, Vector3D planePoint);

	//Get arbitrary 3d vector that is perpendicular to the parameter vector	
	//There are infinite such vectors, return one such.
	Vector3D arbitrary_orthogonal(Vector3D vec);

	//Use spherical coordinates to get a position
	Vector3D OrbitalPosition(float angle1, float angle2, Vector3D centroid);

	//Set the length (magnitude) of a given vector
	Vector3D setVectorMagitude(Vector3D input, float newMag);

	double length(Vector3D vec);

	Vector3D normalize(Vector3D vec);
};