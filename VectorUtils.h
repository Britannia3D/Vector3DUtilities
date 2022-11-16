#pragma once

class Quad
{
public:
	int a;
	int b;
	int c;
	int d;
};

class Vector4D
{
public:

	double x, y, z, w;
};

/////////////////////////////////////////////////////////////////////////
// Self-contained 3D vector class. Utility functions in ::vecUtils
/////////////////////////////////////////////////////////////////////////
class Vector3D
{
public:

	double x, y, z;

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
		this->x = this->x / inputVector.x;
		this->y = this->y / inputVector.y;
		this->z = this->z / inputVector.z;
	}

	void operator*=(double f)
	{
		this->x = f * this->x;
		this->y = f * this->y;
		this->z = f * this->z;
	}

	void operator/=(double f)
	{
		this->x = this->x / f;
		this->y = this->y / f;
		this->z = this->z / f;
	}
};

class Vector2D
{
public:

	float x;
	float y;

	Vector2D(float x, float y)//Constructor
	{
		this->x = x;
		this->y = y;
	}

	Vector2D()//Overloaded Constructor
	{
		x = 0;
		y = 0;
	}
};

/////////////////////////////////////////////////////////////////////////
// Vector Utilties
/////////////////////////////////////////////////////////////////////////
class VectorUtils
{
public:

	//Euclidean distance
	float dist(float x1, float y1, float z1, float x2, float y2, float z2);

	float dist(Vector3D a, Vector3D b);

	//Cross product of two Vector3D's.
	Vector3D cross(const Vector3D& A, const Vector3D& B);

	//Dot product
	double dot(Vector3D input1, const Vector3D& input2);

	Vector3D arbitraryOrthogonal(Vector3D vec);

	//Set the length (magnitude) of a given vector
	Vector3D setVectorMagnitude(Vector3D input, float newMag);

	//Get magnitude of vector
	double length(Vector3D vec);

	double speed(Vector3D vec);

	Vector3D normalize(Vector3D vec);

	Vector3D displaceVectorTowards(Vector3D a, Vector3D b, float amount);

	Vector3D closestPlanePoint(Vector3D pointPosition, Vector3D planePosition, Vector3D planeNormal);

	double pointDistToLine2D(Vector2D A, Vector2D B, Vector2D point);

	Vector3D closestPointAlongLine(Vector3D lineFrom, Vector3D lineTo, Vector3D point);

	float angularDifference(Vector3D a, Vector3D b);

	bool rayTriangleIntersect(Vector3D rayOrigin, Vector3D rayVector, Vector3D* v1, Vector3D* v2, Vector3D* v3, Vector3D& outIntersectionPoint);//Möller–Trumbore intersection algorithm

	bool insideTriangle2D(Vector3D pt, Vector3D v1, Vector3D v2, Vector3D v3);

	Vector3D lerp3D(Vector3D a, Vector3D b, float scale);

	int angleOf2DVector(Vector3D vec);

	bool pointOnLine_2(Vector3D a, Vector3D b, Vector3D point);

	bool pointOnLine_3(Vector3D a, Vector3D b, Vector3D point);//The intersect point lies within the two line points

	float angularDifference2D(float p1x, float p1y, float p2x, float p2y);

	Vector3D cartesanToSpherical(float x, float y, float z);//Cartesan coordinates to spherical

	Vector3D sphericalToCartesan(float theta, float phi, float radius);//Spherical coordinates to cartesan

	Vector2D orthogonalVector2D_CW(Vector2D a, Vector2D b);//Get a vector perpendicular to two 2D vectors
	
	Vector2D orthogonalVector2D_CCW(Vector2D a, Vector2D b);

private:

	float sign(Vector3D p1, Vector3D p2, Vector3D p3);
};

