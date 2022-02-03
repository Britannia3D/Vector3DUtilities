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

#include <iostream>
#include <sstream>

#include "VectorUtilities.h"

using namespace std;

//Euclidean distance
float Vector3DUtils::dist(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return(sqrt(pow((float)(x2 - x1), (float)2) + pow((float)(y2 - y1), (float)2) + pow((float)(z2 - z1), (float)2)));
}

//Cross product of two vector array.
Vector3D Vector3DUtils::cross(const Vector3D& A, const Vector3D& B)
{
	Vector3D crossP(0, 0, 0);
	crossP.x = A.y * B.z - A.z * B.y;
	crossP.y = A.z * B.x - A.x * B.z;
	crossP.z = A.x * B.y - A.y * B.x;

	return crossP;
}

double Vector3DUtils::dot(Vector3D input1, Vector3D input2)
{
	return input1.x * input2.x + input1.y * input2.y + input1.z * input2.z;
}

Vector3D Vector3DUtils::intersectPlane(Vector3D rayVector, Vector3D rayPoint, Vector3D planeNormal, Vector3D planePoint)
{
	Vector3D diff = rayPoint - planePoint;
	double prod1 = dot(diff, planeNormal);
	double prod2 = dot(rayVector, planeNormal);
	double prod3 = prod1 / prod2;
	return rayPoint - rayVector * prod3;
}


//The vector actually intersects the plane given its direction
bool Vector3DUtils::doesIntersect(Vector3D &n, Vector3D &p0, Vector3D &l0, Vector3D &l)
{
	float t;
	//assuming vectors are all normalized
	float denom = dot(n, l);

	//if (denom > 1e-6)
	//{
	Vector3D p0l0 = p0 - l0;
	t = dot(p0l0, n) / denom;
	return (t >= 0);
	//}

	return false;
}


bool Vector3DUtils::intersectTriangle(Vector3D *intersectPoint, Vector3D rayVector, Vector3D rayPoint, Vector3D A, Vector3D B, Vector3D C)
{
	Vector3D dir = cross(B - A, C - A);
	Vector3D triangleNormal = normalize(dir);

	Vector3D centroid;

	centroid.x = (A.x + B.x + C.x) / 3;
	centroid.y = (A.y + B.y + C.y) / 3;
	centroid.z = (A.z + B.z + C.z) / 3;

	//cout << "centroid: " << centroid.x  << " " << centroid.y << " " << centroid.z << endl;

	Vector3D Q = intersectPlane(rayVector, rayPoint, triangleNormal, centroid);

	bool does = doesIntersect(triangleNormal, centroid, rayPoint, rayVector);
	cout << "intersect plane?: " << does << endl;

	Vector3D op1 = cross((B - A), (Q - A));
	Vector3D op2 = cross((C - B), (Q - B));
	Vector3D op3 = cross((A - C), (Q - C));

	//dot(op1, triangleNormal);
	//dot(op2, triangleNormal);
	//dot(op3, triangleNormal);
	//Vector3D op1b = dot(op1, triangleNormal);
	//Vector3D op2b = dot(op2, triangleNormal);
	//Vector3D op3b = dot(op3, triangleNormal);

	//cout << op1b.x << " " << op1b.y << " " << op1b.z << endl;
	//cout << op2b.x << " " << op2b.y << " " << op2b.z << endl;
	//cout << op3b.x << " " << op3b.y << " " << op3b.z << endl;

	intersectPoint = &Q;

	cout <<"Intersect at: " << Q.x << " , " << Q.y << " , " << Q.z << endl;

	//Return if intersects the triangle
	if (dot(op1, triangleNormal) >= 0)
	{
		if (dot(op2, triangleNormal) >= 0)
		{
			if (dot(op3, triangleNormal) >= 0)
			{
				return 1;
			}
		}
	}

	return 0;
}

//Get arbitrary 3d vector that is perpendicular to the parameter vector
//There are infinite such vectors, get one such
Vector3D Vector3DUtils::arbitraryOrthogonal(Vector3D vec)
{
	bool b0 = (vec.x < vec.y) && (vec.x < vec.z);
	bool b1 = (vec.y <= vec.x) && (vec.y < vec.z);
	bool b2 = (vec.z <= vec.x) && (vec.z <= vec.y);

	Vector3D op(0, 0, 0);
	op = cross(vec, Vector3D(int(b0), int(b1), int(b2)));

	return op;
}

//Use spherical coordinates to get a position
Vector3D Vector3DUtils::OrbitalPosition(float angle1, float angle2, Vector3D centre, float radius)
{
	float Theta = angle1;
	float Phi = angle2;
	float Y = radius * sin(Theta);
	float X = radius * cos(Theta) * cos(Phi);
	float Z = radius * cos(Theta) * sin(Phi);

	return Vector3D(X + centre.x, Y + centre.y, Z + centre.z);
}

//Set the length (magnitude) of a given vector
Vector3D Vector3DUtils::setVectorMagitude(Vector3D input, float newMag)
{
	float mag = sqrt(input.x * input.x + input.y * input.y + input.z * input.z);

	float new_x = input.x * newMag / mag;
	float new_y = input.y * newMag / mag;
	float new_z = input.z * newMag / mag;

	Vector3D op(new_x, new_y, new_z);
	return op;
}


Vector3D Vector3DUtils::lerp(Vector3D a, Vector3D b, float scale)
{
	Vector3D op0(0, 0, 0);

	//[End-Start]
	op0.x = b.x - a.x;
	op0.y = b.y - a.y;
	op0.z = b.z - a.z;
	//[Multiply by scale]
	op0 *= scale;

	Vector3D op1(0, 0, 0);
	op1.x = a.x + op0.x;
	op1.y = a.y + op0.y;
	op1.z = a.z + op0.z;

	return(op1);
}


Vector3D Vector3DUtils::displaceVectorTowards(Vector3D a, Vector3D b, float ammount)
{
	Vector3D op0(0, 0, 0);

	//[End-Start]
	op0.x = b.x - a.x;
	op0.y = b.y - a.y;
	op0.z = b.z - a.z;
	
	Vector3D op1(op0.x, op0.y, op0.z);
	Vector3D vi(op1.x, op1.y, op1.z);
	float vLen0 = length(vi);
	float vLen1 = 1 / vLen0;//Amount to scale to increase by 1

	Vector3D op3(op1.x, op1.y, op1.z);
	op3 *= vLen1 * ammount;

	Vector3D op2(0, 0, 0);
	op2.x = a.x + op3.x;
	op2.y = a.y + op3.y;
	op2.z = a.z + op3.z;

	return(op2);
}

double Vector3DUtils::length(Vector3D vec)
{
	return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

Vector3D Vector3DUtils::normalize(Vector3D vec)
{
	float op1 = pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2);
	op1 = sqrt(op1);

	Vector3D op;
	op.x = vec.x / op1;
	op.y = vec.y / op1;
	op.z = vec.z / op1;

	return op;
}

Vector3D Vector3DUtils::getNormal(Vector3D vertex1, Vector3D vertex2, Vector3D vertex3)
{
	Vector3D dir = cross(vertex2 - vertex1, vertex3 - vertex1);
	Vector3D norm = normalize(dir);

	return norm;
}

bool Vector3DUtils::RayTriangleIntersect(Vector3D rayOrigin, Vector3D rayVector, Vector3D* v1, Vector3D* v2, Vector3D* v3, Vector3D& outIntersectionPoint)
{
	const float EPSILON = 0.0000001;
	Vector3D vertex0 = *v1;
	Vector3D vertex1 = *v2;
	Vector3D vertex2 = *v3;
	Vector3D edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;

	h = cross(rayVector, edge2);

	a = dot(edge1, h);
	if (a > -EPSILON && a < EPSILON)
		return false;  
	
	f = 1.0 / a;
	s = rayOrigin - vertex0;
	u = f * dot(s, h);
	if (u < 0.0 || u > 1.0)
		return false;
	
	q = cross(s, edge1);
	v = f * dot(rayVector, q);
	if (v < 0.0 || u + v > 1.0)
		return false;
	
	float t = f * dot(edge2, q);
	if (t > EPSILON)
	{
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else
		return false;
}


//Algorithm to test if the line actually reaches the poly rather than extending to infinity
bool Vector3DUtils::LineTriangleIntersect(Vector3D lineStart, Vector3D lineEnd, Vector3D* v1, Vector3D* v2, Vector3D* v3, Vector3D* outIntersectionPoint)
{
	Vector3D rayVector(0,0,0);
	rayVector = vecUtils.normalize(lineEnd - lineStart);

	//Vector3D rayVector(0, 0, -1);

	const float EPSILON = 0.0000001;
	Vector3D vertex0 = *v1;
	Vector3D vertex1 = *v2;
	Vector3D vertex2 = *v3;
	Vector3D edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;

	//h = rayVector.crossProduct(edge2);
	h = cross(rayVector, edge2);

	//a = edge1.dotProduct(h);
	a = dot(edge1, h);
	if (a > -EPSILON && a < EPSILON)
		return false;//This ray is parallel to this triangle.

	f = 1.0 / a;
	s = lineStart - vertex0;
	u = f * dot(s, h);//s.dotProduct(h);
	if (u < 0.0 || u > 1.0)
		return false;

	q = cross(s, edge1);//s.crossProduct(edge1);
	v = f * dot(rayVector, q);//rayVector.dotProduct(q);
	if (v < 0.0 || u + v > 1.0)
		return false;

	//At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * dot(edge2, q);//edge2.dotProduct(q);
	if (t > EPSILON)//ray intersection
	{
		*outIntersectionPoint = lineStart + rayVector * t;
		//return true;
	}
	else//This means that there is a line intersection but not a ray intersection.
	{
		return false;
	}

	float d1 = dist(lineStart.x, lineStart.y, lineStart.z, outIntersectionPoint->x, outIntersectionPoint->y, outIntersectionPoint->z);
	float d2 = dist(lineEnd.x, lineEnd.y, lineEnd.z, outIntersectionPoint->x, outIntersectionPoint->y, outIntersectionPoint->z);
	
	bool furthest = 0;
	if (d1 < d2) { furthest = 1; }

	float lineLength = dist(lineStart.x, lineStart.y, lineStart.z, lineEnd.x, lineEnd.y, lineEnd.z);

	if (furthest == 0)//D2
	{	
		if (lineLength > d1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		if (lineLength > d2)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}