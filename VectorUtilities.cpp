#include <iostream>
#include <sstream>

#include "JointHeader.h"

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

float Vector3DUtils::angularDifference(Vector3D a, Vector3D b)
{
	float angle = std::acos(dot(a, b) / (length(a)*length(b)));
	return angle;
}



double Vector3DUtils::dot(Vector3D input1, const Vector3D& input2)
{
	return input1.x * input2.x + input1.y * input2.y + input1.z * input2.z;
}

Vector3D Vector3DUtils::intersectPoint(Vector3D rayVector, Vector3D rayPoint, Vector3D planeNormal, Vector3D planePoint)
{
	Vector3D diff = rayPoint - planePoint;
	double prod1 = dot(diff, planeNormal);
	double prod2 = dot(rayVector, planeNormal);
	double prod3 = prod1 / prod2;
	return rayPoint - rayVector * prod3;
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
Vector3D Vector3DUtils::OrbitalPosition(float angle1, float angle2, Vector3D centroid)
{
	float sx = centroid.x;// -0.013;
	float sy = centroid.y;// 1.06;
	float sz = centroid.z;// 1.06;

	float Theta = angle1;
	float Phi = angle2;
	float radius = 1.0;
	float Y = radius * sin(Theta);
	float X = radius * cos(Theta) * cos(Phi);
	float Z = radius * cos(Theta) * sin(Phi);

	return Vector3D(X + sx, Y + sy, Z + sz);
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


Vector3D Vector3DUtils::displaceVectorTowards(Vector3D a, Vector3D b, float amount)
{

	if (utils.dist(a.x, a.y, a.z, b.x, b.y, b.z) <= 0.0)
	{
		//Vector3D op4(256, 256, 256);
		return(a);
	}

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
	op3 *= vLen1 * amount;

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


// Algorithm to test if the line actually reaches rather than extending to infinity
// Get position of intersect, if it lies between start and end of the line then its a hit
// Get Top XYZ and Bottom XYZ

Vector3D vertex0;
Vector3D vertex1;
Vector3D vertex2;
Vector3D edge1, edge2, h, s, q;
const float EPSILON = 0.0000001;
Vector3D rayVector;
bool Vector3DUtils::LineTriangleIntersect(Vector3D lineStart, Vector3D lineEnd, Vector3D v1, Vector3D v2, Vector3D v3, Vector3D* outIntersectionPoint)
{
	rayVector = vector3DUtils.normalize(lineEnd - lineStart);

	//Vector3D rayVector(0, 0, -1);

	vertex0 = v1;
	vertex1 = v2;
	vertex2 = v3;

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
	else//No ray intersection
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

	//h = rayVector.crossProduct(edge2);
	h = cross(rayVector, edge2);

	//a = edge1.dotProduct(h);
	a = dot(edge1, h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.

	f = 1.0 / a;
	s = rayOrigin - vertex0;
	u = f * dot(s, h);//s.dotProduct(h);
	if (u < 0.0 || u > 1.0)
		return false;

	q = cross(s, edge1);// s.crossProduct(edge1);
	v = f * dot(rayVector, q);// rayVector.dotProduct(q);
	if (v < 0.0 || u + v > 1.0)
		return false;

	// At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * dot(edge2, q);// edge2.dotProduct(q);
	if (t > EPSILON) // ray intersection
	{
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}


//--------------------------------------------------------------------------------------------------------------------


void Vector3DUtils::VmV(float Vr[3], const float V1[3], const float V2[3])
{
	Vr[0] = V1[0] - V2[0];
	Vr[1] = V1[1] - V2[1];
	Vr[2] = V1[2] - V2[2];
}

float Vector3DUtils::VdotV(const float V1[3], const float V2[3])
{
	return (V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]);
}

void Vector3DUtils::VcV(float Vr[3], const float V[3])
{
	Vr[0] = V[0];  Vr[1] = V[1];  Vr[2] = V[2];
}

void Vector3DUtils::VpV(float Vr[3], const float V1[3], const float V2[3])
{
	Vr[0] = V1[0] + V2[0];
	Vr[1] = V1[1] + V2[1];
	Vr[2] = V1[2] + V2[2];
}

void Vector3DUtils::VpVxS(float Vr[3], const float V1[3], const float V2[3], float s)
{
	Vr[0] = V1[0] + V2[0] * s;
	Vr[1] = V1[1] + V2[1] * s;
	Vr[2] = V1[2] + V2[2] * s;
}

void Vector3DUtils::VcrossV(float Vr[3], const float V1[3], const float V2[3])
{
	Vr[0] = V1[1] * V2[2] - V1[2] * V2[1];
	Vr[1] = V1[2] * V2[0] - V1[0] * V2[2];
	Vr[2] = V1[0] * V2[1] - V1[1] * V2[0];
}

void Vector3DUtils::VxS(float Vr[3], const float V[3], float s)
{
	Vr[0] = V[0] * s;
	Vr[1] = V[1] * s;
	Vr[2] = V[2] * s;
}

float Vector3DUtils::VdistV2(const float V1[3], const float V2[3])
{
	return ((V1[0] - V2[0]) * (V1[0] - V2[0]) +
		(V1[1] - V2[1]) * (V1[1] - V2[1]) +
		(V1[2] - V2[2]) * (V1[2] - V2[2]));
}

void Vector3DUtils::MxVpV(float Vr[3], const float M1[3][3], const float V1[3], const float V2[3])
{
	Vr[0] = (M1[0][0] * V1[0] +
		M1[0][1] * V1[1] +
		M1[0][2] * V1[2] +
		V2[0]);
	Vr[1] = (M1[1][0] * V1[0] +
		M1[1][1] * V1[1] +
		M1[1][2] * V1[2] +
		V2[1]);
	Vr[2] = (M1[2][0] * V1[0] +
		M1[2][1] * V1[1] +
		M1[2][2] * V1[2] +
		V2[2]);
}


//-------------------------------------
//
//-------------------------------------
void Vector3DUtils::SegPoints(float VEC[3], float X[3], float Y[3], // closest points
	const float P[3], const float A[3], // seg 1 origin, vector
	const float Q[3], const float B[3]) // seg 2 origin, vector
{
	float T[3], A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
	float TMP[3];

	VmV(T, Q, P);
	A_dot_A = VdotV(A, A);
	B_dot_B = VdotV(B, B);
	A_dot_B = VdotV(A, B);
	A_dot_T = VdotV(A, T);
	B_dot_T = VdotV(B, T);

	// t parameterizes ray P,A 
	// u parameterizes ray Q,B 

	float t, u;

	// compute t for the closest point on ray P,A to
	// ray Q,B

	float denom = A_dot_A * B_dot_B - A_dot_B * A_dot_B;

	t = (A_dot_T*B_dot_B - B_dot_T * A_dot_B) / denom;

	// clamp result so t is on the segment P,A

	if ((t < 0) || isnan(t)) t = 0; else if (t > 1) t = 1;

	// find u for point on ray Q,B closest to point at t

	u = (t*A_dot_B - B_dot_T) / B_dot_B;

	// if u is on segment Q,B, t and u correspond to 
	// closest points, otherwise, clamp u, recompute and
	// clamp t 

	if ((u <= 0) || isnan(u)) {

		VcV(Y, Q);

		t = A_dot_T / A_dot_A;

		if ((t <= 0) || isnan(t)) {
			VcV(X, P);
			VmV(VEC, Q, P);
		}
		else if (t >= 1) {
			VpV(X, P, A);
			VmV(VEC, Q, X);
		}
		else {
			VpVxS(X, P, A, t);
			VcrossV(TMP, T, A);
			VcrossV(VEC, A, TMP);
		}
	}
	else if (u >= 1) {

		VpV(Y, Q, B);

		t = (A_dot_B + A_dot_T) / A_dot_A;

		if ((t <= 0) || isnan(t)) {
			VcV(X, P);
			VmV(VEC, Y, P);
		}
		else if (t >= 1) {
			VpV(X, P, A);
			VmV(VEC, Y, X);
		}
		else {
			VpVxS(X, P, A, t);
			VmV(T, Y, P);
			VcrossV(TMP, T, A);
			VcrossV(VEC, A, TMP);
		}
	}
	else {

		VpVxS(Y, Q, B, u);

		if ((t <= 0) || isnan(t)) {
			VcV(X, P);
			VcrossV(TMP, T, B);
			VcrossV(VEC, B, TMP);
		}
		else if (t >= 1) {
			VpV(X, P, A);
			VmV(T, Q, X);
			VcrossV(TMP, T, B);
			VcrossV(VEC, B, TMP);
		}
		else {
			VpVxS(X, P, A, t);
			VcrossV(VEC, A, B);
			if (VdotV(VEC, T) < 0) {
				VxS(VEC, VEC, -1);
			}
		}
	}
}


//-------------------------------------
// Minimum distance between two triangles
//-------------------------------------
float Vector3DUtils::triDist(float P[3], float Q[3], const float S[3][3], const float T[3][3])
{
	//Compute vectors along the 6 sides

	float Sv[3][3], Tv[3][3];
	float VEC[3];

	VmV(Sv[0], S[1], S[0]);
	VmV(Sv[1], S[2], S[1]);
	VmV(Sv[2], S[0], S[2]);

	VmV(Tv[0], T[1], T[0]);
	VmV(Tv[1], T[2], T[1]);
	VmV(Tv[2], T[0], T[2]);

	float V[3];
	float Z[3];
	float minP[3], minQ[3], mindd;
	int shown_disjoint = 0;

	mindd = VdistV2(S[0], T[0]) + 1;  // Set first minimum safely high

	for (unsigned short i = 0; i < 3; i++)
	{
		for (unsigned short j = 0; j < 3; j++)
		{
			// Find closest points on edges i & j, plus the 
			// vector (and distance squared) between these points

			SegPoints(VEC, P, Q, S[i], Sv[i], T[j], Tv[j]);

			VmV(V, Q, P);
			float dd = VdotV(V, V);

			// Verify this closest point pair only if the distance 
			// squared is less than the minimum found thus far.

			if (dd <= mindd)
			{
				VcV(minP, P);
				VcV(minQ, Q);
				mindd = dd;

				VmV(Z, S[(i + 2) % 3], P);
				float a = VdotV(Z, VEC);
				VmV(Z, T[(j + 2) % 3], Q);
				float b = VdotV(Z, VEC);

				if ((a <= 0) && (b >= 0)) return sqrt(dd);

				float p = VdotV(V, VEC);

				if (a < 0) a = 0;
				if (b > 0) b = 0;
				if ((p - a + b) > 0) shown_disjoint = 1;
			}
		}
	}


	float Sn[3], Snl;
	VcrossV(Sn, Sv[0], Sv[1]); // Compute normal to S triangle
	Snl = VdotV(Sn, Sn);      // Compute square of length of normal

	// If cross product is long enough,

	if (Snl > 1e-15)
	{
		// Get projection lengths of T points

		float Tp[3];

		VmV(V, S[0], T[0]);
		Tp[0] = VdotV(V, Sn);

		VmV(V, S[0], T[1]);
		Tp[1] = VdotV(V, Sn);

		VmV(V, S[0], T[2]);
		Tp[2] = VdotV(V, Sn);

		// If Sn is a separating direction,
		// find point with smallest projection

		int point = -1;
		if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
		{
			if (Tp[0] < Tp[1]) point = 0; else point = 1;
			if (Tp[2] < Tp[point]) point = 2;
		}
		else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
		{
			if (Tp[0] > Tp[1]) point = 0; else point = 1;
			if (Tp[2] > Tp[point]) point = 2;
		}

		// If Sn is a separating direction, 

		if (point >= 0)
		{
			shown_disjoint = 1;

			// Test whether the point found, when projected onto the 
			// other triangle, lies within the face.

			VmV(V, T[point], S[0]);
			VcrossV(Z, Sn, Sv[0]);
			if (VdotV(V, Z) > 0)
			{
				VmV(V, T[point], S[1]);
				VcrossV(Z, Sn, Sv[1]);
				if (VdotV(V, Z) > 0)
				{
					VmV(V, T[point], S[2]);
					VcrossV(Z, Sn, Sv[2]);
					if (VdotV(V, Z) > 0)
					{
						// T[point] passed the test - it's a closest point for 
						// the T triangle; the other point is on the face of S

						VpVxS(P, T[point], Sn, Tp[point] / Snl);
						VcV(Q, T[point]);
						return sqrt(VdistV2(P, Q));
					}
				}
			}
		}
	}

	float Tn[3], Tnl;
	VcrossV(Tn, Tv[0], Tv[1]);
	Tnl = VdotV(Tn, Tn);

	if (Tnl > 1e-15)
	{
		float Sp[3];

		VmV(V, T[0], S[0]);
		Sp[0] = VdotV(V, Tn);

		VmV(V, T[0], S[1]);
		Sp[1] = VdotV(V, Tn);

		VmV(V, T[0], S[2]);
		Sp[2] = VdotV(V, Tn);

		int point = -1;
		if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
		{
			if (Sp[0] < Sp[1]) point = 0; else point = 1;
			if (Sp[2] < Sp[point]) point = 2;
		}
		else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
		{
			if (Sp[0] > Sp[1]) point = 0; else point = 1;
			if (Sp[2] > Sp[point]) point = 2;
		}

		if (point >= 0)
		{
			shown_disjoint = 1;

			VmV(V, S[point], T[0]);
			VcrossV(Z, Tn, Tv[0]);
			if (VdotV(V, Z) > 0)
			{
				VmV(V, S[point], T[1]);
				VcrossV(Z, Tn, Tv[1]);
				if (VdotV(V, Z) > 0)
				{
					VmV(V, S[point], T[2]);
					VcrossV(Z, Tn, Tv[2]);
					if (VdotV(V, Z) > 0)
					{
						VcV(P, S[point]);
						VpVxS(Q, S[point], Tn, Sp[point] / Tnl);


						return sqrt(VdistV2(P, Q));
					}
				}
			}
		}
	}

	if (shown_disjoint)
	{
		VcV(P, minP);
		VcV(Q, minQ);
		return sqrt(mindd);
	}
	else return 0;
}