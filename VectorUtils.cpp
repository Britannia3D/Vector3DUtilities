#include "UniHeader.h"

using namespace std;

int VectorUtils::angleOf2DVector(Vector3D a)
{
	int angle = atan2(a.y, a.x) * 57.2958;
	return angle;
}

bool VectorUtils::pointOnLine_2(Vector3D a, Vector3D b, Vector3D point)
{
	float x0 = a.x;
	float y0 = a.y;
	float z0 = a.z;

	float x1 = b.x;
	float y1 = b.y;
	float z1 = b.z;

	float x = point.x;
	float y = point.y;
	float z = point.z;

	float dx = x1 - x0;
	float dy = y1 - y0;
	float dz = z1 - z0;

	float ex = x - x0;
	float ey = y - y0;
	float ez = z - z0;

	float q = dx * ex;
	q += dy * ey;
	q += dz * ez;
	q *= q;
	q /= (dx * dx + dy * dy + dz * dz);
	q /= (ex * ex + ey * ey + ez * ez);

	if (q >= 1.0 - 1e-10)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//3D point to spherical coordinates
Vector3D VectorUtils::cartesanToSpherical(float x, float y, float z)
{
	float radius = vecUtils.length(Vector3D(x, y, z));
	float theta = atan2(sqrt(x * x + y * y), z);
	float phi = atan2(y, x);

	return Vector3D(theta, phi, radius);
}

//Sherical coordinates and radius to 3D point
Vector3D VectorUtils::sphericalToCartesan(float theta, float phi, float radius)
{
	float x = radius * cos(phi) * sin(theta);
	float y = radius * sin(phi) * sin(theta);
	float z = radius * cos(theta);

	return Vector3D(x, y, z);
}


//Set fractional amount from a to b
Vector3D VectorUtils::lerp3D(Vector3D a, Vector3D b, float scale)
{
	Vector3D op0;
	//End-Start
	op0.x = b.x - a.x;
	op0.y = b.y - a.y;
	op0.z = b.z - a.z;
	op0 *= scale;

	Vector3D op1;
	op1.x = a.x + op0.x;
	op1.y = a.y + op0.y;
	op1.z = a.z + op0.z;

	return(op1);
}

//Euclidean distance
float VectorUtils::dist(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return(sqrt(pow((float)(x2 - x1), (float)2) + pow((float)(y2 - y1), (float)2) + pow((float)(z2 - z1), (float)2)));
}

float VectorUtils::dist(Vector3D a, Vector3D b)
{
	return(sqrt(pow((float)(b.x - a.x), (float)2) + pow((float)(b.y - a.y), (float)2) + pow((float)(b.z - a.z), (float)2)));
}

//Cross product of two vectors
Vector3D VectorUtils::cross(const Vector3D& A, const Vector3D& B)
{
	Vector3D crossP(0, 0, 0);
	crossP.x = A.y * B.z - A.z * B.y;
	crossP.y = A.z * B.x - A.x * B.z;
	crossP.z = A.x * B.y - A.y * B.x;

	return crossP;
}

Vector3D VectorUtils::arbitraryOrthogonal(Vector3D vec)
{
	bool b0 = (vec.x < vec.y) && (vec.x < vec.z);
	bool b1 = (vec.y <= vec.x) && (vec.y < vec.z);
	bool b2 = (vec.z <= vec.x) && (vec.z <= vec.y);

	Vector3D op(0, 0, 0);
	op = cross(vec, Vector3D(int(b0), int(b1), int(b2)));

	return op;
}

float VectorUtils::angularDifference(Vector3D a, Vector3D b)
{
	float angle = acos(dot(a, b) / (length(a)*length(b)));
	return angle;
}

//Angular difference 2D points. Tested working.
float VectorUtils::angularDifference2D(float p1x, float p1y, float p2x, float p2y)
{
	return atan2(p1x - p2x, p1y - p2y);
}

//The intersect point lies within the two line points
bool VectorUtils::pointOnLine_3(Vector3D lineStart, Vector3D lineEnd, Vector3D point)
{
	float lineDist = utils.dist(lineStart.x, lineStart.y, lineStart.z, lineEnd.x, lineEnd.y, lineEnd.z);

	float d1 = utils.dist(point.x, point.y, point.z, lineEnd.x, lineEnd.y, lineEnd.z);
	float d2 = utils.dist(point.x, point.y, point.z, lineStart.x, lineStart.y, lineStart.z);

	//Get furthest
	if (d1 > d2)
	{
		if (d1 <= lineDist)
		{
			return true;
		}
	}

	if (d2 > d1)
	{
		if (d2 <= lineDist)
		{
			return true;
		}
	}

	return false;
}

Vector2D VectorUtils::orthogonalVector2D_CW(Vector2D a, Vector2D b)
{
	return Vector2D(a.y, -b.x);
}

Vector2D VectorUtils::orthogonalVector2D_CCW(Vector2D a, Vector2D b)
{
	return Vector2D(-a.y, b.x);
}


//Only does an infinite line
Vector3D VectorUtils::closestPointAlongLine(Vector3D lineFrom, Vector3D lineTo, Vector3D c)
{
	//Convert to unit vector
	Vector3D c0 = (lineFrom - lineTo);
	float l = vecUtils.length(c0);
	Vector3D u = Vector3D(c0.x / l, c0.y / l, c0.z / l);

	Vector3D v = c - lineFrom;
	float w = vecUtils.dot(u, v);
	Vector3D r((u.x * w), (u.y * w), (u.z * w));
	return lineFrom + r;
};

double VectorUtils::pointDistToLine2D(Vector2D A, Vector2D B, Vector2D point)
{
	Vector2D AB;
	AB.x = B.x - A.x;
	AB.y = B.y - A.y;

	Vector2D BE;
	BE.x = point.x - B.x;
	BE.y = point.y - B.y;

	Vector2D AE;
	AE.x = point.x - A.x, AE.y = point.y - A.y;

	double AB_BE, AB_AE;

	AB_BE = (AB.x * BE.x + AB.y * BE.y);
	AB_AE = (AB.x * AE.x + AB.y * AE.y);

	double output = 0;

	if (AB_BE > 0)
	{
		double y = point.y - B.y;
		double x = point.x - B.x;
		output = sqrt(x * x + y * y);
	}
	else if (AB_AE < 0)
	{
		double y = point.y - A.y;
		double x = point.x - A.x;
		output = sqrt(x * x + y * y);
	}
	else
	{
		double x1 = AB.x;
		double y1 = AB.y;
		double x2 = AE.x;
		double y2 = AE.y;
		double mod = sqrt(x1 * x1 + y1 * y1);
		output = abs(x1 * y2 - y1 * x2) / mod;
	}

	return output;
}

Vector3D VectorUtils::closestPlanePoint(Vector3D pointPosition, Vector3D planePosition, Vector3D planeNormal)
{
	float sb, sn, sd;

	Vector3D d1 = pointPosition - planePosition;
	sn = -vecUtils.dot(planeNormal, d1);
	sd = vecUtils.dot(planeNormal, planeNormal);

	sb = sn / sd;

	Vector3D result = pointPosition + (planeNormal * sb);

	return result;
}

//Move point a towards b, return c as output
Vector3D VectorUtils::displaceVectorTowards(Vector3D a, Vector3D b, float amount)
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

	Vector3D c(0, 0, 0);
	c.x = a.x + op3.x;
	c.y = a.y + op3.y;
	c.z = a.z + op3.z;

	return(c);
}

double VectorUtils::dot(Vector3D input1, const Vector3D& input2)
{
	return input1.x * input2.x + input1.y * input2.y + input1.z * input2.z;
}

//Set the length (magnitude) of a given vector
Vector3D VectorUtils::setVectorMagnitude(Vector3D input, float newMag)
{
	float mag = sqrt(input.x * input.x + input.y * input.y + input.z * input.z);

	float new_x = input.x * newMag / mag;
	float new_y = input.y * newMag / mag;
	float new_z = input.z * newMag / mag;

	Vector3D op(new_x, new_y, new_z);
	return op;
}

//Same as length
double VectorUtils::speed(Vector3D vec)
{
	return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

double VectorUtils::length(Vector3D vec)
{
	return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

Vector3D VectorUtils::normalize(Vector3D vec)
{
	float op1 = pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2);
	op1 = sqrt(op1);

	Vector3D op;
	op.x = vec.x / op1;
	op.y = vec.y / op1;
	op.z = vec.z / op1;

	return op;
}

float VectorUtils::sign(Vector3D p1, Vector3D p2, Vector3D p3)
{
	return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

//2D point within triangle
bool VectorUtils::insideTriangle2D(Vector3D pt, Vector3D v1, Vector3D v2, Vector3D v3)
{
	float d1, d2, d3;
	bool has_neg, has_pos;

	d1 = sign(pt, v1, v2);
	d2 = sign(pt, v2, v3);
	d3 = sign(pt, v3, v1);

	has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
	has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

	return !(has_neg && has_pos);
}

//Möller–Trumbore intersection algorithm. Fastest.
bool VectorUtils::rayTriangleIntersect(Vector3D rayOrigin, Vector3D rayVector, Vector3D* v1, Vector3D* v2, Vector3D* v3, Vector3D& outIntersectionPoint)
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
