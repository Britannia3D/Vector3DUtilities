#include "JointHeader.h"

#include <iostream>
#include <sstream>

using namespace std;

int main()
{
	//Two triangles
	Vector3D p1A(-0.535202, -0.825005, 0.258526);
	Vector3D p2A(0, -0.355825, -0.60174);
	Vector3D p3A(-1.0035, -0.353941, -0.605195);

	Vector3D p1B(-0.535202, -0.024653, 0.711492);
	Vector3D p2B(0, 0.45408, -0.14349);
	Vector3D p3B(-1.0035, 0.45601, -0.146924);

	Vector3DUtils vecUtils;

	float IntersectPointA[3];
	float IntersectPointB[3];

	//Return closest triangle distance
	float tDist = vecUtils.triangleDistance(IntersectPointA, IntersectPointB, p1A, p2A, p3A, p1B, p2B, p3B);

	cout << tDist << endl;
	system("PAUSE");

	return 1;
}
