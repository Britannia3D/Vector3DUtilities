#include "VectorUtilities.h"

#include <iostream>
#include <sstream>

using namespace std;

//For printing out vector object using a stream
std::ostream & operator<<(std::ostream & os, const Vector3D &f)
{
	std::stringstream ss;
	ss << "(" << f.x << ", " << f.y << ", " << f.z << ")";
	return os << ss.str();
}

Vector3DUtils vecUtils;

int main()
{
	Vector3D vertex1(0.810067, 2.05786, 1.10034);
	Vector3D vertex2(0.140595, 2.06562, 1.10034);
	Vector3D vertex3(0.159295, 2.72633, 1.10034);

	Vector3D point(0.455545, 2.34121, 1.49899);
	Vector3D lineEnd(0.455545, 2.12, 0.777187);
	Vector3D intersect(0, 0, 0);
	bool does = vecUtils.LineTriangleIntersect(point, lineEnd, &vertex1, &vertex2, &vertex3, intersect);

	cout << "Intersect?: " << does << endl;
	cout << intersect.x << " , " << intersect.y << " , " << intersect.z << endl;

	system("PAUSE");

	return 1;
}
