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
	Vector3D vector1(1, 1, 0);
	Vector3D vector2(5, 1, 0);

	Vector3D vector3 = vecUtils.displaceVectorTowards(vector1, vector2, 7);

	cout << vector3 << endl;

	/*
	Vector3D theVector(0, 0, 0);//Instantiate the vector class

	//Set values directly
	theVector.x = 1;
	theVector.y = 1;
	theVector.z = 1;

	//Multiply operator
	theVector *= 5;

	cout << theVector << endl;//Print out using stream

	theVector.x += 5;//Add 5 to x

	cout << theVector << endl;

	//Dot product of two vectors
	Vector3D newVector(1, 2, 3);
	cout << vecUtils.dot(theVector, newVector) << endl;

	system("PAUSE");
	*/

	return 1;
}