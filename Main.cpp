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
	system("PAUSE");

	return 1;
}
