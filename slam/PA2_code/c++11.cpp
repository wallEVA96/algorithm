/*
 * c++11.cpp
 * Copyright (C) 2018 exbot <exbot@ubuntu>
 *
 * Distributed under terms of the MIT license.
 */

#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class A {
public:
	A(const int &i) :index(i){}
	int index = 0;
};

int main(int argc, char **argv){
	A a1(3), a2(5), a3(9);
	vector<A> aVec{a1, a2, a3};
	for( auto &a:aVec )
		cout << a.index << "  ";
	cout << endl;
	std::sort(aVec.begin(), aVec.end(), [](const A&a1, const A&a2){return a1.index > a2.index;} );
	for( auto &a:aVec )
		cout << a.index << "  ";
	cout << endl;


return 0;
}
