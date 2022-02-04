#include <stdio.h>
#include <string>

using namespace std;


bool isPathExist(const std::string& s)
{
	struct stat buffer;
	return (stat(s.c_str(), &buffer) == 0);
}