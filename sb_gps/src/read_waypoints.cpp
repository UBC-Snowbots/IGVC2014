#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main() {

	string output;
	ifstream waypoints_file ("waypoints.txt");
	if (waypoints_file.is_open())
	{
	  while (getline(waypoints_file, output))
	  {
	    cout << output << "\n";
	  }
	  waypoints_file.close();
	}

	else 
	{
	  cout << "Unable to open file";
	}

	return 0;
}
