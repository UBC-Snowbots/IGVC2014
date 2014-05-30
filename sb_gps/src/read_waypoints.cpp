#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "read_waypoints.h"

using namespace std;


// Output: lat/long structure
struct waypoint* ReturnWaypoints()
{
	struct waypoint* points_array = NULL;
	string output;
	int count = 0;
	ifstream waypoints_file ("waypoints.txt");

	if (waypoints_file.is_open())
	{
	  while (getline(waypoints_file, output))
	  {
	    struct waypoint insert;
	    int index = output.find(","); // delimiter for lat,long
	    const char* y = (output.substr(0,index-1)).c_str(); 
	    const char* x = (output.substr(index+1)).c_str(); 
	    insert.long_x = atof(x);
	    insert.lat_y = atof(y);
	    *(points_array+count) = insert;
	    count++;
	  }
	  waypoints_file.close();
	}

	else 
	{
	  cout << "Unable to open file";

	}
	return points_array;
}

