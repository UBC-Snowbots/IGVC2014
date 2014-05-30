#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "read_waypoints.h"

using namespace std;


// Output: lat/long structure
struct waypoint* ReturnWaypoints()
{
	struct waypoint* points_array;
	string output;
	ifstream waypoints_file ("waypoints.txt");
	if (waypoints_file.is_open())
	{
	  while (getline(waypoints_file, output))
	  {
	    cout << output << "\n";
	    struct waypoint insert;
	    insert.long_x = atof(;
	    insert.lat_y; 
	    
	  }
	  waypoints_file.close();
	}

	else 
	{
	  cout << "Unable to open file";
	  return NULL;
	}
	
}

