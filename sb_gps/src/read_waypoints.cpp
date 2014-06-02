#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "read_waypoints.h"

using namespace std;

// Get the next waypoint: returns n for the nth waypoint, starting at 1
// return -1 for 'no more waypoints'
int GetWaypoint(double goal_x, double goal_y, double* list, int c) 
{
	if (c >= size ) { c = 0; }
	goal_x = list[c];
	goal_y = list[c+1];
	c += 2;
	return (c+1)/2;
}


// Calculates Euclidean distance from position to goal
void CalculateDistance()
{
	x_dist = goal_waypoint.long_x - current_waypoint.long_x;
	y_dist = goal_waypoint.lat_y - current_waypoint.lat_y;
	dist = sqrt(pow(x_dist, 2.0) + pow(y_dist, 2.0));
}	


double* ReturnWaypoints() 
{
	string output;
	int count = 0;
	int array_size;
	double* waypoints_array = NULL;
	ifstream waypoints_file ("/home/jechli/snowbots_ws/src/WaypointsTxt/waypoints.txt");
	if (waypoints_file.is_open())
	{
		while (getline(waypoints_file, output))
		{
			if (count == 0) {
				array_size = atoi(output.c_str())*2;
				waypoints_array = new double [array_size];
				cout << array_size << "\n";
				count++;
			}
			
			else { 
				waypoints_array[count-1] = atof(output.c_str());
				count++;
				cout << waypoints_array[count-1] << "\n"; 
			}
		}
		waypoints_file.close();
	}

	else
	{
		cout << "Unable to open file";
	}

	return waypoints_array;
}


/* Example usage:
int main() {
	double* x = ReturnWaypoints();
	int size = sizeof(x);
	int i = 0;
	cout << size << "\n";
	while ( i < size*2 ) { cout << x[i] << "\n"; i++; }
	return 0;
}
*/

