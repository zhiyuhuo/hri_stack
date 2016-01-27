#include "ros/ros.h"
#include "hri_spatial_language_grounding/SpatialLanguageGrounding.h"
#include <cstdlib>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "client_test");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<hri_spatial_language_grounding::SpatialLanguageGrounding>("hri_spatial_language_grounding");
	hri_spatial_language_grounding::SpatialLanguageGrounding srv;
	srv.request.str = "the mug is on the table on the right in the bedroom";
	
	if (client.call(srv))
	{
		//vector<string> gds;
		int L = srv.response.strarr.size();
		for (int i = 0; i < L; i++)
		{
			cout << (string)srv.response.strarr[i] << endl;
		}
	}
	else
	{
		ROS_ERROR("Failed to call service SpatialLanguageGrounding");
		return 1;
	}

	return 0;
}
