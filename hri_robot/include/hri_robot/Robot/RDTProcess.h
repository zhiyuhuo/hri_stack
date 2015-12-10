#ifndef RDTPROCESS_H
#define RDTPROCESS_H

using namespace std;

int Robot::AskGroundingService(string cmd)
{
	hri_spatial_language_grounding::SpatialLanguageGrounding srv;
	srv.request.str = cmd;
	vector<string> groundings;
	
	cout << "start calling: " << endl;
	if (m_groundingClient.call(srv))
	{
		//vector<string> gds;
		int L = srv.response.strarr.size();
		for (int i = 0; i < L; i++)
		{
			cout << (string)srv.response.strarr[i] << endl;
			groundings.push_back((string)srv.response.strarr[i]);
		}
	}
	else
	{
		ROS_ERROR("Failed to call service SpatialLanguageGrounding");
		return 1;
	}
	cout << "finish calling: " << endl;
	
	vector<RDTNode> rdtList;
	vector<string> labelList;
	RDTNode rdt;
	for (int i = 0; i < groundings.size(); i++)
	{
		std::size_t tr;
		tr = groundings[i].find_last_of(":");
		labelList.push_back(groundings[i].substr(0, tr));
		cout << "label to push: " << groundings[i].substr(0, tr) << endl;
		
		tr = groundings[i].find("target_room:");	
		if (tr != string::npos)
		{
			m_targetRoom = groundings[i].substr(12);
			continue;
		}
		
		tr = groundings[i].find("target_object:");	
		if (tr != string::npos)
		{
			m_targetObject = groundings[i].substr(14);
			continue;
		}
		
		tr = groundings[i].find("tar:");	
		if (tr != string::npos)
		{
			if (labelList[i-1].compare("tar") != 0 && rdtList.size() > 0)
			{
				rdtList.push_back(rdt);
				rdt.m_tar = "";
				rdt.m_refList.clear();
				rdt.m_dirList.clear();
				rdt.m_tar = groundings[i].substr(4);
			}
			else
			{
				if (groundings[i].substr(4).size() > 0)
				{
					rdt.m_tar = groundings[i].substr(4);
					rdt.m_refList.clear();
					rdt.m_dirList.clear();
				}
				else
				{
					rdt.m_refList.clear();
					rdt.m_dirList.clear();					
				}
			}
			continue;
		}
		
		tr = groundings[i].find("ref:");
		if (tr != string::npos)
		{
			rdt.m_refList.push_back(groundings[i].substr(4));
			continue;
		}
		
		tr = groundings[i].find("dir:");	
		if (tr != string::npos)
		{
			rdt.m_dirList.push_back(groundings[i].substr(4));
			continue;
		}
		
	}
	rdtList.push_back(rdt);
	
	m_RDTList = rdtList;
	return 0;
}

void Robot::ShowRobotCmdInfo()
{
	
	cout << "Command Grouding Info: " << endl;
	printf("m_targetObject: %s\n", m_targetObject.c_str());
	printf("m_targetORoom: %s\n", m_targetRoom.c_str());
	cout << "RDT list size: " << m_RDTList.size() << endl;
	for (int i = 0; i < m_RDTList.size(); i++)
	{
		printf("Node %d:\n", i);
		printf("-m_tar: %s\n", m_RDTList[i].m_tar.c_str());
		for (int j = 0; j < m_RDTList[i].m_refList.size(); j++)
		{
			printf("-m_refList[%d]: %s\n", j, m_RDTList[i].m_refList[j].c_str());
			printf("-m_dirList[%d]: %s\n", j, m_RDTList[i].m_dirList[j].c_str());
		}
	}
}

#endif