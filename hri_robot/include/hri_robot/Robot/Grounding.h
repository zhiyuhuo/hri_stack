#ifndef ROBOT_GROUNDING_H
#define ROBOT_GROUNDING_H

using namespace std;

void Robot::cmdstrCallback(const std_msgs::StringConstPtr& msg)
{
	m_spatialCommand = msg->data;
}

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

int Robot::BuildGroundingList(vector<string> groundings)
{
	vector<RDTNode> rdtList;
	vector<string> labelList;
	RDTNode rdt;
	for (int i = 0; i < groundings.size(); i++)
	{
		cout << "groundings " << i << ": " << groundings[i] << endl;
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

int Robot::FormatGroundings()
{
	//transfer m_targetObject, m_targetRoom, m_RDTList to string_format groundings.
	m_groundings.clear();
	
	if (m_targetRoom.size() > 1)
	{
		vector<string> groundingObject(5, "non");
		groundingObject[0] = m_targetRoom;
		m_groundings.push_back(groundingObject);
	}
	
	if (m_targetObject.size() > 1)
	{
		vector<string> groundingObject(5, "non");
		groundingObject[1] = m_targetObject;
		m_groundings.push_back(groundingObject);
	}
	
	for (int i = 0; i < m_RDTList.size(); i++)
	{
		RDTNode node = m_RDTList[i];
		if (node.m_refList.size() != 0)
		{
			if (node.m_refList[0].size() > 1 && node.m_refList[0] != "non"
			    && node.m_dirList[0].size() > 1 && node.m_dirList[0] != "non"
			)
			{
				vector<string> groundingObject(5, "non");
				groundingObject[2] = node.m_refList[0];
				groundingObject[3] = node.m_dirList[0];
				if (node.m_tar.size() > 1 && node.m_tar != "non")
					groundingObject[4] = node.m_tar;
				m_groundings.push_back(groundingObject);				
			}
		}
	}
	
	for (int i = 0; i < m_groundings.size(); i++)
	{
		cout << "cmd " << i << ": ";
		for (int j = 0; j < m_groundings[i].size(); j++)
		{
			cout << m_groundings[i][j] << " ";
		}
		cout << endl;
	}
	
	return m_groundings.size();
}

void Robot::ShowRobotCmdInfo()
{
	
	cout << "Command Grouding Info: " << endl;
	printf("m_targetObject: %s\n", m_targetObject.c_str());
	printf("m_targetRoom: %s\n", m_targetRoom.c_str());
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

int Robot::BuildFakeGroundingList()
{
	m_groundings.clear();
	
	const char* argsObject[] = {"non", "mug", "non", "non", "non"};
	vector<string> groundingObject(argsObject, argsObject + 5);
	//m_groundings.push_back(groundingObject);
	
	const char* argsRoom[] = {"livingroom", "non", "non", "non", "non"};
	vector<string> groundingRoom(argsRoom, argsRoom + 5);
	//m_groundings.push_back(groundingRoom);

	const char* argsRDT1[] = {"non", "non", "move", "front", "non"};
	vector<string> groundingRDT1(argsRDT1, argsRDT1 + 5);
	m_groundings.push_back(groundingRDT1);
	
    /*
	const char* argsRDT2[] = {"non", "non", "move", "right", "non"};
	vector<string> groundingRDT2(argsRDT2, argsRDT2 + 5);
	//m_groundings.push_back(groundingRDT2);

	const char* argsRDT3[] = {"non", "non", "couch", "front", "table"};
	vector<string> groundingRDT3(argsRDT3, argsRDT3 + 5);
	m_groundings.push_back(groundingRDT3);	
    */
    
	return 0;
}


#endif