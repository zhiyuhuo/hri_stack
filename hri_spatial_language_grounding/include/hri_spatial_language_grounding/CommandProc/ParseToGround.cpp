#ifndef PARSETOGROUND_CPP_
#define PARSETOGROUND_CPP_

#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include "ParseToGround.h"

using namespace std;

ParseToGround::ParseToGround(void)
{
	m_spatialCommand = "";
	m_uniFurLabel = 0;
	m_ifModTarget = 0;
}

ParseToGround::~ParseToGround(void)
{
	
}

int ParseToGround::ParseXml(string xmlFileName, int id)
{
	XMLDocument doc;
	int errID = doc.LoadFile(xmlFileName.c_str());
	if (errID != 0)
	{
		printf("errID: %d\n", errID);
		printf("Can not Load File!\n");
		return 0;
	}
	XMLElement* nodes = doc.FirstChildElement();  
	if(nodes == NULL)
	{
		//doc.Clear();
		return 0;
	}
	int t = 1;
	LoadGroundingDictionaryFromXml("/home/hri/hri_DATA/Grounding/");
	LoadGroundingWordWeightsFromTXT("/home/hri/hri_DATA/Grounding_wordweights/");
	//OutputInfo();
	for (XMLNode* note = nodes->FirstChildElement(); note != NULL; note = note->NextSiblingElement())
	{
		string noteName = note->Value();
		if (noteName.compare("S") == 0)
		{
			if (t++ == id)
			{
				ReadCommandNode(note);
				ParseCommnad();
				BuildCommandGrounding();
				break;
			}
		}
	}
	return 1;
}

int ParseToGround::ReadCommandNode(XMLNode* node)
{
	for (XMLNode* elem = node->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
	{
		string elemName = elem->Value();
		if (elemName.length() < 4)
		{
			string str = GetNodeText(elem);
			//printf("\n~%s-%s\n", elemName.c_str(), str.c_str());
		}
		SearchDeeperChunk(elemName, elem);
	}
	return 1;	
}

string ParseToGround::GetNodeText(XMLNode* node)
{
	std::string res;
	for(XMLNode* e = node->FirstChild(); e != NULL; e = e->NextSibling())
	{
		XMLText* text = e->ToText();
		if(text == NULL)
		{	
			continue;
		}
		std::string str = text->Value();
		res = str;
	}	
	return res;
}


int ParseToGround::SearchDeeperChunk(string chunkName, XMLNode* node)
{
	if (chunkName.length() >= 4)
	{
		//printf("%s\n", chunkName.c_str());
		DealWithChunk(node);
	}
	else
	{
		//string str = GetNodeText(node);
		//printf("%s-%s\n", chunkName.c_str(), str.c_str());
	}
	return 0;
}

int ParseToGround::DealWithChunk(XMLNode* node)
{
	string nodeName = node->Value();
	//printf("\n%s\n", nodeName.c_str());
	SampleChunk sampleChunk;
	for (XMLNode* elem = node->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
	{
		string elemName = elem->Value();
		if (elemName.length() < 4)
		{
			string str = GetNodeText(elem);
			//printf("%s-%s ", elemName.c_str(), str.c_str());
			sampleChunk.m_tags.push_back(elemName);
			sampleChunk.m_words.push_back(str);
		}
	}
	sampleChunk.m_chunkName = nodeName;

	if (nodeName.compare("FURTP") == 0)
	{
		m_uniFurLabel++;
		m_ifModTarget = 1;
	}
	sampleChunk.m_uniFurLabel = m_uniFurLabel;
	sampleChunk.m_ifModTarget = m_ifModTarget;

	m_sampleChunks.push_back(sampleChunk);

	for (XMLNode* elem = node->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
	{
		string elemName = elem->Value();
		SearchDeeperChunk(elemName, elem);
	}

	if (nodeName.compare("FURTP") == 0)
	{
		m_ifModTarget = 0;
	}
	return 0;
}

int ParseToGround::GenerateAGroundingDictionay(string rootDir, string groundingType, string groundingVariable)
{
	char fName[100] = {0};
	sprintf(fName, "%s/GroundingTXT/%s-%s.txt", rootDir.c_str(), groundingType.c_str(), groundingVariable.c_str());
	FILE* fid = fopen(fName, "r");
	char c = ' '; 
	string dataStr;
	while (!feof(fid))
	{	
		dataStr.push_back(fgetc(fid));
	}
	fclose(fid);

	string tempStr;
	vector<string> rawStr;
	vector<string> chunktempStr;
	vector<string> chunkNameSet;
	vector<vector<string> > chunkTagSet;
	vector<vector<string> > chunkWordSet;

	for(int i = 0; dataStr[i] != '\0'; i++)
	{
		if (dataStr[i] >= 0)
		{	
			printf("%c %d\n", dataStr[i], dataStr[i]);
			if(isspace(dataStr[i]) || ispunct(dataStr[i]))
			{			
				rawStr.push_back(tempStr);
				tempStr.clear();
			}
			else
			{
				tempStr.push_back(dataStr[i]);
			}
		}
		else
		{
			tempStr.push_back('a');
		}
	}

	for (int i = 0; i < rawStr.size(); i++)
	{
		int n = 0;
		if (rawStr[i+n].size() > 0)
		{
			chunktempStr.push_back(rawStr[i+n]);
			n++;
		}
		else
		{
			chunkNameSet.push_back(chunktempStr[0]);
			vector<string> chunktagtempStr;
			vector<string> chunkwordtempStr;
			for (int j = 1; j < (chunktempStr.size()+1)/2; j++)
			{
				chunktagtempStr.push_back(chunktempStr[2*j-1]);
				chunkwordtempStr.push_back(chunktempStr[2*j]);
			}
			chunkTagSet.push_back(chunktagtempStr);
			chunkWordSet.push_back(chunkwordtempStr);
			chunktempStr.clear();
		}
	}

	XMLDocument doc;
	XMLElement* allNode = doc.NewElement("ALL");

	for (int i = 0; i < chunkNameSet.size(); i++)
	{
		XMLElement* sampleNode = doc.NewElement("sample");
		XMLElement* chunknameElem = doc.NewElement("CHUNKNAME");
		char chunknameStr[100] = {0};
		sprintf(chunknameStr, "%s", chunkNameSet[i].c_str());
		XMLText* chunknametextElem = doc.NewText(chunknameStr);
		chunknameElem->InsertEndChild(chunknametextElem);
		sampleNode->InsertEndChild(chunknameElem);
		for (int j = 0; j < chunkTagSet[i].size(); j++)
		{
			char chunktagStr[100] = {0};
			char chunkwordStr[100] = {0};
			printf("%d %d\n", i ,j);
			string chunktagtempstr = (chunkTagSet[i])[j];
			string chunkwordtempstr = (chunkWordSet[i])[j];
			printf("%s\n", (chunkTagSet[i])[j].c_str());
			printf("%s\n", (chunkWordSet[i])[j].c_str());
			sprintf(chunktagStr, "%s", (chunkTagSet[i])[j].c_str());
			sprintf(chunkwordStr, "%s", (chunkWordSet[i])[j].c_str());
			XMLElement* chunktagElem = doc.NewElement(chunktagStr);
			XMLText* chunkwordElem = doc.NewText(chunkwordStr);
			chunktagElem->InsertEndChild(chunkwordElem);
			sampleNode->InsertEndChild(chunktagElem);
		}
		allNode->InsertEndChild(sampleNode);
	}

	doc.InsertEndChild(allNode);

	char fNameXML[100] = {0};
	sprintf(fNameXML, "%s/Grounding/%s-%s.xml", rootDir.c_str(), groundingType.c_str(), groundingVariable.c_str());
	doc.SaveFile(fNameXML);


	return 1;
}

int ParseToGround::LoadGroundingDictionaryFromXml(string rootDir) {
	
	m_groundingChunks.clear();
	DIR *dp;
	struct dirent *dirp;
	if((dp  = opendir(rootDir.c_str())) == NULL) 
	{
		cout << "Error(" << errno << ") opening " << rootDir << endl;
		return errno;
	}
	vector<string> files;
	while ((dirp = readdir(dp)) != NULL) 
	{
		//cout << string(dirp->d_name) << endl;
		if (string(dirp->d_name).size() > 2 && string(dirp->d_name).find('~') == string::npos)
			files.push_back(string(dirp->d_name));
	}
	closedir(dp);
	string groundingType;
	string groundingVariable;
	for (int i = 0; i < files.size(); i++) 
	{
		//cout << files[i] << endl;
		string file = files[i].substr(0,files[i].size()-4);
		int dashPos = file.find('-');
		//cout << file << " " << dashPos << endl;
		groundingType = file.substr(0,dashPos);
		groundingVariable = file.substr(dashPos+1);
		//cout << "|" << groundingType << "|" << groundingVariable <<"|" << endl;
		LoadAGroundingDictionary(rootDir, groundingType, groundingVariable);
	}
	
	return files.size()-2;
}

int ParseToGround::LoadAGroundingDictionary(string rootDir, string groundingType, string groundingVariable)
{
	//printf("%s - %s\n", groundingType.c_str(), groundingVariable.c_str());
	char fNameXML[100] = {0};
	sprintf(fNameXML, "%s/%s-%s.xml", rootDir.c_str(), groundingType.c_str(), groundingVariable.c_str());

	XMLDocument doc;
	if(doc.LoadFile(fNameXML) != 0)
	{
		printf("%s %s, Er! Can not Load File!\n", groundingType.c_str(), groundingVariable.c_str());
		return 0;
	}
	XMLElement* node = doc.FirstChildElement();
	if(node == NULL)
	{
		return 0;
	}

	for (XMLElement* elem = node->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
	{
		if (elem == NULL)
		{
			return 0;
		}
		string elemName = elem->Value();
		string elem2ChunkName = GetNodeText(elem->FirstChildElement());
		GroundingChunk patternChunk;
		patternChunk.m_chunkName = elem2ChunkName;
		patternChunk.m_groundingType = groundingType;
		patternChunk.m_groundingVariable = groundingVariable;
		
		for (XMLElement* elem2 = elem->FirstChildElement()->NextSiblingElement(); elem2 != NULL; elem2 = elem2->NextSiblingElement())
		{
			string elem2Name = elem2->Value();
			string elem2Text = GetNodeText(elem2);
			patternChunk.m_tags.push_back(elem2Name);
			patternChunk.m_words.push_back(elem2Text);
		}
		m_groundingChunks.push_back(patternChunk);

	}

	return 1;
}

int ParseToGround::LoadGroundingWordWeightsFromTXT(string rootDir)
{
	DIR *dp;
	struct dirent *dirp;
	
	if((dp  = opendir(rootDir.c_str())) == NULL) 
	{
		cout << "Error(" << errno << ") opening " << rootDir << endl;
		return errno;
	}

	vector<string> files;
	while ((dirp = readdir(dp)) != NULL) 
	{
		cout << string(dirp->d_name) << endl;
		if (string(dirp->d_name).size() > 2 && string(dirp->d_name).find('~') == string::npos)
			files.push_back(string(dirp->d_name));
	}
	closedir(dp);
	string groundingType;
	string groundingVariable;
	for (int i = 0; i < files.size(); i++) 
	{
		//cout << files[i] << endl;
		string file = files[i].substr(0,files[i].size()-4);
		int dashPos = file.find('-');
		//cout << file << " " << dashPos << endl;
		groundingType = file.substr(0,dashPos);
		groundingVariable = file.substr(dashPos+1);
		//cout << "|" << groundingType << "|" << groundingVariable <<"|" << endl;
		LoadAGroundingWordWeights(rootDir, groundingType, groundingVariable);
	}
	cout << "m_groundingWordWeights.size(): " << m_groundingWordWeights.size() << endl;
	for(map<string,GroundingWordWeights>::iterator it = m_groundingWordWeights.begin(); it != m_groundingWordWeights.end(); ++it) 
	{
		cout << it->first << "\n";
		map<string,float> w = it->second.m_weights;
		for (map<string,float>::iterator it2 = w.begin(); it2 != w.end(); ++it2)
		{
			cout << it2->first << " " << it2->second << "\n";
		}
	}
	return files.size();
}

int ParseToGround::LoadAGroundingWordWeights(string rootDir, string groundingType, string groundingVariable)
{
	string name = groundingType + "-" + groundingVariable;
	string filename = rootDir + name + ".txt";
	cout << filename << endl;
	ifstream infile(filename.c_str());
	vector<string> lines;
	string line;
	while (getline(infile, line))
	{
		lines.push_back(line);
	}

	vector<int> chunknameNodes;
	for (int i = 0; i < lines.size(); i++)
	{
		string line = lines[i];
		if (line.find(":") != string::npos)
		{
		    chunknameNodes.push_back(i);
		}
	}
	chunknameNodes.push_back(lines.size());
	for (int i = 0; i < chunknameNodes.size()-1; i++)
	{
		GroundingWordWeights g;
		string chunkName = lines[chunknameNodes[i]].substr(0,lines[chunknameNodes[i]].size()-1);
		string key = name + "-" + chunkName;
		g.m_chunkName = chunkName;
		g.m_groundingType = groundingType;
		g.m_groundingVariable = groundingVariable;
		for (int j = chunknameNodes[i]+1; j < chunknameNodes[i+1]; j++)
		{
			string line = lines[j];
			istringstream iss(line);
			string word;
			float weight;
			if (!(iss >> word >> weight)) { break; } // error
			cout << word << " " << weight << endl;
			g.m_weights[word] = weight;
		}
		
		m_groundingWordWeights[key] = g;
	}
	
	return 1;
}

int ParseToGround::ParseCommnad()
{
	for (int i = 0; i < m_sampleChunks.size(); i++)
	{
		m_sampleChunks[i] = ParseAChunk(m_sampleChunks[i]);
	}

	return 0;
}

SampleChunk ParseToGround::ParseAChunk(SampleChunk sample)
{
	SampleChunk res = sample;
	string groundingTypeList[5] = {"target room", "target object", "reference", "direction", "target"};
	float bestScoreList[5] = {MAXSCORE, MAXSCORE, MAXSCORE, MAXSCORE, MAXSCORE}; //m_targetRoom, m_targetObject, m_ref, m_dir, m_tar
	int bestScoreGrounding[5] = {0};
	for (int i = 0; i < m_groundingChunks.size(); i++)
	{
		//float score = MatchBetweenASampleAndAGrounding(sample, m_groundingChunks[i]);
		float score = 1 - WeightedMatchBetweenASampleAndAGrounding(sample, m_groundingChunks[i]);
		//cout << "score: " << score << endl;
		for (int j = 0; j < 5; j++)
		{
			if (m_groundingChunks[i].m_groundingType.compare(groundingTypeList[j]) == 0 && score <= bestScoreList[j])
			{
				bestScoreList[j] = score;
				bestScoreGrounding[j] = i;
			}
		}
	}

	for (int i = 0; i < 5; i++)
	{
		res.m_groundings.push_back(m_groundingChunks[bestScoreGrounding[i]].m_groundingVariable);
		res.m_groundingsScore.push_back(bestScoreList[i]);
	}
	
	return res;
}

float ParseToGround::MatchBetweenASampleAndAGrounding(SampleChunk sample, GroundingChunk grounding)
{
	float res = MAXSCORE;
	if (sample.m_chunkName.compare(grounding.m_chunkName) != 0)
	{
		return MAXSCORE;
	}

	vector<string> sWords = sample.m_words;
	vector<string> sTags = sample.m_tags;
	vector<string> gWords = grounding.m_words;
	vector<string> gTags = grounding.m_tags;
	
	float ld = LevenshteinDistance(sWords, gWords);
	res = ld;
	return res;
}

float ParseToGround::LevenshteinDistance(vector<string> s1, vector<string> s2)
{
	float res = 0;
	const size_t len1 = s1.size(), len2 = s2.size();
	vector<vector<int> > d(len1 + 1, vector<int>(len2 + 1));

	d[0][0] = 0;
	for(int i = 1; i <= len1; ++i) 
	{
		d[i][0] = i;
	}
	for(int i = 1; i <= len2; ++i) 
	{
		d[0][i] = i;
	}

	for(int i = 1; i <= len1; ++i)
	{
		for(int j = 1; j <= len2; ++j)
		{
			d[i][j] = min( min(d[i-1][j]+1, d[i][j-1]+1), d[i-1][j-1]+(!s1[i-1].compare(s2[j-1]) ? 0 : 1) );
		}
	}

	res = d[len1][len2];
	return res;
}

float ParseToGround::WeightedMatchBetweenASampleAndAGrounding(SampleChunk sample, GroundingChunk grounding)
{	
	float res = 0;
	if (sample.m_chunkName.compare(grounding.m_chunkName) != 0)
	{
		return 0;
	}
	vector<string> sWords = sample.m_words;
	vector<string> gWords = grounding.m_words;
	string key = grounding.m_groundingType + "-" + grounding.m_groundingVariable + "-" + grounding.m_chunkName;
	//cout << "key: " << key << endl;
	GroundingWordWeights groundingWeights;
	if (m_groundingWordWeights.find(key) != m_groundingWordWeights.end())
	{
		groundingWeights = m_groundingWordWeights[key];
	}
	
// 	map<string,float> wm = groundingWeights.m_weights;
// 	for (map<string,float>::iterator it = wm.begin(); it != wm.end(); ++it)
// 	{
// 		cout << it->first << " " << it->second << endl;
// 	}

	vector<float> sWeights(sWords.size(), 0.01);
	vector<float> gWeights(gWords.size(), 0.01);
	for (int i = 0; i < sWords.size(); i++)
	{
		if (groundingWeights.m_weights.find(sWords[i]) != groundingWeights.m_weights.end())
		{
			sWeights[i] = groundingWeights.m_weights[sWords[i]];
		}
	}
	
	for (int i = 0; i < gWords.size(); i++)
	{
		if (groundingWeights.m_weights.find(gWords[i]) != groundingWeights.m_weights.end())
		{
			gWeights[i] = groundingWeights.m_weights[gWords[i]];
		}
	}
	
// 	cout << "grounding: ";
// 	for (int i = 0; i < gWeights.size(); i++)
// 	{	cout << gWords[i] << " " << gWeights[i] << ", ";} cout << endl;
// 	cout << "sample: ";
// 	for (int i = 0; i < sWeights.size(); i++)
// 	{	cout << sWords[i] << " " << sWeights[i] << ", ";} cout << endl;
	
	res = WeightedLevenshteinDistance(sWords, gWords, sWeights, gWeights);

	return res; 
}

float ParseToGround::WeightedLevenshteinDistance(vector<string> s1, vector<string> s2, vector<float> w1, vector<float> w2)
{
	float res = 0;
	const size_t len1 = s1.size(), len2 = s2.size();
	vector<vector<int> > L(len1 + 1, vector<int>(len2 + 1, 0));
	w1.insert(w1.begin(), 0);
	w2.insert(w2.begin(), 0);
	float g = 1;
	float m = 0;
	float d = 1;
	L[0][0] = 0;
	for(int i = 0; i <= len1; ++i) 
	{
		L[i][0] = i * w1[i];
	}
	for(int i = 0; i <= len2; ++i) 
	{
		L[0][i] = i * w2[i];
	}
	for(int i = 1; i <= len1; ++i)
	{
		for(int j = 1; j <= len2; ++j)
		{
			float score = 0;
			if (s1[i-1].compare(s2[j-1]) == 0 )
			{
				score = m;
			}
			else
			{
				score = max(w1[i-1], w2[j-1]);
			}
			float m1 = L[i-1][j-1] + score;
			float m2 = L[i-1][j] + (len1+1) * w1[i-1];
                        float m3 = L[i][j-1] + (len2+1) * w2[j-1];
			L[i][j] = min(m1, min(m2, m3));
		}
	}
	res = L[len1][len2];
	return res;
}

int ParseToGround::BuildCommandGrounding()
{
	//target room
	float targetRoomBestScore = MAXSCORE;
	int targetRoomBestScoreSample = -1;
	for (int i = 0; i < m_sampleChunks.size(); i++)
	{
		SampleChunk sample = m_sampleChunks[i];
		if (sample.m_groundingsScore[0] < MAXSCORE)
		{
			if (sample.m_groundingsScore[0] < targetRoomBestScore)
			{
				targetRoomBestScore = sample.m_groundingsScore[0];
				m_targetRoom = sample.m_groundings[0];
				targetRoomBestScoreSample = i;
			}
		}
	}
	if (targetRoomBestScoreSample >= 0)
		m_sampleChunks.erase(m_sampleChunks.begin() + targetRoomBestScoreSample);
	//target object
	float targetObjectBestScore = MAXSCORE;
	int targetObjectBestScoreSample = -1;
	for (int i = 0; i < m_sampleChunks.size(); i++)
	{
		SampleChunk sample = m_sampleChunks[i];
		if (sample.m_groundingsScore[1] < MAXSCORE)
		{
			if (sample.m_groundingsScore[1] < targetObjectBestScore)
			{
				targetObjectBestScore = sample.m_groundingsScore[1];
				m_targetObject = sample.m_groundings[1];
				targetObjectBestScoreSample = i;
			}
		}
	}
	m_sampleChunks.erase(m_sampleChunks.begin() + targetObjectBestScoreSample);
	//Delete Nosense node
	for (int i = 0; i < m_sampleChunks.size(); i++)
	{
		SampleChunk s = m_sampleChunks[i];
		if (s.m_groundingsScore[2] == MAXSCORE 
			&& s.m_groundingsScore[3] == MAXSCORE
			&& s.m_groundingsScore[4] == MAXSCORE)
		{
			m_sampleChunks.erase(m_sampleChunks.begin() + i);
		}

	}
	//RDT
	vector<RDTNode> rdtNodeChain;
	for (int i = 0; i < m_sampleChunks.size(); i++)
	{
		SampleChunk s = m_sampleChunks[i];
		if (s.m_groundingsScore[2] != MAXSCORE 
			|| s.m_groundingsScore[3] != MAXSCORE
			|| s.m_groundingsScore[4] != MAXSCORE)
		{
			RDTNode rdt;
			if (s.m_groundingsScore[2] != MAXSCORE)
			{
				rdt.m_refList.push_back(s.m_groundings[2]);
			}
			if (s.m_groundingsScore[3] != MAXSCORE)
			{
				rdt.m_dirList.push_back(s.m_groundings[3]);
			}
			if (s.m_groundingsScore[4] != MAXSCORE)
			{
				rdt.m_tar = s.m_groundings[4];
			}			
			rdtNodeChain.push_back(rdt);
		}
		
	}
	m_RDTNodeSet = rdtNodeChain;
	int n = 0;
	int t = 1;
	while (n < m_RDTNodeSet.size())
	{
 		if (m_RDTNodeSet[n].m_tar.size() > 0)
		{
			while(n+1 < m_RDTNodeSet.size() && m_sampleChunks[n+1].m_ifModTarget != 0 && m_sampleChunks[n+1].m_uniFurLabel ==  m_sampleChunks[n].m_uniFurLabel)
			{
				m_RDTNodeSet[n].m_refList.push_back(m_RDTNodeSet[n+t].m_refList[0]);
				m_RDTNodeSet[n].m_dirList.push_back(m_RDTNodeSet[n+t].m_dirList[0]);
				m_RDTNodeSet.erase(m_RDTNodeSet.begin() + 1);
				m_sampleChunks.erase(m_sampleChunks.begin() + 1);
			}
			n += 1;
		}
		else
		{
			n++;
		}
	}
	ReGroupRDTNodes();

	return 0;
}

int ParseToGround::ReGroupRDTNodes()
{
	int n = 0; 
	vector<RDTNode>::iterator it = m_RDTNodeSet.begin();
	while (it+1 != m_RDTNodeSet.end()) 
	{
		RDTNode nd = *it;
		RDTNode nd1 = *(it+1);
		if (nd.m_tar != ""
		 && nd.m_refList.size() == 0
		 && nd.m_dirList.size() == 0
		 && nd1.m_tar == ""
		 && nd1.m_refList.size() > 0
		 && nd1.m_refList[0] != "move"
		)
		{
			it->m_refList = nd1.m_refList;
			it->m_dirList = nd1.m_dirList;
			m_RDTNodeSet.erase(it+1);
		}
		else
		{
			it++;
		}
	}
	
	return m_RDTNodeSet.size();
}

int ParseToGround::OutputInfo()
{
	cout << "m_groundingChunks.size(): " << m_groundingChunks.size() << endl;
	for (int i = 0; i < m_groundingChunks.size(); i++) 
	{
		cout << "i: " << i << endl;
		cout << m_groundingChunks[i].m_groundingType << " " << m_groundingChunks[i].m_groundingVariable << " " << m_groundingChunks[i].m_chunkName << endl;
	}
	return 0;
}

#endif