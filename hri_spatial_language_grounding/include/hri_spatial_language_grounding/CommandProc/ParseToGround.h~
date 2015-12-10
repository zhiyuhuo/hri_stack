#ifndef PARSETOGROUND_H_
#define PARSETOGROUND_H_

#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include "tinyxml2.h"

#define MAXSCORE 100

using namespace tinyxml2;
using namespace std;

class SampleChunk
{
public:
	SampleChunk(){m_uniFurLabel = 0; m_ifModTarget = 0;}
	~SampleChunk(){}

	string m_chunkName;
	vector<string> m_tags;
	vector<string> m_words;
	vector<string> m_groundings;
	vector<float> m_groundingsScore;

	int m_uniFurLabel;
	int m_ifModTarget;
};

class GroundingChunk
{
public:
	GroundingChunk(){}
	~GroundingChunk(){}

	string m_chunkName;
	vector<string> m_tags;
	vector<string> m_words;
	string m_groundingType;
	string m_groundingVariable;
};

class RDTNode
{
public:
	RDTNode(){}
	~RDTNode(){}

	string m_tar;
	vector<string> m_refList;
	vector<string> m_dirList;
};

class ParseToGround
{
public:
	ParseToGround(void);
	~ParseToGround(void);

	string m_targetRoom;
	string m_targetObject;
	vector<string> m_tarList;
	vector<pair<string, string> > m_refdirPair;

	vector<GroundingChunk> m_groundingChunks;
	vector<SampleChunk> m_sampleChunks;

	vector<RDTNode> m_RDTNodeSet;
	string m_targetFurniture;
	int m_uniFurLabel;
	int m_ifModTarget;

public:
	int ParseXml(string xmlFileName, int id);
	int ReadCommandNode(XMLNode* node);
	string GetNodeText(XMLNode* node);
	int SearchDeeperChunk(string chunkName, XMLNode* node);
	int DealWithChunk(XMLNode* node);

	int GenerateGroundingDictionayFromTXT();
	int GenerateAGroundingDictionay(string groundingType, string groundingVariable);

	int LoadGroundingDictionaryFromXml();
	int LoadAGoundingDictionary(string groundingType, string groundingVariable);

	int ParseCommnad();
	SampleChunk ParseAChunk(SampleChunk sample);
	float MatchBetweenASampleAndAGrounding(SampleChunk sample, GroundingChunk grounding);
	float LevenshteinDistance(vector<string> s1, vector<string> s2);

	int BuildCommandGrounding();
}; 

#endif