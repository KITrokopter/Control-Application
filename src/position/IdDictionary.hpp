#pragma once

#include <map>
#include <vector>
#include <boost/thread/mutex.hpp>

#include "../controller/Mutex.hpp"

using namespace std;

class IdDictionary
{
private:
	Mutex mutex;
	map<int, int> forward;
	map<int, int> backward;
	vector<int> ids;
	
	bool translated;
	
public:
	IdDictionary();
	
	void insert(int n);
	bool contains(int n);
	int size();
	
	void translateIds();
	bool isTranslated();
	
	int getForward(int n);
	int getBackward(int n);
};