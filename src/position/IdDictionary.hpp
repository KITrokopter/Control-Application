#pragma once

#include <map>
#include <vector>
#include <boost/thread/mutex.hpp>

#include "../controller/Mutex.hpp"

using namespace std;

/**
 * A dictionary to translate bewteen random and continuous ids, keeping the order of the numbers.<br />
 * For the documentation, we call the number space of the random ids A, and the space of the continuous ids B.
 * 
 * @author Sebastian Schmidt
 */
class IdDictionary
{
private:
	boost::mutex mutex;
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