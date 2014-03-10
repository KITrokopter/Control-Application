#include "IdDictionary.hpp"

#include <ros/console.h>

using namespace std;

IdDictionary::IdDictionary()
{
	translated = false;
}

bool IdDictionary::contains(int n)
{
	mutex.lock();
	
	for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++) {
		if (*it == n) {
			mutex.unlock();
			return true;
		}
	}
	
	mutex.unlock();
	return false;
}

void IdDictionary::insert(int n)
{
	if (contains(n)) {
		return;
	}
	
	if (translated) {
		ROS_ERROR("insert: Ids already translated!");
	}
	
	mutex.lock();
	
	ROS_DEBUG("Inserted id %d", n);
	ids.push_back(n);
	
	mutex.unlock();
}

int IdDictionary::size()
{
	return ids.size();
}

void IdDictionary::translateIds()
{
	if (translated) {
		return;
	}
	
	mutex.lock();
	
	ROS_DEBUG("Translating dictionary with %d ids", size());
	
	translated = true;
	
	std::sort(ids.begin(), ids.end());
	int id = 0;
	
	for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++, id++) {
		backward[id] = *it;
		forward[*it] = id;
	}
	
	mutex.unlock();
}

bool IdDictionary::isTranslated()
{
	return translated;
}

int IdDictionary::getForward(int n)
{
	if (!translated) {
		ROS_ERROR("getForward: Ids not translated!");
	}
	
	mutex.lock();
	
	if (forward.count(n)) {
		mutex.unlock();
		return forward[n];
	} else {
		ROS_ERROR("getForward: Unknown id %d", n);
		mutex.unlock();
		return -1;
	}
}

int IdDictionary::getBackward(int n)
{
	if (!translated) {
		ROS_ERROR("getBackward: Ids not translated!");
	}
	
	mutex.lock();
	
	if (backward.count(n)) {
		mutex.unlock();
		return backward[n];
	} else {
		ROS_ERROR("getBackward: Unknown id %d", n);
		mutex.unlock();
		return -1;
	}
}