#include "IdDictionary.hpp"

#include <ros/console.h>

using namespace std;

IdDictionary::IdDictionary()
{
	translated = false;
}

bool IdDictionary::contains(int n)
{
	boost::mutex::scoped_lock lock(mutex);
	
	for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++) {
		if (*it == n) {
			return true;
		}
	}
	
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
	
	boost::mutex::scoped_lock lock(mutex);
	
	ROS_DEBUG("Inserted id %d", n);
	ids.push_back(n);
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
	
	boost::mutex::scoped_lock lock(mutex);
	
	ROS_DEBUG("Translating dictionary with %d ids", size());
	
	translated = true;
	
	std::sort(ids.begin(), ids.end());
	int id = 0;
	
	for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++, id++) {
		backward[id] = *it;
		forward[*it] = id;
	}
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
	
	boost::mutex::scoped_lock lock(mutex);
	
	if (forward.count(n)) {
		int result = forward[n];
		return result;
	} else {
		ROS_ERROR("getForward: Unknown id %d", n);
		return -1;
	}
}

int IdDictionary::getBackward(int n)
{
	if (!translated) {
		ROS_ERROR("getBackward: Ids not translated!");
	}
	
	ROS_DEBUG("getBackward: Locking mutex");
	boost::mutex::scoped_lock lock(mutex);
	ROS_DEBUG("getBackward: Mutex locked");
	
	if (backward.count(n)) {
		int result = backward[n];
		return result;
	} else {
		ROS_ERROR("getBackward: Unknown id %d", n);
		return -1;
	}
}