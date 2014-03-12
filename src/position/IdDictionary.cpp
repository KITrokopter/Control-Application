#include "IdDictionary.hpp"

#include <ros/console.h>

using namespace std;

IdDictionary::IdDictionary()
{
	translated = false;
}

bool IdDictionary::contains(int n)
{
	ROS_DEBUG("contains: getting lock");
	{
		boost::mutex::scoped_lock lock(mutex);
		ROS_DEBUG("contains: got lock");
		
		for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++) {
			if (*it == n) {
				ROS_DEBUG("contains: releasing lock");
				return true;
			}
		}
		
		ROS_DEBUG("contains: releasing lock");
		return false;
	}
}

void IdDictionary::insert(int n)
{
	if (contains(n)) {
		return;
	}
	
	if (translated) {
		ROS_ERROR("insert: Ids already translated!");
	}
	
	ROS_DEBUG("insert: getting lock");
	{
		boost::mutex::scoped_lock lock(mutex);
		ROS_DEBUG("insert: got lock");
		
		ROS_DEBUG("Inserted id %d", n);
		ids.push_back(n);
		
		ROS_DEBUG("insert: released lock");
	}
}

int IdDictionary::size()
{
	ROS_DEBUG("size: getting lock");
	{
		boost::mutex::scoped_lock lock(mutex);
		ROS_DEBUG("size: got lock");
		
		ROS_DEBUG("size: released lock");
		return ids.size();
	}
}

void IdDictionary::translateIds()
{
	if (translated) {
		return;
	}
	
	ROS_DEBUG("translateIds: getting lock");
	{
		boost::mutex::scoped_lock lock(mutex);
		ROS_DEBUG("translateIds: got lock");
		
		ROS_DEBUG("Translating dictionary with %d ids", size());
		
		translated = true;
		
		std::sort(ids.begin(), ids.end());
		int id = 0;
		
		for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++, id++) {
			backward[id] = *it;
			forward[*it] = id;
		}
		
		ROS_DEBUG("translateIds: released lock");
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
	
	ROS_DEBUG("getForward: getting lock");
	{
		boost::mutex::scoped_lock lock(mutex);
		ROS_DEBUG("getForward: got lock");
		
		if (forward.count(n)) {
			int result = forward[n];
			
			ROS_DEBUG("getForward: released lock");
			return result;
		} else {
			ROS_ERROR("getForward: Unknown id %d", n);
			ROS_DEBUG("getForward: released lock");
			return -1;
		}
	}
}

int IdDictionary::getBackward(int n)
{
	if (!translated) {
		ROS_ERROR("getBackward: Ids not translated!");
	}
	
	ROS_DEBUG("getBackward: Locking mutex");
	ROS_DEBUG("forward size: %ld, backward size: %ld, vector size: %ld", forward.size(), backward.size(), ids.size());
	{
		boost::mutex::scoped_lock lock(mutex);
		ROS_DEBUG("getBackward: Mutex locked");
		
		if (backward.count(n)) {
			int result = backward[n];
			
			ROS_DEBUG("getBackward: released lock");
			return result;
		} else {
			ROS_ERROR("getBackward: Unknown id %d", n);
			ROS_DEBUG("getBackward: released lock");
			return -1;
		}
	}
}