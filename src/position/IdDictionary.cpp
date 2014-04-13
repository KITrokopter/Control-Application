#include "IdDictionary.hpp"

#include <ros/console.h>

using namespace std;

/**
 * Creates a new empty dictionary.
 */
IdDictionary::IdDictionary()
{
	translated = false;
}

/**
 * Checks if the dictionary contains the given id of space A.
 *
 * @param n The id to check.
 * @return True if space A contains n, false otherwise.
 */
bool IdDictionary::contains(int n)
{
	{
		boost::mutex::scoped_lock lock(mutex);

		for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++) {
			if (*it == n) {
				return true;
			}
		}

		return false;
	}
}

/**
 * Inserts a new id in space A.
 *
 * @param n The id to insert.
 */
void IdDictionary::insert(int n)
{
	if (contains(n)) {
		return;
	}

	if (translated) {
		ROS_ERROR("insert: Ids already translated!");
	}

	{
		boost::mutex::scoped_lock lock(mutex);

		ROS_DEBUG("Inserted id %d", n);
		ids.push_back(n);
	}
}

/**
 * Returns the size of id space A.
 *
 * @return The size of id space A.
 */
int IdDictionary::size()
{
	{
		boost::mutex::scoped_lock lock(mutex);

		return ids.size();
	}
}

/**
 * Translates the ids from id space A to id space B. This method can be called
 * only once and has to be called before translation works.
 * It takes all ids from space A, sorts them, and assigns ids from space B to
 * them. The lowest id from space A gets the zero in space B,
 * the second lowest id from space A gets one in space B and so on...
 */
void IdDictionary::translateIds()
{
	if (translated) {
		return;
	}

	{
		boost::mutex::scoped_lock lock(mutex);

		ROS_DEBUG("Translating dictionary with %ld ids", ids.size());

		translated = true;

		std::sort(ids.begin(), ids.end());
		int id = 0;

		for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++, id++) {
			backward[id] = *it;
			forward[*it] = id;
		}
	}
}

/**
 * Returns true if the ids were already translated.
 *
 * @return True if the translateIds() function was called, false otherwise.
 */
bool IdDictionary::isTranslated()
{
	return translated;
}

/**
 * Translates the given id from space A to the corresponding id from space B.
 *
 * @param n The id from space A.
 * @return The corresponding id from space B.
 */
int IdDictionary::getForward(int n)
{
	if (!translated) {
		ROS_ERROR("getForward: Ids not translated!");
	}

	{
		boost::mutex::scoped_lock lock(mutex);

		if (forward.count(n)) {
			int result = forward[n];
			return result;
		} else {
			ROS_ERROR("getForward: Unknown id %d", n);
			return -1;
		}
	}
}

/**
 * Translates the given id from space B to the corresponding id from space A.
 *
 * @param n The id from space B.
 * @return The corresponding id from space A.
 */
int IdDictionary::getBackward(int n)
{
	if (!translated) {
		ROS_ERROR("getBackward: Ids not translated!");
	}

	{
		boost::mutex::scoped_lock lock(mutex);

		if (backward.count(n)) {
			int result = backward[n];
			return result;
		} else {
			ROS_ERROR("getBackward: Unknown id %d", n);
			return -1;
		}
	}
}

