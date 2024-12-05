#pragma once
#include<vector>
namespace GAlgo {
	template<typename T, typename U>
	class SpacePartitionI {
	public:
		// return elements inside AABBs
		virtual const std::vector<T>& locate_ele(U&& coor) = 0;
		virtual void add_ele(U&& coor) = 0;
		virtual void remove_ele(U&& coor) = 0;
		virtual ~SpacePartitionI() {};
	};
}