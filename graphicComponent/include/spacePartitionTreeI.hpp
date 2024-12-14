#pragma once
#include "Triangle.hpp"
#include "spacePartitionI.hpp"
#include "types.hpp"
#include <functional>
#include <queue>
#include <tuple>

namespace GAlgo {
// tree structure of space partition
template <typename T, typename U>
class SpacePartitionTree : public SpacePartitionI {
public:
  // may use a cache friendly struct. optimization if necessary.
  struct TreeNode;
  using TreeNodeP = std::shared_ptr<TreeNode>;
  struct TreeNode {
    std::vector<T> data;
    TreeNodeP left;
    TreeNodeP right;
    AABB aabb;
  };
  // algo return left node's vec and right node's vec
  using PARITION_ALGO =
      std::function<std::tuple<std::vector<T>, std::vector<T>>(TreeNodes *)>;
  using LOCATE_ALGO = std::function<bool(TreeNodeP)>;

protected:
  int aabb_ele_size = 0;
  TreeNodeP root;
  PARITION_ALGO partition_f;
  LOCATE_ALGO locate_f;

public:
  virtual SpacePartitionTreeI(int aabb_ele_size, PARITION_ALGO partition_f,
                              LOCATE_ALGO locate_f)
      : aabb_ele_size(aabb_ele_size), partition_f(partition_f),
        locate_f(locate_f){};
  // return elements inside AABBs
  virtual const std::vector<T> &locate_ele(U &&coor) = 0;
  virtual void add_ele(U &&coor) override {
    std::queue<TreeNodeP> queue;
    queue.push(root);
    while (!queue.empty()) {
      TreeNodeP current = queue.front();
      queue.pop();
      locate_f(current);
    }
  }
  virtual void remove_ele(U &&coor) = 0;
  virtual ~SpacePartitionI(){};
};

using TriSpacePartitionTree = SpacePartitionTree<Triangle &, POINT_EGDE_3D>;
} // namespace GAlgo