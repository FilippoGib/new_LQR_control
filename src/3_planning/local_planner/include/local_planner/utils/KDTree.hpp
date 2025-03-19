/*
 * file: KDTree.hpp
 * author: J. Frederico Carvalho
 *
 * This is an adaptation of the KD-tree implementation in rosetta code
 * https://rosettacode.org/wiki/K-d_tree
 * 
 * It is a reimplementation of the C code using C++. It also includes a few
 * more queries than the original, namely finding all points at a distance
 * smaller than some given distance to a point.
 * 
 */

#ifndef UTILS_KDTREE_HPP
#define UTILS_KDTREE_HPP

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <list>
#include <memory>
#include <set>
#include <unordered_set>
#include <vector>

#include "structures/Point.hpp"

using indexArr = std::vector<size_t>;
using pointIndex = typename std::pair<local_planner::Point, size_t>;

template <class T>
class KDTData {
 private:
  bool valid;
  T value;

 public:
  KDTData() : valid(false) {}
  KDTData(const T &_value) : value(_value), valid(true) {}
  KDTData &operator=(const T &_value) {
    valid = true;
    value = _value;
    return *this;
  }
  explicit operator bool() const { return valid; }
  T &operator*() { return value; }
  T *operator->() { return &value; }
};
using pointIndexV = typename KDTData<pointIndex>::KDTData;

class KDNode {
 public:
  using KDNodePtr = std::shared_ptr<KDNode>;
  KDTData<size_t> index;
  KDTData<local_planner::Point> x;
  KDNodePtr left;
  KDNodePtr right;

  // initializer
  KDNode();
  KDNode(const local_planner::Point &, const size_t &, const KDNodePtr &,
         const KDNodePtr &);
  KDNode(const pointIndex &, const KDNodePtr &, const KDNodePtr &);
  ~KDNode();

  // getter
  double coord(const size_t &);

  // conversions
  explicit operator bool() const;
  explicit operator size_t();
  explicit operator pointIndex();
};

using KDNodePtr = std::shared_ptr<KDNode>;

KDNodePtr NewKDNodePtr();

// square euclidean distance
inline double dist2(const local_planner::Point &, const local_planner::Point &);
inline double dist2(const KDNodePtr &, const KDNodePtr &);

// euclidean distance
inline double dist(const local_planner::Point &, const local_planner::Point &);
inline double dist(const KDNodePtr &, const KDNodePtr &);

// Need for sorting
class comparer {
 public:
  size_t idx;
  explicit comparer(size_t idx_);
  inline bool compare_idx(
      const std::pair<local_planner::Point, size_t> &,  //
      const std::pair<local_planner::Point, size_t> &   //
  );
};

using pointIndexArr = typename std::vector<pointIndex>;

inline void sort_on_idx(const pointIndexArr::iterator &,  //
                        const pointIndexArr::iterator &,  //
                        size_t idx);

using pointVec = std::vector<local_planner::Point>;

class KDTree {
  KDNodePtr root;
  KDNodePtr leaf;

  KDNodePtr make_tree(const pointIndexArr::iterator &begin,  //
                      const pointIndexArr::iterator &end,    //
                      const size_t &length,                  //
                      const size_t &level                    //
  );

 public:
  KDTree() = default;
  explicit KDTree(const pointVec &point_array);
  explicit KDTree(const std::list<local_planner::Point> &point_list);

 private:
  KDNodePtr nearest_(           //
      const KDNodePtr &branch,  //
      const local_planner::Point &pt,          //
      const size_t &level,      //
      const KDNodePtr &best,    //
      const double &best_dist,  //
      const std::set<size_t> &excs) const;

  // default caller
  KDNodePtr nearest_(const local_planner::Point &pt, const std::set<size_t> &excs) const;

 public:
  KDTData<local_planner::Point> nearest_point(const local_planner::Point &pt, const std::set<size_t> &excs = std::set<size_t>()) const;
  KDTData<size_t> nearest_index(const local_planner::Point &pt, const std::set<size_t> &excs = std::set<size_t>()) const;
  pointIndexV nearest_pointIndex(const local_planner::Point &pt, const std::set<size_t> &excs = std::set<size_t>()) const;

 private:
  pointIndexArr neighborhood_(  //
      const KDNodePtr &branch,  //
      const local_planner::Point &pt,          //
      const double &rad,        //
      const size_t &level       //
  ) const;

 public:
  pointIndexArr neighborhood(  //
      const local_planner::Point &pt,         //
      const double &rad) const;

  pointVec neighborhood_points(  //
      const local_planner::Point &pt,           //
      const double &rad) const;

  indexArr neighborhood_indices(  //
      const local_planner::Point &pt,            //
      const double &rad) const;

  std::unordered_set<size_t> neighborhood_indices_set(  //
      const local_planner::Point &pt,            //
      const double &rad) const;
};

#endif  // UTILS_KDTREE_HPP