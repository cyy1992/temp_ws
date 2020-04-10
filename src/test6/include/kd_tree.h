/*
 * Copyright 2020 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef KDTREE_H
#define KDTREE_H

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
// using point_t = std::vector< double >;
// template <typename T>
struct pose{
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};

using indexArr = std::vector< size_t >;
using pointIndex = typename std::pair< pose, size_t >;

class KDNode {
   public:
    using KDNodePtr = std::shared_ptr< KDNode >;
    size_t index;
    pose x;
    KDNodePtr left;
    KDNodePtr right;
    bool is_empty_;
    bool empty();
    // initializer
    KDNode();
    KDNode(const pose &, const size_t &, const KDNodePtr &,
           const KDNodePtr &);
    KDNode(const pointIndex &, const KDNodePtr &, const KDNodePtr &);
    ~KDNode();

    // getter
    double coord(const size_t &);

    // conversions
    explicit operator bool();
    explicit operator pose();
    explicit operator size_t();
    explicit operator pointIndex();
};

using KDNodePtr = std::shared_ptr< KDNode >;

KDNodePtr NewKDNodePtr();

inline double dist(const pose &, const pose &);
inline double dist(const KDNodePtr &, const KDNodePtr &);

// Need for sorting
class comparer {
   public:
    size_t idx;
    explicit comparer(size_t idx_);
    inline bool compare_idx(
        const std::pair< pose, size_t > &,  //
        const std::pair< pose, size_t > &   //
    );
};

using pointIndexArr = typename std::vector< pointIndex >;

inline void sort_on_idx(const pointIndexArr::iterator &,  //
                        const pointIndexArr::iterator &,  //
                        size_t idx);

using poseVec = std::vector< pose >;

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
    explicit KDTree(poseVec point_array);

   private:
    KDNodePtr nearest_(           //
        const KDNodePtr &branch,  //
        const pose &pt,        //
        const size_t &level,      //
        const KDNodePtr &best,    //
        const double &best_dist   //
    );

    // default caller
    KDNodePtr nearest_(const pose &pt);

   public:
    pose nearest_point(const pose &pt);
    size_t nearest_index(const pose &pt);
    pointIndex nearest_pointIndex(const pose &pt);

   private:
    pointIndexArr neighborhood_(  //
        const KDNodePtr &branch,  //
        const pose &pt,        //
        const double &rad,        //
        const size_t &level       //
    );

   public:
    pointIndexArr neighborhood(  //
        const pose &pt,       //
        const double &rad);

    poseVec neighborhood_points(  //
        const pose &pt,         //
        const double &rad);

    indexArr neighborhood_indices(  //
        const pose &pt,          //
        const double &rad);
    
    int cnt_;
};

#endif // KDTREE_H
