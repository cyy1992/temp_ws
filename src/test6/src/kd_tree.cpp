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

#include "kd_tree.h"


#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>

const size_t& size_of_dimension = 2;
KDNode::KDNode():is_empty_(true){};

KDNode::KDNode(const pose &pt, const size_t &idx_, const KDNodePtr &left_,
               const KDNodePtr &right_):is_empty_(false) {
    x = pt;
    index = idx_;
    left = left_;
    right = right_;
}

KDNode::KDNode(const pointIndex &pi, const KDNodePtr &left_,
               const KDNodePtr &right_):is_empty_(false) {
    x = pi.first;
    index = pi.second;
    left = left_;
    right = right_;
}

KDNode::~KDNode() = default;

double KDNode::coord(const size_t &idx) { return x.p(idx); }
KDNode::operator bool() { return (!empty()); }
KDNode::operator pose() { return x; }
KDNode::operator size_t() { return index; }
KDNode::operator pointIndex() { return pointIndex(x, index); }
bool KDNode::empty()
{
  return is_empty_;
}

KDNodePtr NewKDNodePtr() {
    KDNodePtr mynode = std::make_shared< KDNode >();
    return mynode;
}

inline double dist2(const pose &a, const pose &b) {
    double distc = 0;
    for (size_t i = 0; i < size_of_dimension; i++) {
        double di = a.p(i) - b.p(i);
        distc += di * di;
    }
    return distc;
}

inline double dist2(const KDNodePtr &a, const KDNodePtr &b) {
    return dist2(a->x, b->x);
}

comparer::comparer(size_t idx_) : idx{idx_} {};

inline bool comparer::compare_idx(const pointIndex &a,  //
                                  const pointIndex &b   //
) {
    return (a.first.p(idx) < b.first.p(idx));  //
}

inline void sort_on_idx(const pointIndexArr::iterator &begin,  //
                        const pointIndexArr::iterator &end,    //
                        size_t idx) {
    comparer comp(idx);
    comp.idx = idx;

    using std::placeholders::_1;
    using std::placeholders::_2;

    std::sort(begin, end, std::bind(&comparer::compare_idx, comp, _1, _2));
}

using poseVec = std::vector< pose >;

KDNodePtr KDTree::make_tree(const pointIndexArr::iterator &begin,  //
                            const pointIndexArr::iterator &end,    //
                            const size_t &length,                  //
                            const size_t &level                    //
) {
    if (begin == end) {
        return NewKDNodePtr();  // empty tree
    }

    if (length > 1) {
        sort_on_idx(begin, end, level);
    }

    auto middle = begin + (length / 2);

    auto l_begin = begin;
    auto l_end = middle;
    auto r_begin = middle + 1;
    auto r_end = end;

    size_t l_len = length / 2;
    size_t r_len = length - l_len - 1;
    std::cout <<"node: " <<middle->first.p.transpose() <<std::endl;
    KDNodePtr left;
    if (l_len > 0) {
        left = make_tree(l_begin, l_end, l_len, (level + 1) % size_of_dimension);
    } else {
        left = leaf;
    }
    KDNodePtr right;
    if (r_len > 0) {
        right = make_tree(r_begin, r_end, r_len, (level + 1) % size_of_dimension);
    } else {
        right = leaf;
    }

    // KDNode result = KDNode();
    return std::make_shared< KDNode >(*middle, left, right);
}

KDTree::KDTree(poseVec point_array) {
    leaf = std::make_shared< KDNode >();
    // iterators
    pointIndexArr arr;
    for (size_t i = 0; i < point_array.size(); i++) {
        arr.push_back(pointIndex(point_array.at(i), i));
    }

    auto begin = arr.begin();
    auto end = arr.end();

    size_t length = arr.size();
    size_t level = 0;  // starting

    root = KDTree::make_tree(begin, end, length, level);
    cnt_ = 0;
}
using namespace std;

KDNodePtr KDTree::nearest_(   //
    const KDNodePtr &branch,  //
    const pose &pt,        //
    const size_t &level,      //
    const KDNodePtr &best,    //
    const double &best_dist   //
) {
    double d, dx, dx2;
    
    if (!bool(*branch)) {
        return NewKDNodePtr();  // basically, null
    }
    std::cout << "branch: " << branch->x.p.transpose() 
      <<"\tbest_node: " << best->x.p.transpose()<< std::endl;
    cnt_++;
    pose branch_pt(*branch);
    d = dist2(branch_pt, pt);
    dx = branch_pt.p(level) - pt.p(level);
    dx2 = dx * dx;

    KDNodePtr best_l = best;
    double best_dist_l = best_dist;
    if (d <= best_dist) {
        best_dist_l = d;
        best_l = branch;
    }
    size_t next_lv = (level + 1) % size_of_dimension;
    KDNodePtr section;
    KDNodePtr other;

    // select which branch makes sense to check
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    // keep nearest neighbor from further down the tree
    KDNodePtr further = nearest_(section, pt, next_lv, best_l, best_dist_l);
    if (!further->empty()) {
        double dl = dist2(further->x, pt);
        if (dl < best_dist_l) {
            best_dist_l = dl;
            best_l = further;
        }
    }
    // only check the other branch if it makes sense to do so
    if (dx2 < best_dist_l) {
        further = nearest_(other, pt, next_lv, best_l, best_dist_l);
        if (!further->empty()) {
            double dl = dist2(further->x, pt);
            if (dl < best_dist_l) {
                best_dist_l = dl;
                best_l = further;
            }
        }
    }

    return best_l;
};

// default caller
KDNodePtr KDTree::nearest_(const pose &pt) {
    size_t level = 0;
    // KDNodePtr best = branch;
    double branch_dist = dist2(pose(*root), pt);
    cnt_ =0;
    return nearest_(root,          // beginning of tree
                    pt,            // point we are querying
                    level,         // start from level 0
                    root,          // best is the root
                    branch_dist);  // best_dist = branch_dist
};

pose KDTree::nearest_point(const pose &pt) {
    return pose(*nearest_(pt));
};
size_t KDTree::nearest_index(const pose &pt) {
    return size_t(*nearest_(pt));
};

pointIndex KDTree::nearest_pointIndex(const pose &pt) {
    KDNodePtr Nearest = nearest_(pt);
    return pointIndex(pose(*Nearest), size_t(*Nearest));
}

pointIndexArr KDTree::neighborhood_(  //
    const KDNodePtr &branch,          //
    const pose &pt,                //
    const double &rad,                //
    const size_t &level               //
) {
    double d, dx, dx2;
    if (!bool(*branch)) {
        // branch has no point, means it is a leaf,
        // no points to add
        return pointIndexArr();
    }

    double r2 = rad * rad;

    d = dist2(pose(*branch), pt);
    dx = pose(*branch).p(level) - pt.p(level);
    dx2 = dx * dx;

    pointIndexArr nbh, nbh_s, nbh_o;
    if (d <= r2) {
        nbh.push_back(pointIndex(*branch));
    }

    //
    KDNodePtr section;
    KDNodePtr other;
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    nbh_s = neighborhood_(section, pt, rad, (level + 1) % size_of_dimension);
    nbh.insert(nbh.end(), nbh_s.begin(), nbh_s.end());
    if (dx2 < r2) {
        nbh_o = neighborhood_(other, pt, rad, (level + 1) % size_of_dimension);
        nbh.insert(nbh.end(), nbh_o.begin(), nbh_o.end());
    }

    return nbh;
};

pointIndexArr KDTree::neighborhood(  //
    const pose &pt,               //
    const double &rad) {
    size_t level = 0;
    return neighborhood_(root, pt, rad, level);
}

poseVec KDTree::neighborhood_points(  //
    const pose &pt,                 //
    const double &rad) {
    size_t level = 0;
    pointIndexArr nbh = neighborhood_(root, pt, rad, level);
    poseVec nbhp;
    nbhp.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
                   [](pointIndex x) { return x.first; });
    return nbhp;
}

indexArr KDTree::neighborhood_indices(  //
    const pose &pt,                  //
    const double &rad) {
    size_t level = 0;
    pointIndexArr nbh = neighborhood_(root, pt, rad, level);
    indexArr nbhi;
    nbhi.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhi.begin(),
                   [](pointIndex x) { return x.second; });
    return nbhi;
}


int main(int argc, char** argv)
{
  poseVec points;
  pose pt;

  pt.p = Eigen::Vector3d(0,0,0);
  points.push_back(pt);
  
//   pt = {1.0};
  pt.p = Eigen::Vector3d(1,0.5,0);
  points.push_back(pt);
  pt.p = Eigen::Vector3d(2,1.5,0);
  points.push_back(pt);
  pt.p = Eigen::Vector3d(3,2.5,0);
  points.push_back(pt);
  pt.p = Eigen::Vector3d(4,3.5,0);
  points.push_back(pt);
  pt.p = Eigen::Vector3d(6,4.5,0);
  points.push_back(pt);
  pt.p = Eigen::Vector3d(8,5.5,0);
  points.push_back(pt);
  KDTree tree(points);

  std::cout << "nearest test\n";
  pt.p = Eigen::Vector3d(3.3,1.5,0);
  auto res = tree.nearest_point(pt);
      std::cout << res.p.transpose() <<endl;

  std::cout << '\n';

  return 1;
    
}
