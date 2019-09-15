/*
 * Copyright [2019] [Ke Sun]
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
 */

#pragma once

#include <boost/smart_ptr.hpp>
#include <boost/pointer_cast.hpp>

namespace planner {

/**
 * \brief LatticeNode is supposed to be the base class of all nodes
 *        to be used with the the Lattice class.
 *
 * This class provides the interface for accessing and setting the
 * nodes around a node object.
 */
template<typename Derived>
class LatticeNode {

protected:

  /// Front node.
  boost::weak_ptr<Derived> front_;

  /// Back node.
  boost::weak_ptr<Derived> back_;

  /// Left node.
  boost::weak_ptr<Derived> left_;

  /// Right node.
  boost::weak_ptr<Derived> right_;

public:

  // Default constructor.
  LatticeNode() = default;

  /** @name Accessors
   *
   * front(), back(), left(), right() returns reference
   * of the boost weak pointers stored in the object, so that one can
   * update the weak pointers directly.
   */
  /// @{

  boost::weak_ptr<Derived>& front() {
    return front_;
  }

  boost::weak_ptr<Derived>& back() {
    return back_;
  }

  boost::weak_ptr<Derived>& left() {
    return left_;
  }

  boost::weak_ptr<Derived>& right() {
    return right_;
  }

  /// @}

  /** @name const Accessors
   *
   * front(), back(), left(), right() returns boost shared pointers
   * pointering to const LatticeNode objects.
   */
  /// @{

  boost::shared_ptr<const Derived> front() const {
    return boost::const_pointer_cast<const Derived>(front_.lock());
  }

  boost::shared_ptr<const Derived> back() const {
    return boost::const_pointer_cast<const Derived>(back_.lock());
  }

  boost::shared_ptr<const Derived> left() const {
    return boost::const_pointer_cast<const Derived>(left_.lock());
  }

  boost::shared_ptr<const Derived> right() const {
    return boost::const_pointer_cast<const Derived>(right_.lock());
  }

  /// @}

}; // End class WaypointNode.
} // End namespace planner.
