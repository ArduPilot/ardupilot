// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  Singly Linked List for APM
 *  Author: Siddharth B Purohit
 */
//TODO: Make it thread safe, Move to HAL Utils

template<class T>
class node_t {
public:
   node_t() : _prev_node(NULL) {}

   void set_prev(T prev_node) { _prev_node = prev_node; }
   T get_prev() { return _prev_node; }
protected:
   T _prev_node;
};

template<class T>
class calllist_t {
public:
   calllist_t() : _head(NULL) {}
   void push_back(T new_node) {
      new_node->set_prev(_head);
      _head = new_node;
   }
   T get_head() { return _head; }
protected:
   T _head;
};