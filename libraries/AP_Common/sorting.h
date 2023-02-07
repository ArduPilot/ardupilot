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
  in-place insertion sort for small arrays of data. This is O(n) if
  already sorted and O(n^2) for worst case (elements are reversed)
  sort order is smallest first
 */
void insertion_sort_uint16(uint16_t *data, uint16_t n);

/*
  remove duplicates from a sorted uint16_t array, returning the new
  count
 */
uint16_t remove_duplicates_uint16(uint16_t *data, uint16_t n);

/*
  bisection search on a sorted uint16_t array to find an element
  return true if found
*/
bool bisect_search_uint16(const uint16_t *data, uint16_t n, uint16_t value);

/*
  remove elements in a 2nd sorted array from a sorted uint16_t array
  return the number of remaining elements
 */
uint16_t remove_list_uint16(uint16_t *data, uint16_t n, const uint16_t *rem, uint16_t n2);

/*
  return number of common elements between two sorted uint16_t lists
 */
uint16_t common_list_uint16(uint16_t *data, uint16_t n, const uint16_t *rem, uint16_t n2);
