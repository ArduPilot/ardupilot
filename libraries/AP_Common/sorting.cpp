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

#include <stdint.h>
#include "sorting.h"

/*
  in-place insertion sort for small arrays of data. This is O(n) if
  already sorted and O(n^2) for worst case (elements are reversed)
  sort order is smallest first
 */
void insertion_sort_uint16(uint16_t *data, uint16_t n)
{
    for (uint16_t i=1; i<n; i++) {
        uint16_t temp = data[i];
        int16_t j = i - 1;

        while (j >= 0 && data[j] > temp) {
            data[j+1] = data[j];
            j--;
		}
        data[j+1] = temp;
    }
}

/*
  remove duplicates from a sorted uint16_t array, returning the new
  count
 */
uint16_t remove_duplicates_uint16(uint16_t *data, uint16_t n)
{
    uint16_t removed = 0;
    for (uint16_t i=1; i<n; i++) {
        if (data[i-(1+removed)] == data[i]) {
            removed++;
        } else if (removed != 0) {
            data[i-removed] = data[i];
        }
    }
    return n - removed;
}

/*
  bisection search on a sorted uint16_t array to find an element
  return true if found
*/
bool bisect_search_uint16(const uint16_t *data, uint16_t n, uint16_t value)
{
    if (n == 0) {
        return false;
    }
    uint16_t low=0, high=n-1;
    while (low < high) {
        uint16_t mid = (low+high)/2;
        if (value < data[mid]) {
            high = mid;
            continue;
        }
        if (value > data[mid]) {
            low = mid+1;
            continue;
        }
        return true;
    }
    return data[low] == value;
}

/*
  remove elements in a 2nd sorted array from a sorted uint16_t array
  return the number of remaining elements
 */
uint16_t remove_list_uint16(uint16_t *data, uint16_t n, const uint16_t *rem, uint16_t n2)
{
    uint16_t removed = 0;
    for (uint16_t i=0; i<n; i++) {
        if (bisect_search_uint16(rem, n2, data[i])) {
            removed++;
        } else if (removed != 0) {
            data[i-removed] = data[i];
        }
    }
    return n - removed;
}

/*
  return number of common elements between two sorted uint16_t lists
 */
uint16_t common_list_uint16(uint16_t *data, uint16_t n, const uint16_t *data2, uint16_t n2)
{
    uint16_t common = 0;
    for (uint8_t i=0; i<n2; i++) {
        if (bisect_search_uint16(data, n, data2[i])) {
            common++;
        }
    }
    return common;
}
