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

#define DECLARE_TYPESAFE_INDEX(NAME, TYPE) typedef typesafe_index<TYPE, class TAG_##NAME> NAME

/// This template allows for indexing with compile time check and generating
/// error for using one index variable for another array that is to be indexed
/// with different variable.
/// base_type specifies
///
/// @param  base_type      Base integral type
///
/// @param tag             Tag Name to prevent copy from one type to other
///
template <class base_type, class tag> class typesafe_index
{
public:
    explicit typesafe_index(base_type i = base_type()) : p(i) {}

    void operator=(const base_type& val)
    {
        p = val;
    }

    constexpr base_type get_int() const
    {
        return p;
    }

    typesafe_index operator++()
    {
        return typesafe_index(++p);
    }

    typesafe_index operator++(int)
    {
        return typesafe_index(p++);
    }

    bool operator<(const base_type& val) const
    {
        return (p<val);
    }

    bool operator<=(const base_type& val) const
    {
        return (p<=val);
    }

    bool operator>=(const base_type& val) const
    {
        return (p>=val);
    }


    bool operator>(const base_type& val) const
    {
        return (p>val);
    }

    bool operator!=(const base_type& val) const
    {
        return (p!=val);
    }

    bool operator==(const base_type& val) const
    {
        return (p==val);
    }

    explicit constexpr operator base_type() const
    {
        return p;
    }

    typesafe_index operator+(const base_type& val) const
    {
        return typesafe_index(p+val);
    }
private:
    base_type p;
};

/// This template associates the the base_type array with accessor_type(index).
/// So the elements can be accessed using [] only using accessor_type index
/// _priv_instance is kept public for use in Parameter declaration.
///
template <class base_type, uint32_t num_instances, typename accessor_type> class RestrictIDTypeArray
{
public:
    base_type _priv_instance[num_instances];
    base_type& operator[](const accessor_type& index)
    {
        return _priv_instance[index.get_int()];
    }

    constexpr const base_type& operator[](const accessor_type& index) const
    {
        return _priv_instance[index.get_int()];
    }
};