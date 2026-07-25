/*
        simple_vector.h

        Defines the simple_vector<> template as an example on how to define
        custom containers that can be auto-serialized in XML with gSOAP.

        In order for the auto-generated XML serializers to work for templates,
        we must define at least these methods:

	void           clear()
        iterator       begin()
        const_iterator begin() const
        iterator       end()
        const_iterator end() const
        size_t         size() const
        iterator       insert(iterator pos, const_reference val)

	where begin() gives a forward iterator over the container.

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2010, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

// declare the simple_vector<> template:
#include <stdlib.h>

template <class T>
class simple_vector
{
public:
  typedef T                       value_type;
  typedef value_type            * pointer;
  typedef const value_type      * const_pointer;
  typedef value_type            & reference;
  typedef const value_type      & const_reference;
  typedef pointer                 iterator;
  typedef const_pointer           const_iterator;
protected:
  iterator                        head;
  iterator                        tail;
  size_t                          capacity;
public:
                                  simple_vector()       { head = tail = NULL; }
                                  simple_vector(const simple_vector& v)
                                                        { operator=(v); }
                                  ~simple_vector()      { if (head) delete[] head; }
  void                            clear()               { tail = head; }
/* the member functions below are required for (de)serialization of templates */
  iterator                        begin()               { return head; }
  const_iterator                  begin() const         { return head; }
  iterator                        end()                 { return tail; }
  const_iterator                  end() const           { return tail; }
  size_t                          size() const          { return tail - head; }
  iterator                        insert(iterator pos, const_reference val)
  { if (!head)
      head = tail = new value_type[capacity = 1];
    else if (tail >= head + capacity)
    { iterator i = head;
      iterator j = new value_type[capacity *= 2];
      iterator k = j;
      while (i < tail)
        *k++ = *i++;
      if (pos)
        pos = j + (pos - head);
      tail = j + (tail - head);
      delete[] head;
      head = j;
    }
    if (pos && pos >= head && pos < tail)
    { iterator i = tail;
      iterator j = i - 1;
      while (j != pos)
        *i-- = *j--;
      *pos = val;
    }
    else
    { pos = tail;
      *tail++ = val;
    }
    return pos;
  }
  simple_vector& operator=(const simple_vector& v)
  { head = tail = NULL;
    capacity = v.capacity;
    if (v.head)
    { head = tail = new value_type[capacity];
      iterator i = v.head;
      while (i != v.tail)
        *tail++ = *i++;
    }
    return *this;
  }
};
