
Demonstates the use of a user-defined C++ container simple_vector<> defined in
simple_vector.h suitable for XML serialization. Elements contained can be
automatically serialized to and from XML. This requires a container that
supports at least these methods:

iterator	begin()
const_iterator	begin() const
iterator	end()
const iterator	end() const
size_t          size() const
iterator	insert(iterator, const_reference)

