#pragma once

#if !defined(HAVE_TYPE_TRAITS_H) || HAVE_TYPE_TRAITS_H
#include_next <type_traits>
#endif

#if defined(HAVE_STD_REMOVE_REFERENCE) && !HAVE_STD_REMOVE_REFERENCE

namespace std {
    template<class T> struct remove_reference      { typedef T type; };
    template<class T> struct remove_reference<T&>  { typedef T type; };
    template<class T> struct remove_reference<T&&> { typedef T type; };
}

#endif
