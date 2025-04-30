/*
  ArduPilot JSON parser
 */

#pragma once

#include <cstdio>
#include <map>
#include <vector>

/*
  this avoids an issue on stm32 with std::string
 */
#undef _GLIBCXX_USE_C99_STDIO
#include <string>
#define _GLIBCXX_USE_C99_STDIO 1

class AP_JSON
{
public:
    struct null {};

    class value
    {
    public:
        typedef std::vector<value> array;
        typedef std::map<std::string, value> object;
        union _storage {
            bool boolean_;
            double number_;
            std::string *string_;
            array *array_;
            object *object_;
        };


    protected:
        int type_;
        _storage u_;

    public:
        value();
        value(int type, bool);
        explicit value(bool b);
        explicit value(double n);
        explicit value(const std::string &s);
        explicit value(const array &a);
        explicit value(const object &o);
        explicit value(std::string &&s);
        explicit value(array &&a);
        explicit value(object &&o);

        explicit value(const char *s);
        value(const char *s, size_t len);
        ~value();
        value(const value &x);
        value &operator=(const value &x);
        value(value &&x);
        value &operator=(value &&x);
        void swap(value &x);
        template <typename T> bool is() const;
        template <typename T> const T &get() const;
        template <typename T> T &get();
        template <typename T> void set(const T &);
        template <typename T> void set(T &&);
        bool evaluate_as_boolean() const;
        const value &get(const size_t idx) const;
        const value &get(const std::string &key) const;
        value &get(const size_t idx);
        value &get(const std::string &key);

        bool contains(const size_t idx) const;
        bool contains(const std::string &key) const;
        std::string to_str() const;

    private:
        template <typename T> value(const T *); // intentionally defined to block implicit conversion of pointer to bool
        void clear();
    };

    static std::string parse(value &out, const std::string &s);

    // load a json file, returning a value object.
    // caller must delete the returned pointer when done.
    static value *load_json(const char *filename);
};

