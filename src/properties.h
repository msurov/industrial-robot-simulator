#pragma once

#include <json/json.h>
#include "throws.h"


using Property = Json::Value;


namespace properties
{
    void init(std::string const& filepath);
    Property parse(std::string const& json);

    Property& root();
    bool find(std::string const& name);
    bool find(Property const& parent, std::string const& name);
    Property& get(std::string const& name);
    Property& get(Property& parent, std::string const& name);
    Property const& get(Property const& parent, std::string const& name);
    Property const& get(Property const& parent, int idx);

    template <typename T>
    inline void set(Property& prop, T const& val)
    {
        prop = val;
    }

    template <typename T>
    void add(Property& p, std::string const& key, T const& val)
    {
        p[key] = val;
    }

    bool is_string(Property const& prop);
    bool is_array(Property const& prop);

    inline std::string to_string(Property const& prop) { return prop.asString(); }
    inline double to_double(Property const& prop) { return prop.asDouble(); }
    inline int to_int(Property const& prop) { return prop.asInt(); }
    inline uint to_uint(Property const& prop) { return prop.asUInt(); }
    inline int64_t to_int64(Property const& prop) { return prop.asInt64(); }
    inline uint64_t to_uint64(Property const& prop) { return prop.asUInt64(); }

    template <typename T> T convert_property(Property const&);
    template <> inline double convert_property<double>(Property const& prop) { return to_double(prop); }
    template <> inline int convert_property<int>(Property const& prop) { return to_int(prop); }
    template <> inline uint convert_property<uint>(Property const& prop) { return to_uint(prop); }
    template <> inline int64_t convert_property<int64_t>(Property const& prop) { return to_int64(prop); }
    template <> inline uint64_t convert_property<uint64_t>(Property const& prop) { return to_uint64(prop); }
    template <> inline std::string convert_property<std::string>(Property const& prop) { return to_string(prop); }

    template <typename T>
    inline std::vector<T> to_vector(Property const& prop)
    {
        const int n = prop.size();
        std::vector<T> vec(n);

        for (int i = 0; i < n; ++ i)
            vec[i] = convert_property<T>(prop[i]);

        return vec;
    }

    template <typename T, int n>
    inline std::array<T, n> to_array(Property const& prop)
    {
        if (prop.size() != n)
            throw_runtime_error("property has ", prop.size(), " elements, but requited ", n);
        std::array<T, n> arr(n);

        for (int i = 0; i < n; ++ i)
            arr[i] = convert_property<T>(prop[i]);

        return arr;
    }

    template <typename T>
    struct GetNumeric
    {
        inline T operator()(Property const& parent, std::string const& name) const
        {
            static_assert(std::is_arithmetic<T>::value, "expected a numeric type");
            auto const& p = get(parent, name);
            return convert_property<T>(p);
        }

        inline T operator()(Property const& parent, int i) const
        {
            static_assert(std::is_arithmetic<T>::value, "expected a numeric type");
            auto const& p = get(parent, i);
            return convert_property<T>(p);
        }

        inline T operator()(Property const& parent, std::string const& name, T const& minval, T const& maxval) const
        {
            auto const& p = get(parent, name);
            auto const& v = convert_property<T>(p);
            if (v < minval || v > maxval)
                throw_runtime_error("parameter ", name, " is out of range ", minval, "..", maxval);
            return v;
        }

        inline T operator()(Property const& parent, int i, T const& minval, T const& maxval) const
        {
            auto const& p = get(parent, i);
            auto const& v = convert_property<T>(p);
            if (v < minval || v > maxval)
                throw_runtime_error("parameter ", i, " is out of range ", minval, "..", maxval);
            return v;
        }
    };

    static const auto get_int = GetNumeric<int>();
    static const auto get_double = GetNumeric<double>();
    static const auto get_int64 = GetNumeric<int64_t>();

    std::string get_string(Property const& parent, std::string const& name);
    std::string get_string(Property const& parent, std::string const& name, std::string const& defval);
}
