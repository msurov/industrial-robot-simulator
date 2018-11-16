#include <fstream>
#include "properties.h"


namespace properties
{
    static Property _root;

    Property& root()
    {
        return _root;
    }

    void init(std::string const& filepath)
    {
        std::ifstream file(filepath);
        if (file.bad())
            throw_runtime_error("failed to open json file '", filepath, "'");
        file >> _root;
    }

    Property parse(std::string const& s)
    {
        std::istringstream ss(s);
        Json::Value json;
        ss >> json;
        return json;
    }

    bool find(std::string const& name)
    {
        auto& p = root();
        return p.isMember(name);
    }

    bool find(Property const& parent, std::string const& name)
    {
        return parent.isMember(name);
    }

    Property& get(std::string const& name)
    {
        auto& p = root();
        if (!p.isMember(name))
            throw_runtime_error("there is no such a property '", name, "'");
        return p[name];
    }

    Property& get(Property& parent, std::string const& name)
    {
        if (!parent.isMember(name))
            throw_runtime_error("there is no such a property '", name, "' in ", parent);
        return parent[name];
    }

    Property const& get(Property const& parent, int idx)
    {
        return parent[idx];
    }

    Property const& get(Property const& parent, std::string const& name)
    {
        if (!parent.isMember(name))
            throw_runtime_error("there is no such a property '", name, "' in ", parent);
        return parent[name];
    }

    std::string get_string(Property const& parent, std::string const& name)
    {
        return to_string(get(parent, name));
    }

    std::string get_string(Property const& parent, std::string const& name, std::string const& defval)
    {
        if (!find(parent, name))
            return defval;
        return to_string(get(parent, name));
    }

    bool is_string(Property const& prop)
    {
        return prop.isString();
    }

    bool is_array(Property const& prop)
    {
        return prop.isArray();
    }
}
