/**
 * @brief: this is a light-weight tool to read a yaml file and return a value in runtime.
 * Assumption: we currently do NOT support reading fields with the same name but in different namespaces.
 */
#pragma once

#include <any>
#include <typeindex>
#include <string>
#include <unordered_map>
#include <sstream>
#include <yaml-cpp/yaml.h>

using std::stringstream;

// That gives a working node.as<std::vector<double>>() in your reload lambda.
namespace YAML {
template <typename T>
struct convert<std::vector<T>> {
    static Node encode(const std::vector<T> &rhs) {
        Node node(NodeType::Sequence);
        for (auto const &v : rhs)
            node.push_back(v);
        return node;
    }

    static bool decode(const Node &node, std::vector<T> &rhs) {
        if (!node.IsSequence())
            return false;
        rhs.clear();
        for (auto const &element : node)
            rhs.push_back(element.as<T>());
        return true;
    }
};
}   // namespace YAML

/**
 * Usage:
 * YamlLoadedConfig config;
 * config.add_option("Option1", "default_value");   // using type deduction
 * config.add_option<size_t>("Option2", "default_value"); // using explicit type
 * config.load_from_yaml("path/to/config.yaml");
 * Voila, the options will magically be loaded from the YAML file! If a field is not found, the default value will be used.
 */
class YamlLoadedConfig {
  public:
    struct Field {
        std::string name;
        std::string ns;   // ← store the namespace here
        std::type_index type;
        std::any value;
        std::function<void(const YAML::Node &)> reload_func;

        Field() : type(typeid(void)) {}
    };

    YamlLoadedConfig() = default;

    template <typename T>
    void add_option(const std::string &name,
                    T &&default_value,
                    const std::string &ns = "") {
        // 1) Build the Field object on the stack
        Field field;
        field.name  = name;
        field.ns    = ns;   // save the namespace now
        field.type  = std::type_index(typeid(T));
        field.value = std::forward<T>(default_value);

        // 2) Insert it into the map, then grab a reference to the slot we just inserted.
        auto [it, inserted] = fields_.emplace(name, std::move(field));
        Field &slot         = it->second;   // stable reference to the Field in the map

        // 3) Create a reload_func that only captures:
        //    - `name` by value (key into the map)
        //    - nothing else from the local stack (ns is stored inside slot.ns).
        slot.reload_func = [this, name, &slot](const YAML::Node &root) {
            YAML::Node section;
            // First check: only if root is a defined map do we attempt root[ slot.ns ]
            if (!slot.ns.empty()) {
                YAML::Node sub = root[slot.ns];
                if (sub.IsDefined()) {
                    section = std::move(sub);
                } else {
                    std::cerr << "Namespace '" << slot.ns
                              << "' not found in YAML; defaulting to top-level\n";
                    // section stays as `root`
                    section = root;
                }
            } else {
                section = root;
            }

            // 3b) Now look up the actual field name under that chosen section:
            if (auto n = section[name]) {
                try {
                    // Write into the `value` slot that lives in the map:
                    fields_.at(name).value = n.as<T>();
                } catch (const std::exception &e) {
                    std::cerr << "Field '" << name
                              << "' exists but failed to cast. Using default.\n";
                }
            } else {
                std::cerr << "Field '" << name
                          << "' not found in namespace '"
                          << (slot.ns.empty() ? "<root>" : slot.ns)
                          << "'. Using default.\n";
            }
        };
    }

    void load_from_yaml(const std::string &file_path) {
        YAML::Node root = file_path.empty() ? YAML::Node{} : YAML::LoadFile(file_path);

        for (auto &[k, fld] : fields_) {
            fld.reload_func(root);
        }
    }

    template <typename T>
    T &get(const std::string &name) {
        auto it = fields_.find(name);
        if (it == fields_.end()) {
            throw std::runtime_error("Field '" + name + "' not found");
        }
        try {
            return std::any_cast<T &>(it->second.value);
        } catch (const std::bad_any_cast &) {
            std::stringstream ss;
            ss << "Field '" << name << "' exists but fails to cast: stored as "
               << it->second.type.name() << " but you asked for " << typeid(T).name();
            throw std::runtime_error(ss.str());
        }
    }

    template <typename T>
    const T &get(const std::string &name) const {
        auto it = fields_.find(name);
        if (it == fields_.end()) {
            throw std::runtime_error("Field '" + name + "' not found");
        }
        try {
            return std::any_cast<const T &>(it->second.value);
        } catch (const std::bad_any_cast &) {
            std::stringstream ss;
            ss << "Field '" << name << "' exists but fails to cast: stored as "
               << it->second.type.name() << " but you asked for " << typeid(T).name();
            throw std::runtime_error(ss.str());
        }
    }

  private:
    std::unordered_map<std::string, Field> fields_;
};
