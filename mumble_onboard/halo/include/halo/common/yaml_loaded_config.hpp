#include <typeindex>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

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
    // How do you store a value, then cast it?
    struct Field {
        std::string name;
        std::type_index type;
        std::any value;
        std::function<void(const YAML::Node &)> reload_func;   // <<-- take Node
        Field() : type(typeid(void)) {}
    };

    YamlLoadedConfig() = default;

    template <typename T>
    void add_option(const std::string &name, T &&default_value) {
        Field field;
        field.name        = name;
        field.type        = std::type_index(typeid(T));
        field.value       = std::forward<T>(default_value);
        field.reload_func = [this, name](const YAML::Node &root) {
            if (auto n = root[name]) {
                try {
                    fields_[name].value = n.as<T>();
                } catch (const std::exception &e) {
                    std::cerr << "Field '" << name << "' exists but fails to be casted. Using default value\n";
                }
            } else {
                std::cerr << "Field '" << name << "' not found in YAML. Using default value\n";
            }
        };

        fields_.emplace(name, std::move(field));
    }

    void load_from_yaml(const std::string file_path) {
        YAML::Node root = YAML::LoadFile(file_path);
        for (auto &[k, fld] : fields_) {
            fld.reload_func(root);
        }
    }

    template <typename T>
    T &get(const std::string &name) {
        auto it = fields_.find(name);
        if (it != fields_.end()) {
            try {
                return std::any_cast<T &>(it->second.value);
            } catch (const std::bad_any_cast &e) {
                std::cerr << "Field '" << name << "' exists but fails to be casted. Type: " << it->second.type.name()
                          << ", but wanted: " << typeid(T).name() << '\n';
            }
        } else {
            throw std::runtime_error("Field '" + name + "' not found");
        }
    }

    template <typename T>
    const T &get(const std::string &name) const {
        auto it = fields_.find(name);
        if (it != fields_.end()) {
            try {
                return std::any_cast<const T &>(it->second.value);
            } catch (const std::bad_any_cast &e) {
                std::cerr << "Field '" << name << "' exists but fails to be casted. Type: " << it->second.type.name()
                          << ", but wanted: " << typeid(T).name() << '\n';
            }
        } else {
            throw std::runtime_error("Field '" + name + "' not found");
        }
    }

  private:
    std::unordered_map<std::string, Field> fields_;
};
