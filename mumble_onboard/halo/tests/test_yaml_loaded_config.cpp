#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <gflags/gflags.h>
#include <halo/common/yaml_loaded_config.hpp>

using namespace std;

TEST(TestYAMLLoadedConfig, test_yaml_load_config) {
    std::ofstream ofs("./config.yaml", std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
        FAIL() << "Failed to open file ./config.yaml for writing.";
    }
    ofs << "my_int: 42" << std::endl;
    ofs << "my_float: 3.14" << std::endl;
    ofs << "my_string: Hello World" << std::endl;
    ofs << "my_bool: true" << std::endl;
    ofs << "my_vector: [1, 2, 3]" << std::endl;
    ofs.close();
    YamlLoadedConfig config;
    config.add_option<int>("my_int", 42);
    config.load_from_yaml("./config.yaml");
    cout << "my_int: " << config.get<int>("my_int") << endl;
    config.get<int>("my_int") = 423;
    cout << "my_int: " << config.get<int>("my_int") << endl;
    std::remove("./config.yaml");
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Now parse your --bag_path, --dataset_type, etc.
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
    return RUN_ALL_TESTS();
}
