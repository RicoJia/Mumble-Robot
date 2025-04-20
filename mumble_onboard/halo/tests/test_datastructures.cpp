#include <halo/common/data_structures.hpp>

#include <cassert>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <string>

TEST(TestDatastructures, TestLRUHashMap) {
    try {
        halo::LRUHashMap<int, std::string, true> lru_hashmap(0);
    } catch (std::runtime_error) {
    }

    halo::LRUHashMap<int, std::string, true> lru_hashmap(3);
    lru_hashmap.add(1, "hello1");
    lru_hashmap.add(2, "hello2");
    lru_hashmap.add(3, "hello3");
    lru_hashmap.add(4, "hello4");   // should see popping 1

    lru_hashmap.add(2, "hello22");    // Nothing is popped
    lru_hashmap.add(2, "hello222");   // Nothing is popped
    lru_hashmap.add(5, "hello5");     // should see popping 3

    auto val_ptr = lru_hashmap.get(2);
    if (val_ptr == nullptr) {
        ASSERT_NE(val_ptr, nullptr) << "val_ptr should not be nullptr";
    } else {
        std::cout << "should see hello222: " << *val_ptr << std::endl;
    }

    lru_hashmap.add(6, "hello6");   // should see popping 4
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);

    return RUN_ALL_TESTS();
}