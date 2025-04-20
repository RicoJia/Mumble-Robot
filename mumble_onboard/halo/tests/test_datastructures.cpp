#include <halo/common/data_structures.hpp>

#include <cassert>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <string>

struct KeyTracer {
    std::string name;

    KeyTracer(const std::string &n) : name(n) { std::cout << "[Key Ctor]    \"" << name << "\"\n"; }
    KeyTracer(const KeyTracer &o) : name(o.name) { std::cout << "[Key Copy]    \"" << name << "\"\n"; }
    KeyTracer(KeyTracer &&o) noexcept : name(std::move(o.name)) { std::cout << "[Key Move]    new=\"" << name << "\"  old=\"" << o.name << "\"\n"; }
    // equality for unordered_map
    friend bool operator==(KeyTracer const &a, KeyTracer const &b) {
        return a.name == b.name;
    }
};

std::ostream &operator<<(std::ostream &os, const KeyTracer &k) {
    os << "\"" << k.name << "\"";
    return os;
}

//----------------------------------------------------------------------
// Provide std::hash<KeyTracer>
namespace std {
template <>
struct hash<KeyTracer> {
    size_t operator()(KeyTracer const &k) const noexcept {
        return std::hash<std::string>()(k.name);
    }
};
}   // namespace std

//----------------------------------------------------------------------
// A small tracer for values
struct ValueTracer {
    int x, y;

    ValueTracer(int _x, int _y) : x(_x), y(_y) { std::cout << "[Value Ctor]  (" << x << ", " << y << ")\n"; }
    ValueTracer(const ValueTracer &o) : x(o.x), y(o.y) { std::cout << "[Value Copy]  (" << x << ", " << y << ")\n"; }
    ValueTracer(ValueTracer &&o) noexcept : x(o.x), y(o.y) { std::cout << "[Value Move]  new=(" << x << ", " << y << ")  old=(" << o.x << ", " << o.y << ")\n"; }

    // move‑assign
    ValueTracer &operator=(ValueTracer &&o) noexcept {
        x = o.x;
        y = o.y;
        std::cout << "[Value MoveAssign]  new=("
                  << x << ", " << y << ")  old=("
                  << o.x << ", " << o.y << ")\n";
        return *this;
    }
};

std::ostream &operator<<(std::ostream &os, const ValueTracer &v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

TEST(TestDatastructures, TestLRUHashMapVanilla) {
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

TEST(TestDatastructures, TestLRUHashMapMoveSemantics) {
    halo::LRUHashMap<KeyTracer, ValueTracer, true> lru_hashmap(3);
    // TODO
    std::cout << "expect 1 copy and 2 moves:" << std::endl;
    lru_hashmap.add(KeyTracer("hello1"), ValueTracer(1, 2));
    // TODO
    std::cout << "expect 1 copy and 2 moves:" << std::endl;
    KeyTracer key2("hello2");
    ValueTracer val2(3, 4);
    lru_hashmap.add(std::move(key2), std::move(val2));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);

    return RUN_ALL_TESTS();
}