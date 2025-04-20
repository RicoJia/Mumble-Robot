#pragma once

#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <list>
#include <cstddef>
#include <utility>

namespace halo {

template <typename Key, typename Value, bool verbose = false>
class LRUHashMap {
    /** Workflow:
        1. Add:
            1. create a pair of key, value, add it to the unordered map;
            2. Get the iterator of the new element, bring it to the front of the list
            3. If size exceeds, pop the last one
        2. Get:
            1. return the pointer to the value, and bring the order of the key-val pair front
        Move Construction:
            - We expect to have 1 copy of the key because we need to store it in the unordered_map.
            - Other than that, as we add, there is 1 move of the value, and 1 move of the key.
     */
  public:
    LRUHashMap(size_t sz) : size_(sz) {
        if (size_ == 0)
            throw std::runtime_error("LRUHashMap is instatiated with 0 size.");
        itr_lookup_.reserve(size_);
    }

    // So the compiler will have to make sure K, and V actually match Key and Value
    template <typename K, typename V>
    void add(K &&key, V &&value) {
        if (auto map_itr = itr_lookup_.find(key); map_itr != itr_lookup_.end()) {
            auto &list_itr   = map_itr->second;
            list_itr->second = std::forward<V>(value);
            cache_.splice(cache_.begin(), cache_, list_itr);
        } else {
            cache_.emplace_front(
                std::forward<K>(key), std::forward<V>(value));
            // key has been moved-from, need to create a copy for key
            itr_lookup_.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(cache_.begin()->first),
                std::forward_as_tuple(cache_.begin()));
        }
        // Pop if size has exceeded:
        if (cache_.size() > size_) {
            if constexpr (verbose) {
                std::cout << "popping: " << cache_.back().first << std::endl;
            }
            auto lru_key = cache_.back().first;
            itr_lookup_.erase(lru_key);
            cache_.pop_back();
        }
    }

    Value *get(const Key &key) {
        if (itr_lookup_.find(key) != itr_lookup_.end()) {
            auto list_itr = itr_lookup_[key];
            cache_.splice(cache_.begin(), cache_, list_itr);
            // TODO Should I bring it to the front?
            return &list_itr->second;
        } else {
            return nullptr;
        }
    }

  private:
    std::list<std::pair<Key, Value>> cache_;
    std::unordered_map<Key, typename std::list<std::pair<Key, Value>>::iterator> itr_lookup_;
    size_t size_;
};

}   // namespace halo