#pragma once

#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <list>
#include <cstddef>
#include <deque>
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
    LRUHashMap(size_t sz) {
        initialize_if_havent(sz);
    }

    LRUHashMap() = default;
    void initialize_if_havent(size_t sz) {
        size_ = sz;
        if (size_ == 0)
            throw std::runtime_error("LRUHashMap is instatiated with 0 size.");
        itr_lookup_.reserve(size_);
    }

    /**
     * insert_only: if true, do not update the value if the key already exists.
     * In this case, the key is NOT moved to the front of the list.
     */
    template <typename K, typename V>
    void add(K &&key, V &&value, bool insert_only = false) {
        if (auto map_itr = itr_lookup_.find(key); map_itr != itr_lookup_.end()) {
            if (insert_only) {
                return;
            }
            auto &list_itr   = map_itr->second;
            list_itr->second = std::forward<V>(value);
            cache_.splice(cache_.begin(), cache_, list_itr);
        } else {
            cache_.emplace_front(
                std::forward<K>(key), std::forward<V>(value));
            // key has been moved-from, need to create a copy for key
            itr_lookup_.try_emplace(
                cache_.begin()->first,   // key
                cache_.begin()           // mapped_type must be constructible from this iterator
            );
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

    // For the getter, we currently are not bringing it to the front
    // for thread-safety and simplicity
    Value *get(const Key &key) {
        if (itr_lookup_.find(key) != itr_lookup_.end()) {
            auto list_itr = itr_lookup_[key];
            // cache_.splice(cache_.begin(), cache_, list_itr);
            return &list_itr->second;
        } else {
            return nullptr;
        }
    }

    size_t size() const {
        return cache_.size();
    }

  private:
    std::list<std::pair<Key, Value>> cache_;
    std::unordered_map<Key, typename std::list<std::pair<Key, Value>>::iterator> itr_lookup_;
    size_t size_;
};

template <typename T>
class SizeLimitedDeque {
  public:
    SizeLimitedDeque(size_t max_size) : max_size_(max_size) {}

    template <typename V>
    void push(V &&value) {
        if (deque_.size() >= max_size_) {
            deque_.pop_front();
        }
        deque_.emplace_back(std::forward<T>(value));
    }

    T &at(size_t index) {
        if (index >= deque_.size()) {
            throw std::out_of_range("SizeLimitdQueue query index out of range: " + std::to_string(index));
        }
        auto it = deque_.begin();
        std::advance(it, index);
        return *it;
    }

    const T at(size_t index) const {
        if (index >= deque_.size()) {
            throw std::out_of_range("SizeLimitdQueue query index out of range: " + std::to_string(index));
        }
        return deque_.at(index);
    }

    size_t size() const {
        return deque_.size();
    }

    void pop() {
        if (!deque_.empty()) {
            deque_.pop_front();
        }
    }

  private:
    std::deque<T> deque_;
    size_t max_size_;
};

}   // namespace halo