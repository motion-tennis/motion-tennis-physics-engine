#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

namespace motion_tennis::physics {

// Fixed-capacity sequence without heap allocation. Capacities mirror the static
// v0.3 codec bounds so the simulation path never grows a collection.
template <typename T, std::size_t Capacity>
class BoundedVector {
public:
    static constexpr std::size_t capacity() { return Capacity; }

    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == Capacity; }

    // Returns false instead of growing; callers must account for the rejection.
    bool push_back(const T& value) {
        if (size_ == Capacity) {
            return false;
        }
        items_[size_++] = value;
        return true;
    }

    void clear() {
        items_ = {};
        size_ = 0;
    }

    T& operator[](std::size_t index) { return items_[index]; }
    const T& operator[](std::size_t index) const { return items_[index]; }

    T* begin() { return items_.data(); }
    T* end() { return items_.data() + size_; }
    const T* begin() const { return items_.data(); }
    const T* end() const { return items_.data() + size_; }

    friend bool operator==(const BoundedVector& left, const BoundedVector& right) {
        return left.size_ == right.size_ && std::equal(left.begin(), left.end(), right.begin());
    }

private:
    std::array<T, Capacity> items_{};
    std::size_t size_ = 0;
};

}  // namespace motion_tennis::physics
