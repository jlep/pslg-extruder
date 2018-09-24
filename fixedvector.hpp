#pragma once

template <typename T, size_t N>
struct FixedVector {

    size_t size = 0;
    std::array<T, N> arr;
    using Size = typename std::array<T, N>::size_type;

    void push_back(T value) {
        arr[size++] = std::move(value);
    }

    const T& operator[](Size i) const {
        return arr[i];
    }

    auto begin() {
        return arr.begin();
    }

    auto end() {
        return arr.begin() + size;
    }
};
