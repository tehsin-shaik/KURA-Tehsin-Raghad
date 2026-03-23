#pragma once

#include <fftw3.h>

#include <cstdlib>
#include <vector>

template <typename T>
class fftwf_allocator {
public:
    using value_type = T;

    fftwf_allocator() = default;
    template <class U>
    constexpr fftwf_allocator(const fftwf_allocator<U>&) noexcept {}
    T* allocate(std::size_t n) {
        if (n > std::size_t(-1) / sizeof(T))
            throw std::bad_alloc();
        if (auto p = static_cast<T*>(fftwf_malloc(n * sizeof(T))))
            return p;
        throw std::bad_alloc();
    }
    void deallocate(T* p, std::size_t) noexcept {
        fftwf_free(p);
    }
    // destroy does nothing since the allocator only uses primitive types.
    void destroy(T*) {}
};

template <class T, class U>
bool operator==(const fftwf_allocator<T>&, const fftwf_allocator<U>&) {
    return true;
}
template <class T, class U>
bool operator!=(const fftwf_allocator<T>&, const fftwf_allocator<U>&) {
    return false;
}

template <typename T>
using fftwf_vector = std::vector<T, fftwf_allocator<T>>;
