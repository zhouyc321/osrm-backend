#ifndef OSRM_DJB2_H
#define OSRM_DJB2_H

#include <cstdint>

namespace osrm
{
namespace util
{

// Daniel J. Bernstein's djb2 string hashing function.
inline std::uint64_t djb2(const void *bytes)
{
    const auto *str = (const unsigned char *)bytes;

    std::uint64_t hash = 5381;
    int c;

    while ((c = *str++))
        hash = ((hash << 5) + hash) + c;

    return hash;
}
}
}

#endif
