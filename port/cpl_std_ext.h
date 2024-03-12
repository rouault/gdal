#ifndef CPL_STD_EXT_H
#define CPL_STD_EXT_H

/*! @cond Doxygen_Suppress */

#if defined(GDAL_COMPILATION)

#include <map>
#include <set>

namespace cpl
{
#if __cplusplus > 201703L
using std::map;
using std::set;
#else
/** Helper offering C++20-like features for std::map */
template <typename K, typename V> class map : public std::map<K, V>
{
  public:
    inline map() = default;
    inline bool contains(const K &key) const
    {
        return this->find(key) != this->end();
    }
};

/** Helper offering C++20-like features for std::set */
template <typename K> class set : public std::set<K>
{
  public:
    inline set() = default;
    inline bool contains(const K &key) const
    {
        return this->find(key) != this->end();
    }
};
#endif
}  // namespace cpl

#endif  // GDAL_COMPILATION

/*! @endcond */

#endif  // CPL_STD_EXT_H
