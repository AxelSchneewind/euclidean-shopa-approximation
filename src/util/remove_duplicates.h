#pragma once

#include <unordered_map>
#include <vector>

/**
 * removes duplicates in-place
 * @tparam T
 * @param items
 */
template <typename T>
void
remove_duplicates (std::vector<T> &items)
{
  std::unordered_map<T, bool, std::hash<T>> found(10000);

  int j = 0;
  for (int i = 0; i < items.size (); ++i)
  {
    auto element = items[i];

    if (!found.contains (element))
    {
      found[element] = true;
      items[j++] = items[i];
    }
  }

  items.resize(j);
  items.shrink_to_fit();
}