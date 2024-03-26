#ifdef NDEBUG
#define assert(condition) [[gnu::assume(condition)]]
#else
#include <cassert>
#endif