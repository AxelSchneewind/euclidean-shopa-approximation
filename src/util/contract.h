#ifdef NDEBUG
#define assert(condition) [[assume(condition)]]
#else
#include <cassert>
#endif