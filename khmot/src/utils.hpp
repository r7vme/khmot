#pragma once
#include <iostream>

#ifdef DEBUG_LOG
#define KH_DEBUG(x)                                                    \
  do {                                                                 \
    std::cerr << __FILE__ << ":" << __LINE__ << " " << x << std::endl; \
  } while (0)
#else
// clang-format off
#define KH_DEBUG(x) do {} while (0)
// clang-format on
#endif
