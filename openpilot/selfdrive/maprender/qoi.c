// Define QOI_IMPLEMENTATION exactly once to compile the qoi_encode() implementation
// into maprenderd. The declaration lives in qoi.h (used by maprenderd.cc + the
// python-side decoder in sp_maprender_client.py).
#define QOI_IMPLEMENTATION
#include "qoi.h"