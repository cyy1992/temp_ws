#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "apriltag.h"

zarray_t* apriltag_family_list();
struct apriltag_family* apriltag_family_create(const char* family_name);
void apriltag_family_destroy(struct apriltag_family*);

#ifdef __cplusplus
}
#endif
