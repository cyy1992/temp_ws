#include "apriltag_family.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tagCustom24h9.h"
#include "tagCustom32h11.h"
#include "tagStandard32h11.h"
#include "tagStandard52h13.h"
typedef struct apriltag_family* (*factory_func_t)();
struct apriltag_family_info {
  const char* name;
  factory_func_t ctor;
};
static const struct apriltag_family_info lookup[] = {
  { "tag36h11", tag36h11_create },
  { "tag36h10", tag36h10_create },
  {"tagCustom24h9", tagCustom24h9_create },
  {"tagCustom32h11", tagCustom32h11_create },
  {"tagStandard32h11", tagStandard32h11_create },
  {"tagStandard52h13", tagStandard52h13_create },
  { NULL, NULL }
};
zarray_t* apriltag_family_list() {
  zarray_t* rval = zarray_create(sizeof(const char*));
  int i;
  for (i=0; lookup[i].name; ++i) {
    zarray_insert(rval, i, &lookup[i].name);
  }
  return rval;
  
}
apriltag_family_t* apriltag_family_create(const char* famname) {
  int i;
  for (i=0; lookup[i].name; ++i) {
    if (!strcmp(famname, lookup[i].name)) {
      return lookup[i].ctor();
    }
  }
  return NULL;
}
void apriltag_family_destroy(apriltag_family_t *tf) {
  
   free(tf->codes);
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
   
}
