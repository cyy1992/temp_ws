#include <stdlib.h>
#include "tagCustom24h9.h"

apriltag_family_t *tagCustom24h9_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagCustom24h9");
   tf->h = 9;
   tf->ncodes = 26;
   tf->codes = calloc(26, sizeof(uint64_t));
   tf->codes[0] = 0x00000000006d614aUL;
   tf->codes[1] = 0x0000000000387e23UL;
   tf->codes[2] = 0x0000000000749537UL;
   tf->codes[3] = 0x0000000000b0ac4bUL;
   tf->codes[4] = 0x000000000099d4aeUL;
   tf->codes[5] = 0x0000000000bf1425UL;
   tf->codes[6] = 0x0000000000fb2b39UL;
   tf->codes[7] = 0x000000000067361cUL;
   tf->codes[8] = 0x0000000000b07934UL;
   tf->codes[9] = 0x000000000077e28bUL;
   tf->codes[10] = 0x00000000009e2ac1UL;
   tf->codes[11] = 0x0000000000a6676dUL;
   tf->codes[12] = 0x00000000001d26a8UL;
   tf->codes[13] = 0x00000000004f71a4UL;
   tf->codes[14] = 0x0000000000d9347bUL;
   tf->codes[15] = 0x0000000000efd901UL;
   tf->codes[16] = 0x0000000000ff1fcfUL;
   tf->codes[17] = 0x0000000000b9e880UL;
   tf->codes[18] = 0x0000000000a3850eUL;
   tf->codes[19] = 0x0000000000581945UL;
   tf->codes[20] = 0x0000000000574890UL;
   tf->codes[21] = 0x0000000000ca5048UL;
   tf->codes[22] = 0x00000000005abca0UL;
   tf->codes[23] = 0x000000000046824dUL;
   tf->codes[24] = 0x000000000069583dUL;
   tf->codes[25] = 0x000000000003a4f9UL;
   tf->nbits = 24;
   tf->bit_x = calloc(24, sizeof(uint32_t));
   tf->bit_y = calloc(24, sizeof(uint32_t));
   tf->bit_x[0] = 6;
   tf->bit_y[0] = -2;
   tf->bit_x[1] = 7;
   tf->bit_y[1] = -2;
   tf->bit_x[2] = 8;
   tf->bit_y[2] = -2;
   tf->bit_x[3] = 9;
   tf->bit_y[3] = -2;
   tf->bit_x[4] = 10;
   tf->bit_y[4] = -2;
   tf->bit_x[5] = 11;
   tf->bit_y[5] = -2;
   tf->bit_x[6] = 19;
   tf->bit_y[6] = 6;
   tf->bit_x[7] = 19;
   tf->bit_y[7] = 7;
   tf->bit_x[8] = 19;
   tf->bit_y[8] = 8;
   tf->bit_x[9] = 19;
   tf->bit_y[9] = 9;
   tf->bit_x[10] = 19;
   tf->bit_y[10] = 10;
   tf->bit_x[11] = 19;
   tf->bit_y[11] = 11;
   tf->bit_x[12] = 11;
   tf->bit_y[12] = 19;
   tf->bit_x[13] = 10;
   tf->bit_y[13] = 19;
   tf->bit_x[14] = 9;
   tf->bit_y[14] = 19;
   tf->bit_x[15] = 8;
   tf->bit_y[15] = 19;
   tf->bit_x[16] = 7;
   tf->bit_y[16] = 19;
   tf->bit_x[17] = 6;
   tf->bit_y[17] = 19;
   tf->bit_x[18] = -2;
   tf->bit_y[18] = 11;
   tf->bit_x[19] = -2;
   tf->bit_y[19] = 10;
   tf->bit_x[20] = -2;
   tf->bit_y[20] = 9;
   tf->bit_x[21] = -2;
   tf->bit_y[21] = 8;
   tf->bit_x[22] = -2;
   tf->bit_y[22] = 7;
   tf->bit_x[23] = -2;
   tf->bit_y[23] = 6;
   tf->width_at_border = 18;
   tf->total_width = 22;
   tf->reversed_border = true;
   return tf;
}

void tagCustom24h9_destroy(apriltag_family_t *tf)
{
   free(tf->codes);
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
