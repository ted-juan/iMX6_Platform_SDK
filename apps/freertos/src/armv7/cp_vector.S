
#include "asm_defines.h"

    // External symbol references
    .global vectors
    .global __ram_vectors_start
    .global __ram_vectors_end
    
    .code 32
    .section ".startup","ax"
    
    .global _cp_vector
    .func _cp_vector 
    .fnstart
_cp_vector:
     /* copy the vector table into the RAM vectors
     * this assumes that the RAM vectors size is divisible by 3 words (12 bytes)
     */
    ldr     r1,=__ram_vectors_start
    ldr     r2,=__ram_vectors_end
    ldr     r3,=vectors
1:  cmp     r1,r2
    ldmlt   r3!,{r4,r5,r6}
    stmlt   r1!,{r4,r5,r6}
    blt     1b
    bx      lr 
    
    .cantunwind
    .fnend
    .endfunc    @ _cp_vector()

    /* local label for the stack fill pattern */
.Ldeadfeed:
    .word   0xdeadfeed


    .end

