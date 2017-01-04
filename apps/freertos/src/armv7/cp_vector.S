
#include "asm_defines.h"

    // External symbol references
    .global vectors
    .global __ram_vectors_start
    .global __ram_vectors_end
    .global top_of_stacks
    .code 32
    .section ".startup","ax"
    
    .global _cp_vector
    .func _cp_vector 
    .fnstart
_cp_vector:

    ldr		r0, =top_of_stacks  @ symbol defined in linker file
    mov		r1, #EXCEPTION_STACK_SIZE

    @ get cpu id, and subtract the offset from the stacks base address
    mrc     p15,0,r2,c0,c0,5  @ read multiprocessor affinity register
    and     r2, r2, #3        @ mask off, leaving CPU ID field
    mov     r5, r2		      @ save cpu id for later

    mul     r3, r2, r1
    sub     r0, r0, r3
  
    mov     r1, r1, lsl #2
           
    @ set stacks for all other modes 
    msr     CPSR_c, #MODE_FIQ | I_BIT | F_BIT
    mov     sp, r0
    sub     r0, r0, r1 
    
    msr     CPSR_c, #MODE_IRQ | I_BIT | F_BIT
    mov     sp, r0
    sub     r0, r0, r1 
    
    msr     CPSR_c, #MODE_ABT | I_BIT | F_BIT
    mov     sp, r0
    sub     r0, r0, r1 
    
    msr     CPSR_c, #MODE_UND | I_BIT | F_BIT
    mov     sp, r0
    sub     r0, r0, r1 
    
    msr     CPSR_c, #MODE_SYS | I_BIT | F_BIT
    mov     sp, r0
    sub     r0, r0, r1 
    
    @ Set SVC mode stack with interrupts disabled 
    msr     CPSR_c, #MODE_SVC | I_BIT | F_BIT
    mov     sp, r0
    sub     r0, r0, r1 
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

