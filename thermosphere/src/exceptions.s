/* Some macros taken from https://github.com/ARM-software/arm-trusted-firmware/blob/master/include/common/aarch64/asm_macros.S */
/*
 * Copyright (c) 2013-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 

/*
 * Declare the exception vector table, enforcing it is aligned on a
 * 2KB boundary, as required by the ARMv8 architecture.
 * Use zero bytes as the fill value to be stored in the padding bytes
 * so that it inserts illegal AArch64 instructions. This increases
 * security, robustness and potentially facilitates debugging.
 */
.macro vector_base  label, section_name=.vectors
.section \section_name, "ax"
.align 11, 0
\label:
.endm

/*
 * Create an entry in the exception vector table, enforcing it is
 * aligned on a 128-byte boundary, as required by the ARMv8 architecture.
 * Use zero bytes as the fill value to be stored in the padding bytes
 * so that it inserts illegal AArch64 instructions. This increases
 * security, robustness and potentially facilitates debugging.
 */
.macro vector_entry  label, section_name=.vectors
.cfi_sections .debug_frame
.section \section_name, "ax"
.align 7, 0
.type \label, %function
.func \label
.cfi_startproc
\label:
.endm

/*
 * This macro verifies that the given vector doesnt exceed the
 * architectural limit of 32 instructions. This is meant to be placed
 * immediately after the last instruction in the vector. It takes the
 * vector entry as the parameter
 */
.macro check_vector_size since
    .endfunc
    .cfi_endproc
    .if (. - \since) > (32 * 4)
        .error "Vector exceeds 32 instructions"
    .endif
.endm

/* Actual Vectors for Thermosphere. */
.global thermosphere_vectors
vector_base thermosphere_vectors

/* Current EL, SP0 */
.global unknown_exception
unknown_exception:
vector_entry synch_sp0
    /* Guarantee a panic. */
    SMC #2
    check_vector_size synch_sp0

vector_entry irq_sp0
    b unknown_exception
    check_vector_size irq_sp0

vector_entry fiq_sp0
    b unknown_exception
    check_vector_size fiq_sp0

vector_entry serror_sp0
    b unknown_exception
    check_vector_size serror_sp0

/* Current EL, SPx */
vector_entry synch_spx
    b unknown_exception
    check_vector_size synch_spx

vector_entry irq_spx
    b unknown_exception
    check_vector_size irq_spx

vector_entry fiq_spx
    b unknown_exception
    check_vector_size fiq_spx

vector_entry serror_spx
    b unknown_exception
    check_vector_size serror_spx
    
/* Lower EL, A64 */
vector_entry synch_a64
    stp x30, xzr, [sp, #-0x10]!
    stp x28, x29, [sp, #-0x10]!
    stp x26, x27, [sp, #-0x10]!
    stp x24, x25, [sp, #-0x10]!
    bl _handle_synch_exception
    stp x24, x25, [sp, #-0x10]!
    ldp x26, x27, [sp],#0x10
    ldp x28, x29, [sp],#0x10
    ldp x30, xzr, [sp],#0x10
    check_vector_size synch_a64

vector_entry irq_a64
    b unknown_exception
    check_vector_size irq_a64

vector_entry fiq_a64
    b unknown_exception
    check_vector_size fiq_a64

vector_entry serror_a64
    b unknown_exception
    check_vector_size serror_a64

/* Lower EL, A32 */
vector_entry synch_a32
    b unknown_exception
    check_vector_size synch_a32

vector_entry irq_a32
    b unknown_exception
    check_vector_size irq_a32

vector_entry fiq_a32
    b unknown_exception
    check_vector_size fiq_a32

vector_entry serror_a32
    b unknown_exception
    .endfunc
    .cfi_endproc
/* To save space, insert in an unused vector segment. */
.global     _handle_synch_exception
.type       _handle_synch_exception, %function
_handle_synch_exception:
    stp x22, x23, [sp, #-0x10]!
    stp x20, x21, [sp, #-0x10]!
    stp x18, x19, [sp, #-0x10]!
    stp x16, x17, [sp, #-0x10]!
    stp x14, x15, [sp, #-0x10]!
    stp x12, x13, [sp, #-0x10]!
    stp x10, x11, [sp, #-0x10]!
    stp x8, x9, [sp, #-0x10]!
    stp x6, x7, [sp, #-0x10]!
    stp x4, x5, [sp, #-0x10]!
    stp x2, x3, [sp, #-0x10]!
    stp x0, x1, [sp, #-0x10]!
    mrs x0, esr_el2
    mov x1, sp
    stp x29, x30, [sp, #-0x10]!
    bl handle_synch_exception
    ldp x29, x30, [sp],#0x10
    ldp x0, x1, [sp],#0x10
    ldp x2, x3, [sp],#0x10
    ldp x4, x5, [sp],#0x10
    ldp x6, x7, [sp],#0x10
    ldp x8, x9, [sp],#0x10
    ldp x10, x11, [sp],#0x10
    ldp x12, x13, [sp],#0x10
    ldp x14, x15, [sp],#0x10
    ldp x16, x17, [sp],#0x10
    ldp x18, x19, [sp],#0x10
    ldp x20, x21, [sp],#0x10
    ldp x22, x23, [sp],#0x10
    ret

