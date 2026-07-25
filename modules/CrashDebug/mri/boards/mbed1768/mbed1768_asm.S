/*  Copyright (c) 2011 ARM Limited.
 
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
 
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
 
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
/* The __mriDisableMbed() and __mriGetMbedUid() routines were extracted from
   the mbed SDK libraries to  allow the use of the MRI debugger on mbed devices
   without requiring the mbed SDK.  The assembly language code itself
   was written by the mbed team at ARM Limited but the function naming and 
   commenting were done by Adam Green (http://mbed.org/users/AdamGreen/) in
   2012.
*/
    .text
    .code 16
    .syntax unified
    
    .global __mriDisableMbed
    .thumb_func
    /* extern "C" int __mriDisableMbed(void);
       Attempts to disable the debug interface on the mbed device.
    
       Returns -1 on failure.
    */
__mriDisableMbed:
    sub     sp, #8          // Allocate 4 byte parameter block on stack (round up to maintain 8 byte alignment.)
    add     r1, sp, #4      // R1 = pointer to 4 byte parameter block on stack.
    movw    r0, #261        // R0 = semi-host id for disabling JTAG port use from mbed interface chip.
    bkpt	0xab            // Semi-host breakpoint.
    cmp     r0, #0          // Was the call successful (returned 0 in R0)?
    ite     eq              
    ldreq   r0, [sp, #4]    //  If so load R0 with the first word from parameter block.
    movne.w r0, #-1         //  If not then set R0 to -1.
    add     sp, #8          // Restore stack pointer
    bx      lr              //  and return to caller.


    .global __mriGetMbedUid
    .thumb_func
    /* extern "C" int __mriGetMbedUid(uint8_t* pOutputBuffer);
       Reads the unique device id from the mbed interface chip.
       
       Returns -1 on failure.
    */
__mriGetMbedUid:
    sub     sp, #8          // Allocate 8 byte parameter block on stack.
    str     r0, [sp, #0]    // param[0] = pOutputBuffer  Pointer to output buffer for UID.
    movs    r0, #33
    str     r0, [sp, #4]    // param[1] = 33  Size of UID output buffer in bytes.
    mov     r1, sp          // R1 = pointer to 8 byte parameter block on stack.
    movw    r0, #257        // R0 = semi-host id for fetching mbed UID.
    bkpt    0xab            // Semi-host breakpoint.
    add     sp, #8          // Restore stack pointer
    bx      lr              //  and return to caller.
