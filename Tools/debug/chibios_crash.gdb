set pagination off

define chibios-thread-bt
    set $ctx = $arg0->ctx.sp
    set $r4 = $ctx->r4
    set $r5 = $ctx->r5
    set $r6 = $ctx->r6
    set $r7 = $ctx->r7
    set $r8 = $ctx->r8
    set $r9 = $ctx->r9
    set $r10 = $ctx->r10
    set $r11 = $ctx->r11
    set $lr = $ctx->lr
    set $sp = $ctx + 1
    set $pc = $ctx->lr
    bt 12
end

define chibios-crash-threads
    set $crash_r0 = $r0
    set $crash_r1 = $r1
    set $crash_r2 = $r2
    set $crash_r3 = $r3
    set $crash_r4 = $r4
    set $crash_r5 = $r5
    set $crash_r6 = $r6
    set $crash_r7 = $r7
    set $crash_r8 = $r8
    set $crash_r9 = $r9
    set $crash_r10 = $r10
    set $crash_r11 = $r11
    set $crash_r12 = $r12
    set $crash_sp = $sp
    set $crash_lr = $lr
    set $crash_pc = $pc
    set $crash_xpsr = $xpsr
    set $tp = &ch0.mainthread
    set $registry = &ch0.reglist.queue
    set $count = 0
    while $tp != 0
        printf "\nChibiOS thread %u: %s, state=%u, tp=%p, sp=%p\n", $count, $tp->name, $tp->state, $tp, $tp->ctx.sp
        if $tp == ch0.rlist.current
            printf "Current thread uses the CrashCatcher register context\n"
            set $r0 = $crash_r0
            set $r1 = $crash_r1
            set $r2 = $crash_r2
            set $r3 = $crash_r3
            set $r4 = $crash_r4
            set $r5 = $crash_r5
            set $r6 = $crash_r6
            set $r7 = $crash_r7
            set $r8 = $crash_r8
            set $r9 = $crash_r9
            set $r10 = $crash_r10
            set $r11 = $crash_r11
            set $r12 = $crash_r12
            set $sp = $crash_sp
            set $lr = $crash_lr
            set $pc = $crash_pc
            set $xpsr = $crash_xpsr
            bt 12
        else
            chibios-thread-bt $tp
        end
        set $nextq = $tp->rqueue.next
        if $nextq == $registry
            set $tp = 0
        else
            set $tp = (thread_t *)((char *)$nextq - (char *)&((thread_t *)0)->rqueue)
        end
        set $count = $count + 1
    end
    set $r0 = $crash_r0
    set $r1 = $crash_r1
    set $r2 = $crash_r2
    set $r3 = $crash_r3
    set $r4 = $crash_r4
    set $r5 = $crash_r5
    set $r6 = $crash_r6
    set $r7 = $crash_r7
    set $r8 = $crash_r8
    set $r9 = $crash_r9
    set $r10 = $crash_r10
    set $r11 = $crash_r11
    set $r12 = $crash_r12
    set $sp = $crash_sp
    set $lr = $crash_lr
    set $pc = $crash_pc
    set $xpsr = $crash_xpsr
    printf "\nChibiOS thread count: %u\n", $count
end
