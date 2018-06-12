define i2c_sr1_flags
set $s = $arg0
printf "SR1: "

if (($s & (1 << 15)))
    printf "SMBALERT "
end

if (($s & (1 << 14)))
    printf "TIMEOUT "
end

if (($s & (1 << 12)))
    printf "PECERR "
end

if (($s & (1 << 11)))
    printf "OVR "
end

if (($s & (1 << 10)))
    printf "AF "
end

if (($s & (1 << 9)))
    printf "ARLO "
end

if (($s & (1 << 8)))
    printf "BERR "
end

if (($s & (1 << 7)))
    printf "TXE "
end

if (($s & (1 << 6)))
    printf "RXNE "
end

if (($s & (1 << 4)))
    printf "STOPF "
end

if (($s & (1 << 3)))
    printf "ADD10 "
end

if (($s & (1 << 2)))
    printf "BTF "
end

if (($s & (1 << 1)))
    printf "ADDR "
end

if (($s & (1 << 0)))
    printf "SB "
end
end

define i2c_sr2_flags
set $s = $arg0
printf "SR2: "

if (($s & (1 << 7)))
    printf "DUALF "
end

if (($s & (1 << 6)))
    printf "SMBHOST "
end

if (($s & (1 << 5)))
    printf "SMBDEFAULT "
end

if (($s & (1 << 4)))
    printf "GENCALL "
end


if (($s & (1 << 2)))
    printf "TRA "
end

if (($s & (1 << 1)))
    printf "BUSY "
end

if (($s & (1 << 0)))
    printf "MSL "
end

end

define pbc
set $c = crumbs
while ($c->event)
    if ($c->event != 0)
        printf "Event: %d ", $c->event
        if ($c->event == 1)
            i2c_sr1_flags $c->sr1
            printf "\t"
            i2c_sr2_flags $c->sr2
        end
        printf "\n"
    end
    set $c = $c + 1
end


