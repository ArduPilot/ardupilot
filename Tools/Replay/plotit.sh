#!/usr/bin/env bash

cmd="plot "
echo $#
while [ $# -gt 1 ]; do
    if [[ "$1" == *.* ]]; then
        cmd="$cmd 'plot.dat' using 1:'$1',"
    else
        cmd="$cmd 'plot2.dat' using 1:'$1',"
    fi
    shift
done
if [[ "$1" == *.* ]]; then
    cmd="$cmd 'plot.dat' using 1:'$1'"
else
    cmd="$cmd 'plot2.dat' using 1:'$1'"
fi
echo $cmd
cat <<EOF > _plot.gnu
set style data lines
set xlabel "time(s)"
$cmd
pause -1 "hit return to exit"
EOF
gnuplot _plot.gnu

