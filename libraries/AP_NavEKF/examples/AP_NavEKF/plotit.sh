#!/bin/sh

cmd="plot "
echo $#
while [ $# -gt 1 ]; do
    cmd="$cmd 'plot.dat' using 1:'$1',"
    shift
done
cmd="$cmd 'plot.dat' using 1:'$1'"
cat <<EOF > _plot.gnu
set style data lines
set xlabel "time(s)"
$cmd
pause -1 "hit return to exit"
EOF
gnuplot _plot.gnu

