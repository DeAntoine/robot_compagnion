#!/bin/bash
 
a=($(sort -g test5 | sed -n '1p;$p' | cut -d' ' -f2))
ymin=-200
ymax=200
   
b=($(sort -k1 -g test5 | sed -n '1p;$p' | cut -d' ' -f1))
xmin=0
xmax=800
     
echo "set terminal png size 3000,800 enhanced background rgb 'white'" > gnuplot_script
echo "set style line 1 lt 1 lw 1.5 pt 3 linecolor rgb '#ff00ff'" >> gnuplot_script
echo "set output 'graphe8.png'" >> gnuplot_script
echo "set autoscale" >> gnuplot_script
echo "set xtic auto" >> gnuplot_script
echo "set ytic auto" >> gnuplot_script 
echo "set title '$1'" >> gnuplot_script
echo "set ylabel 'Distance en px'" >> gnuplot_script
echo "set xlabel 'Seconde'" >> gnuplot_script
echo "set xr [$xmin:$xmax]" >> gnuplot_script
echo "set yr [$ymin:$ymax]" >> gnuplot_script
echo "set nokey" >> gnuplot_script
echo "set multiplot" >> gnuplot_script
echo "plot 'test5' with linespoints lc rgb \"blue\" " >> gnuplot_script*
echo "plot 'mediane.txt' with lines lc rgb \"red\"" >> gnuplot_script  
gnuplot gnuplot_script
gnuplot gnuplot_script*
rm gnuplot_script
rm gnuplot_script*