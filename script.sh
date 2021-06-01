#!/bin/bash

#PARAM généraux
inputValues='resultat.txt'
outputFile="graphe2.png"
grapheTitle="Nombre de Fps dans le temps avec changement toutes les 10 secondes"
xlabel="Milliseconde"
ylabel="Nombre de Fps"



aMin=($(sort -g -k 1 $inputValues | sed -n '1p;$p' | cut -d' ' -f1))
aMax=($(sort -g -k 1 -r $inputValues | sed -n '1p;$p' | cut -d' ' -f1))

# min et max pour l'abscice
xmin=0
xmax=$aMax
 
bMin=($(sort -g -k 2 $inputValues | sed -n '1p;$p' | cut -d' ' -f2))
bMax=($(sort -g -k 2 -r $inputValues | sed -n '1p;$p' | cut -d' ' -f2))

# min et max pour l'ordonnée
ymin=0
ymax=$bMax+1


#PARAM médiane
mediane=1 # 1 = true(activé) / 0 = false(desactivé)
yminMed=10
ymaxMed=12
xminMed=$xmin
xmaxMed=$xmax
if [ $mediane = 1 ]; then
  echo "$xminMed $yminMed
$xmaxMed $ymaxMed"> mediane.txt
fi



#echo "$aMin"
#echo "$aMax"
#echo "$bMin"
#echo "$bMax"


# SCRIPT gnuplot
# NE PAS EDITER a partir d'ici
  
echo "set terminal png size 800,500 enhanced background rgb 'white'" > gnuplot_script
echo "set style line 1 lt 1 lw 1.5 pt 3 linecolor rgb '#ff00ff'" >> gnuplot_script
echo "set output '$outputFile'" >> gnuplot_script
echo "set autoscale" >> gnuplot_script
echo "set xtic auto" >> gnuplot_script
echo "set ytic auto" >> gnuplot_script 
echo "set title '$grapheTitle'" >> gnuplot_script
echo "set ylabel '$ylabel'" >> gnuplot_script
echo "set xlabel '$xlabel'" >> gnuplot_script
echo "set xr [$xmin:$xmax]" >> gnuplot_script
echo "set yr [$ymin:$ymax]" >> gnuplot_script
echo "set nokey" >> gnuplot_script
echo "set multiplot" >> gnuplot_script
echo "plot '$inputValues' with lines lc rgb \"blue\" " >> gnuplot_script
if [ $mediane = 1 ]; then
  echo "plot 'mediane.txt' with lines lc rgb \"red\"" >> gnuplot_script
fi
gnuplot gnuplot_script 
rm gnuplot_script 
