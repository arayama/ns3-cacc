#!/bin/sh

# program to interpolate acceleration profile 
# usage : program filename <num to interpl between each> outfile
# returns file containing new accln points 

cat $1 | awk '{print $2}' > tmp.scratch
#cat $1 > tmp.scratch
chunk=$2
out=$3
edge=0.0

echo $(head -n 1 tmp.scratch) > $out

while read line;
do	
	cnt=0
	inter=$(echo "($line - $edge) / $chunk" | bc)
	while [ $cnt -le $((chunk-1)) ];
	do 
		edge=$(echo "$edge + $inter" | bc)  
		echo $edge >> $out
		cnt=$((cnt +1))
	done
	edge=$( echo $line)
done < tmp.scratch

echo "done!!"
echo "first file length $(cat $1 | wc -l)" 
echo "second file length $(cat $out | wc -l)"
exit
