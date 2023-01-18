echo "Alpha: $1";
echo "Case : $2";

rm -rf build/
cmake -S . -B build/ -DBUILD_EXAMPLES=ON
cmake --build build/ -j4
echo "> [Build OK]";
./build/bin/Lab2 $1 "data/$2.block" "data/$2.nets" "data/output/output2_$2.rpt"
echo "> [Run OK]";
./verifier/verifier $1 "data/$2.block" "data/$2.nets" "data/output/output2_$2.rpt"
python3 ./verifier/floorplan.py data/output/output2_$2.rpt out.png
