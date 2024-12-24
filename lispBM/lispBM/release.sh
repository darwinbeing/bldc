#!/bin/bash

release=$1

if [ -z "$1" ]; then
    echo "No arg"
    exit 1
fi
   

reportdir=./test_reports/version_$release

echo $release
echo $reportdir

mkdir -p $reportdir
mkdir -p $reportdir/scan-build
mkdir -p $reportdir/cppcheck

release_readme="readme_${release}.md"

echo "# LispBM Release ${release} Test logs" > $reportdir/$release_readme
echo "" >> $reportdir/$release_readme

cd repl

cppcheck32log="../${reportdir}/cppcheck/cppcheck_32bit_${release}.txt"
cppcheck64log="../${reportdir}/cppcheck/cppcheck_64bit_${release}.txt"

./run_cppcheck.sh $cppcheck32log $cppcheck64log

cd ..

cd tests

unit_tests_log_file="unit_tests_log_${release}.txt"
failing_unit_tests_log_file="failing_unit_tests_log_${release}.txt"

unit_tests_64_log_file="unit_tests_log_64_${release}.txt"
failing_unit_tests_64_log_file="failing_unit_tests_log_64_${release}.txt"

gc_unit_tests_log_file="gc_unit_tests_log_${release}.txt"
failing_gc_unit_tests_log_file="failing_gc_unit_tests_log_${release}.txt"

revgc_unit_tests_log_file="revgc_unit_tests_log_${release}.txt"
failing_revgc_unit_tests_log_file="failing_revgc_unit_tests_log_${release}.txt"

############################################################
# 32bit tests 
./run_tests.sh ../$reportdir/$failing_unit_tests_log_file >> ../$reportdir/$unit_tests_log_file
echo "" >> ../$reportdir/$release_readme
echo "## 32BIT UNIT TESTS RESULTS" >> ../$reportdir/$release_readme
tail -n 4 ../$reportdir/$unit_tests_log_file >> ../$reportdir/$release_readme

############################################################
# 64bit tests
./run_tests64.sh ../$reportdir/$failing_unit_tests_64_log_file >> ../$reportdir/$unit_tests_64_log_file
echo "" >> ../$reportdir/$release_readme
echo "## 64BIT UNIT TESTS RESULTS" >> ../$reportdir/$release_readme
tail -n 4 ../$reportdir/$unit_tests_64_log_file >> ../$reportdir/$release_readme 

############################################################
# Always GC tests
./run_tests_gc.sh ../$reportdir/$failing_gc_unit_tests_log_file >> ../$reportdir/$gc_unit_tests_log_file
echo "" >> ../$reportdir/$release_readme
echo "## ALWAYS GC UNIT TESTS RESULTS" >> ../$reportdir/$release_readme
tail -n 4 ../$reportdir/$gc_unit_tests_log_file >> ../$reportdir/$release_readme

############################################################
#Pointer reversal gc tests
./run_tests_gc_rev.sh ../$reportdir/$failing_revgc_unit_tests_log_file >> ../$reportdir/$revgc_unit_tests_log_file
echo "" >> ../$reportdir/$release_readme
echo "## POINTER REVERSAL GC UNIT TESTS RESULTS" >> ../$reportdir/$release_readme
tail -n 4 ../$reportdir/$revgc_unit_tests_log_file >> ../$reportdir/$release_readme 

############################################################
# Run the 32bit tests for a coverage report.
./run_tests_cov.sh


cd ..
cp -r tests/coverage $reportdir/coverage

if ! command -v scan-build-14 &> /dev/null
then
    if ! command -v scan-build-10 &> /dev/null
    then
    
        echo "scan-build could not be found"
        exit 1
    else
        make clean
        scan-build-10 -o ./$reportdir/scan-build make -j4 >> ./$reportdir/scan_build_$release.txt
    fi
else
    make clean
    scan-build-14 -o ./$reportdir/scan-build make -j4 >> ./$reportdir/scan_build_$release.txt
fi

echo "" >> ./$reportdir/$release_readme
echo "## SCAN BUILD" >> ./$reportdir/$release_readme
tail -n 1 $reportdir/scan_build_$release.txt >> ./$reportdir/$release_readme 


make clean 
infer run -- make

cp ./infer-out/report.txt $reportdir/infer_${release}.txt

echo "" >> ./$reportdir/$release_readme
echo "## INFER ISSUES" >> ./$reportdir/$release_readme
tail -n 3 $reportdir/infer_${release}.txt >> ./$reportdir/$release_readme
