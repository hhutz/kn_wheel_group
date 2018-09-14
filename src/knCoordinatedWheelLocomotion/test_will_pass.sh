
# Update ckSumValue after making changes to test_Fsd or test_FsdMatrix
export ckSumValue="672141110"

export tmpDataDir="tmp/data"
mkdir -p $tmpDataDir
cd $tmpDataDir
../../test_Fsd
../../test_FsdMatrix
cat pathVars_*.m > ../regression.txt
cd ../..
#rm -rf $tmpDataDir

./test_TrapezoidProfile && [ `cksum tmp/regression.txt | cut -d" " -f1` -eq $ckSumValue ]

exit $STATUS
