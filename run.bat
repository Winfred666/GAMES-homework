cd 101_A8/build

SET NAME=RopeSim

C:/SeniorPrograming/VS2017/IDE/MSBuild/15.0/Bin/MSBuild.exe %NAME%.sln /p:Configuration=Debug /p:Platform=x64 /target:%NAME%

Debug\%NAME%.exe

 