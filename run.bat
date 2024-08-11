cd 101_A7/build

SET NAME=RayTracing

C:/SeniorPrograming/VS2017/IDE/MSBuild/15.0/Bin/MSBuild.exe %NAME%.sln /p:Configuration=Debug /p:Platform=x64 /target:%NAME%

Debug\%NAME%.exe

 