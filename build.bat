cd 101_A3

::Check if the folder exists

powershell -Command "if (Test-Path -Path 'build') { Remove-Item -Path 'build' -Recurse -Force }; New-Item -Path . -Name 'build' -ItemType 'Directory'"

cd build

cmake -DCMAKE_MODULE_PATH="C:/SeniorPrograming/CExtensionLib/cmakeLib/eigen-3.4.0/cmake" ^
    -DEIGEN3_INCLUDE_DIR="C:/SeniorPrograming/CExtensionLib/cmakeLib/eigen-3.4.0" ^
    -DOpenCV_DIR="C:/SeniorPrograming/CExtensionLib/cmakeLib/opencv/build/x64/vc15/lib" ^
    -A x64  ..
