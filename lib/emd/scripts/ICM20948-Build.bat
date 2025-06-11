if "%ATSTUDIO_HOME%"=="" set "ATSTUDIO_HOME=C:\Program Files (x86)\Atmel\Studio\7.0"
set "EMD=ICM20948"

"%ATSTUDIO_HOME%\shellUtils\make.exe" -C "EMD-Core" -f  "Makefile.%EMD%" INVN_EMD_SDK_PATH=.\ SHELL=cmd clean all DEBUG=no 1> scripts\%EMD%-EMDCore.txt 2>&1 || exit /b 1
"%ATSTUDIO_HOME%\shellUtils\make.exe" -C "EMD-App" -f "EMD-App-common.mk" OUTPUT=EMD-App-%EMD% SHELL=cmd rebuild DEBUG=no 1> scripts\%EMD%-EMDApp.txt 2>&1 || exit /b 1

xcopy /Q /Y EMD-App\Release\EMD-App-%EMD%.bin release\%EMD%\ 1>NUL || exit /b 1
xcopy /Q /Y EMD-App\Release\EMD-App-%EMD%.elf release\%EMD%\ 1>NUL || exit /b 1