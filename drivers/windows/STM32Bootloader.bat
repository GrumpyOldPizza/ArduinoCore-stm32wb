::echo off

@echo off
setlocal

if exist "%windir%\sysnative\pnputil.exe" (
    %windir%\sysnative\pnputil.exe /add-driver %0\..\STM32Bootloader.inf /install
) else (
    pnputil /add-driver %0\..\STM32Bootloader.inf /install
)

endlocal