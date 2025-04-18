﻿param([String]$debugfile = "");

# This powershell file has been generated by the IAR Embedded Workbench
# C - SPY Debugger, as an aid to preparing a command line for running
# the cspybat command line utility using the appropriate settings.
#
# Note that this file is generated every time a new debug session
# is initialized, so you may want to move or rename the file before
# making changes.
#
# You can launch cspybat by typing Powershell.exe -File followed by the name of this batch file, followed
# by the name of the debug file (usually an ELF / DWARF or UBROF file).
#
# Read about available command line parameters in the C - SPY Debugging
# Guide. Hints about additional command line parameters that may be
# useful in specific cases :
#   --download_only   Downloads a code image without starting a debug
#                     session afterwards.
#   --silent          Omits the sign - on message.
#   --timeout         Limits the maximum allowed execution time.
#


if ($debugfile -eq "")
{
& "C:\Program Files\IAR Systems\Embedded Workbench 9.0\common\bin\cspybat" -f "D:\Firmware\Olimpia Firmware original\Peripheral BLE\examples\ble_peripheral\ble_app_uart_v2\pca10040\s132\iar\settings\ble_app_uart_pca10040_s132.nrf52832_xxaa.general.xcl" --backend -f "D:\Firmware\Olimpia Firmware original\Peripheral BLE\examples\ble_peripheral\ble_app_uart_v2\pca10040\s132\iar\settings\ble_app_uart_pca10040_s132.nrf52832_xxaa.driver.xcl" 
}
else
{
& "C:\Program Files\IAR Systems\Embedded Workbench 9.0\common\bin\cspybat" -f "D:\Firmware\Olimpia Firmware original\Peripheral BLE\examples\ble_peripheral\ble_app_uart_v2\pca10040\s132\iar\settings\ble_app_uart_pca10040_s132.nrf52832_xxaa.general.xcl" --debug_file=$debugfile --backend -f "D:\Firmware\Olimpia Firmware original\Peripheral BLE\examples\ble_peripheral\ble_app_uart_v2\pca10040\s132\iar\settings\ble_app_uart_pca10040_s132.nrf52832_xxaa.driver.xcl" 
}
