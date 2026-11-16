###############################################################################
# MAX32655FTHR Water Pipe Leakage Detection Project
# Acoustic Leak Detection with Onboard MEMS Microphone + BLE (LightBlue)
###############################################################################

# Target MCU and Board
TARGET = MAX32655
BOARD = FTHR_Apps_P1

# Enable Cordio BLE Library
LIB_CORDIO = 1

# Cordio Library Configuration
INIT_PERIPHERAL = 1
INIT_CENTRAL = 0

# Serial console trace output over DAPLink UART0 (115200 8-N-1)
TRACE = 1

# Services include paths
IPATH += services
VPATH += services

# Bluetooth Device Advertising Name (displayed in LightBlue App)
ADV_NAME ?= PIPE_LEAK_DET
PROJ_CFLAGS += -DADV_NAME=\"$(ADV_NAME)\"

# Configure security: FALSE enables instant connection from LightBlue without PIN
PROJ_CFLAGS += -DINIT_SECURITY=FALSE

# Target board definition
PROJ_CFLAGS += -DBOARD_FTHR_APPS_P1=1

