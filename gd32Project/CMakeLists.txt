cmake_minimum_required(VERSION 3.20)
# 项目名称可以随便起
project(Clion_Keil_gd32 C CXX ASM)

# 1. 填入你的全局宏定义 (稍后从 Keil 复制)
add_compile_definitions(
        GD32E50X
        GD32E50X_HD
        USE_STDPERIPH_DRIVER
)

# 2. 填入你的头文件搜索路径 (稍后从 Keil 复制)
# 注意：路径请使用相对于本文件的相对路径，且使用正斜杠 /
include_directories(
        ./Firmware/CMSIS/GD/GD32E50x/Include
        ./Firmware/GD32E50x_standard_peripheral/Include
        ./Firmware/CMSIS
        ./gd32Foc/BSP
        ./gd32Foc/USER
)

# 3. 自动抓取所有的源码文件
# GLOB_RECURSE 会自动扫描你指定文件夹下的所有 .c 和 .h 文件
file(GLOB_RECURSE SOURCES
        "./gd32Foc/BSP/*.c"
        "./gd32Foc/BSP/*.h"
        "./gd32Foc/USER/*.c"
        "./gd32Foc/USER/*.h"
        "./Firmware/GD32E50x_standard_peripheral/*.c"
        "./Firmware/GD32E50x_standard_peripheral/*.h"
        "Firmware/CMSIS/GD/GD32E50x/Source/system_gd32e50x.c"
        "Firmware/CMSIS/GD/GD32E50x/Include/*.h"
        # 如果你有其他存放代码的文件夹，按照这个格式继续往下加，例如：
        # "UserApp/*.c"
)

# 4. 生成一个伪目标，激活 CLion 的代码补全引擎
add_executable(${PROJECT_NAME}.elf ${SOURCES})