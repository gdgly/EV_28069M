################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
CodeStartBranch.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/cpu/src/32b/f28x/f2806x/CodeStartBranch.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="CodeStartBranch.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Main.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/Main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="Main.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

adc.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/adc/src/32b/f28x/f2806x/adc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="adc.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

clarke.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/clarke/src/32b/clarke.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="clarke.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

clk.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/clk/src/32b/f28x/f2806x/clk.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="clk.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

cpu.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="cpu.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ctrlQEP.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/ctrl/src/32b/ctrlQEP.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="ctrlQEP.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

drv8305.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/drvic/drv8305/src/32b/f28x/f2806x/drv8305.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="drv8305.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

enc.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/enc/src/32b/enc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="enc.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

filter_fo.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/filter/src/32b/filter_fo.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="filter_fo.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

flash.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/flash/src/32b/f28x/f2806x/flash.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="flash.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

gpio.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="gpio.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

hal.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/hal.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="hal.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

hallbldc.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/hallbldc/src/32b/hallbldc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="hallbldc.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

i2c.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/i2c/src/32b/f28x/f2806x/i2c.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="i2c.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

i2c_24lc32.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/i2c_24lc32.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="i2c_24lc32.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

i2c_lcd.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/i2c_lcd.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="i2c_lcd.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

i2c_mcp23017.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/i2c_mcp23017.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="i2c_mcp23017.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

i2c_message.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/i2c_message.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="i2c_message.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ipark.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/ipark/src/32b/ipark.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="ipark.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

memCopy.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/memCopy/src/memCopy.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="memCopy.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

offset.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/offset/src/32b/offset.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="offset.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

osc.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/osc/src/32b/f28x/f2806x/osc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="osc.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

park.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/park/src/32b/park.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="park.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

pid.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/pid/src/32b/pid.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="pid.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

pie.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/pie/src/32b/f28x/f2806x/pie.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="pie.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

pll.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/pll/src/32b/f28x/f2806x/pll.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="pll.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

pwm.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="pwm.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

pwr.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="pwr.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

qep.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/qep/src/32b/f28x/f2806x/qep.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="qep.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

sci.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/sci/src/32b/f28x/f2806x/sci.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="sci.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

sci_message.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/sci_message.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="sci_message.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

sci_modbus.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/sci_modbus.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="sci_modbus.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

sci_operator.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/sci_operator.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="sci_operator.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

slip.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/slip/src/32b/slip.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="slip.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

spi.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/spi/src/32b/f28x/f2806x/spi.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="spi.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

svgen.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/svgen/src/32b/svgen.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="svgen.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

throttle.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/throttle/src/32b/throttle.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="throttle.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

timer.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/timer/src/32b/f28x/f2806x/timer.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="timer.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

traj.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/traj/src/32b/traj.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="traj.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

usDelay.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/usDelay/src/32b/f28x/usDelay.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="usDelay.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

user.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/modules/user/src/32b/user.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="user.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

user_data.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/Sources/user_data.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="user_data.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

wdog.obj: C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M/sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=softlib -O2 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/solutions/instaspin_motion/boards/drv8312kit_revD/f28x/f2806xM/projects/ccs/EV_28069M" --define=QEP --define=FLASH --define=FAST_ROM_V1p6 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="wdog.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


