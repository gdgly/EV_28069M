################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../F28069.cmd 

LIB_SRCS += \
../sw/modules/fast/lib/32b/f28x/f2806x/2806xRevB_FastSpinROMSymbols.lib \
../sw/modules/iqmath/lib/f28x/32b/IQmath.lib 

ASM_SRCS += \
../sw/drivers/cpu/src/32b/f28x/f2806x/CodeStartBranch.asm \
../sw/modules/usDelay/src/32b/f28x/usDelay.asm 

C_SRCS += \
../sw/Sources/Main.c \
../sw/drivers/adc/src/32b/f28x/f2806x/adc.c \
../sw/modules/clarke/src/32b/clarke.c \
../sw/drivers/clk/src/32b/f28x/f2806x/clk.c \
../sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c \
../sw/modules/ctrl/src/32b/ctrlQEP.c \
../sw/modules/datalog/src/32b/datalog.c \
../sw/drivers/drvic/drv8305/src/32b/f28x/f2806x/drv8305.c \
../sw/modules/enc/src/32b/enc.c \
../sw/modules/filter/src/32b/filter_fo.c \
../sw/drivers/flash/src/32b/f28x/f2806x/flash.c \
../sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c \
../sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/hal.c \
../sw/modules/hallbldc/src/32b/hallbldc.c \
../sw/drivers/i2c/src/32b/f28x/f2806x/i2c.c \
../sw/Sources/i2c_24lc32.c \
../sw/Sources/i2c_mcp23017.c \
../sw/Sources/i2c_message.c \
../sw/modules/ipark/src/32b/ipark.c \
../sw/modules/memCopy/src/memCopy.c \
../sw/modules/offset/src/32b/offset.c \
../sw/drivers/osc/src/32b/f28x/f2806x/osc.c \
../sw/modules/park/src/32b/park.c \
../sw/modules/pid/src/32b/pid.c \
../sw/drivers/pie/src/32b/f28x/f2806x/pie.c \
../sw/drivers/pll/src/32b/f28x/f2806x/pll.c \
../sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c \
../sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c \
../sw/drivers/qep/src/32b/f28x/f2806x/qep.c \
../sw/drivers/sci/src/32b/f28x/f2806x/sci.c \
../sw/Sources/sci_message.c \
../sw/Sources/sci_modbus.c \
../sw/Sources/sci_operator.c \
../sw/modules/slip/src/32b/slip.c \
../sw/drivers/spi/src/32b/f28x/f2806x/spi.c \
../sw/modules/svgen/src/32b/svgen.c \
../sw/modules/throttle/src/32b/throttle.c \
../sw/drivers/timer/src/32b/f28x/f2806x/timer.c \
../sw/modules/traj/src/32b/traj.c \
../sw/modules/user/src/32b/user.c \
../sw/Sources/user_data.c \
../sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c 

C_DEPS += \
./Main.d \
./adc.d \
./clarke.d \
./clk.d \
./cpu.d \
./ctrlQEP.d \
./datalog.d \
./drv8305.d \
./enc.d \
./filter_fo.d \
./flash.d \
./gpio.d \
./hal.d \
./hallbldc.d \
./i2c.d \
./i2c_24lc32.d \
./i2c_mcp23017.d \
./i2c_message.d \
./ipark.d \
./memCopy.d \
./offset.d \
./osc.d \
./park.d \
./pid.d \
./pie.d \
./pll.d \
./pwm.d \
./pwr.d \
./qep.d \
./sci.d \
./sci_message.d \
./sci_modbus.d \
./sci_operator.d \
./slip.d \
./spi.d \
./svgen.d \
./throttle.d \
./timer.d \
./traj.d \
./user.d \
./user_data.d \
./wdog.d 

OBJS += \
./CodeStartBranch.obj \
./Main.obj \
./adc.obj \
./clarke.obj \
./clk.obj \
./cpu.obj \
./ctrlQEP.obj \
./datalog.obj \
./drv8305.obj \
./enc.obj \
./filter_fo.obj \
./flash.obj \
./gpio.obj \
./hal.obj \
./hallbldc.obj \
./i2c.obj \
./i2c_24lc32.obj \
./i2c_mcp23017.obj \
./i2c_message.obj \
./ipark.obj \
./memCopy.obj \
./offset.obj \
./osc.obj \
./park.obj \
./pid.obj \
./pie.obj \
./pll.obj \
./pwm.obj \
./pwr.obj \
./qep.obj \
./sci.obj \
./sci_message.obj \
./sci_modbus.obj \
./sci_operator.obj \
./slip.obj \
./spi.obj \
./svgen.obj \
./throttle.obj \
./timer.obj \
./traj.obj \
./usDelay.obj \
./user.obj \
./user_data.obj \
./wdog.obj 

ASM_DEPS += \
./CodeStartBranch.d \
./usDelay.d 

OBJS__QUOTED += \
"CodeStartBranch.obj" \
"Main.obj" \
"adc.obj" \
"clarke.obj" \
"clk.obj" \
"cpu.obj" \
"ctrlQEP.obj" \
"datalog.obj" \
"drv8305.obj" \
"enc.obj" \
"filter_fo.obj" \
"flash.obj" \
"gpio.obj" \
"hal.obj" \
"hallbldc.obj" \
"i2c.obj" \
"i2c_24lc32.obj" \
"i2c_mcp23017.obj" \
"i2c_message.obj" \
"ipark.obj" \
"memCopy.obj" \
"offset.obj" \
"osc.obj" \
"park.obj" \
"pid.obj" \
"pie.obj" \
"pll.obj" \
"pwm.obj" \
"pwr.obj" \
"qep.obj" \
"sci.obj" \
"sci_message.obj" \
"sci_modbus.obj" \
"sci_operator.obj" \
"slip.obj" \
"spi.obj" \
"svgen.obj" \
"throttle.obj" \
"timer.obj" \
"traj.obj" \
"usDelay.obj" \
"user.obj" \
"user_data.obj" \
"wdog.obj" 

C_DEPS__QUOTED += \
"Main.d" \
"adc.d" \
"clarke.d" \
"clk.d" \
"cpu.d" \
"ctrlQEP.d" \
"datalog.d" \
"drv8305.d" \
"enc.d" \
"filter_fo.d" \
"flash.d" \
"gpio.d" \
"hal.d" \
"hallbldc.d" \
"i2c.d" \
"i2c_24lc32.d" \
"i2c_mcp23017.d" \
"i2c_message.d" \
"ipark.d" \
"memCopy.d" \
"offset.d" \
"osc.d" \
"park.d" \
"pid.d" \
"pie.d" \
"pll.d" \
"pwm.d" \
"pwr.d" \
"qep.d" \
"sci.d" \
"sci_message.d" \
"sci_modbus.d" \
"sci_operator.d" \
"slip.d" \
"spi.d" \
"svgen.d" \
"throttle.d" \
"timer.d" \
"traj.d" \
"user.d" \
"user_data.d" \
"wdog.d" 

ASM_DEPS__QUOTED += \
"CodeStartBranch.d" \
"usDelay.d" 

ASM_SRCS__QUOTED += \
"../sw/drivers/cpu/src/32b/f28x/f2806x/CodeStartBranch.asm" \
"../sw/modules/usDelay/src/32b/f28x/usDelay.asm" 

C_SRCS__QUOTED += \
"../sw/Sources/Main.c" \
"../sw/drivers/adc/src/32b/f28x/f2806x/adc.c" \
"../sw/modules/clarke/src/32b/clarke.c" \
"../sw/drivers/clk/src/32b/f28x/f2806x/clk.c" \
"../sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c" \
"../sw/modules/ctrl/src/32b/ctrlQEP.c" \
"../sw/modules/datalog/src/32b/datalog.c" \
"../sw/drivers/drvic/drv8305/src/32b/f28x/f2806x/drv8305.c" \
"../sw/modules/enc/src/32b/enc.c" \
"../sw/modules/filter/src/32b/filter_fo.c" \
"../sw/drivers/flash/src/32b/f28x/f2806x/flash.c" \
"../sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c" \
"../sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/hal.c" \
"../sw/modules/hallbldc/src/32b/hallbldc.c" \
"../sw/drivers/i2c/src/32b/f28x/f2806x/i2c.c" \
"../sw/Sources/i2c_24lc32.c" \
"../sw/Sources/i2c_mcp23017.c" \
"../sw/Sources/i2c_message.c" \
"../sw/modules/ipark/src/32b/ipark.c" \
"../sw/modules/memCopy/src/memCopy.c" \
"../sw/modules/offset/src/32b/offset.c" \
"../sw/drivers/osc/src/32b/f28x/f2806x/osc.c" \
"../sw/modules/park/src/32b/park.c" \
"../sw/modules/pid/src/32b/pid.c" \
"../sw/drivers/pie/src/32b/f28x/f2806x/pie.c" \
"../sw/drivers/pll/src/32b/f28x/f2806x/pll.c" \
"../sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c" \
"../sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c" \
"../sw/drivers/qep/src/32b/f28x/f2806x/qep.c" \
"../sw/drivers/sci/src/32b/f28x/f2806x/sci.c" \
"../sw/Sources/sci_message.c" \
"../sw/Sources/sci_modbus.c" \
"../sw/Sources/sci_operator.c" \
"../sw/modules/slip/src/32b/slip.c" \
"../sw/drivers/spi/src/32b/f28x/f2806x/spi.c" \
"../sw/modules/svgen/src/32b/svgen.c" \
"../sw/modules/throttle/src/32b/throttle.c" \
"../sw/drivers/timer/src/32b/f28x/f2806x/timer.c" \
"../sw/modules/traj/src/32b/traj.c" \
"../sw/modules/user/src/32b/user.c" \
"../sw/Sources/user_data.c" \
"../sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c" 


