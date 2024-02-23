################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./Comunicaciones.obj" \
"./Entradas_Salidas.obj" \
"./Example_EPwmSetup.obj" \
"./Homming.obj" \
"./SetParametrosPID.obj" \
"./adc_motor_1_2_3.obj" \
"./control.obj" \
"./eQEP_motor_1_2_3.obj" \
"./init_PWM_motor_1_2_3.obj" \
"./main.obj" \
"./muestreo.obj" \
"./protecciones.obj" \
"./seteo_salidas.obj" \
"./device/F2837xD_CodeStartBranch.obj" \
"./device/F2837xD_DefaultISR.obj" \
"./device/F2837xD_EPwm.obj" \
"./device/F2837xD_EQep.obj" \
"./device/F2837xD_GlobalVariableDefs.obj" \
"./device/F2837xD_Gpio.obj" \
"./device/F2837xD_Ipc.obj" \
"./device/F2837xD_PieCtrl.obj" \
"./device/F2837xD_PieVect.obj" \
"./device/F2837xD_SysCtrl.obj" \
"./device/F2837xD_usDelay.obj" \
$(GEN_CMDS__FLAG) \
-lrts2800_fpu32.lib \
-l2837xD_RAM_lnk_cpu1.cmd \
-lF2837xD_Headers_nonBIOS_cpu1.cmd \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include device/subdir_vars.mk
-include subdir_rules.mk
-include device/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
blinky_cpu1.out \

EXE_OUTPUTS__QUOTED += \
"blinky_cpu1.out" \


# All Target
all: blinky_cpu1.out

# Tool invocations
blinky_cpu1.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 -z -m"blinky_cpu1.map" --stack_size=0x100 --warn_sections -i"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/lib" -i"C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" -i"C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -i"C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/lib" -i"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" -i"C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/cmd" -i"C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/cmd" --reread_libs --display_error_number --xml_link_info="blinky_cpu1_linkInfo.xml" --entry_point=code_start --rom_model -o "blinky_cpu1.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "Comunicaciones.obj" "Entradas_Salidas.obj" "Example_EPwmSetup.obj" "Homming.obj" "SetParametrosPID.obj" "adc_motor_1_2_3.obj" "control.obj" "eQEP_motor_1_2_3.obj" "init_PWM_motor_1_2_3.obj" "main.obj" "muestreo.obj" "protecciones.obj" "seteo_salidas.obj" "device\F2837xD_CodeStartBranch.obj" "device\F2837xD_DefaultISR.obj" "device\F2837xD_EPwm.obj" "device\F2837xD_EQep.obj" "device\F2837xD_GlobalVariableDefs.obj" "device\F2837xD_Gpio.obj" "device\F2837xD_Ipc.obj" "device\F2837xD_PieCtrl.obj" "device\F2837xD_PieVect.obj" "device\F2837xD_SysCtrl.obj" "device\F2837xD_usDelay.obj" 
	-$(RM) "Comunicaciones.d" "Entradas_Salidas.d" "Example_EPwmSetup.d" "Homming.d" "SetParametrosPID.d" "adc_motor_1_2_3.d" "control.d" "eQEP_motor_1_2_3.d" "init_PWM_motor_1_2_3.d" "main.d" "muestreo.d" "protecciones.d" "seteo_salidas.d" "device\F2837xD_DefaultISR.d" "device\F2837xD_EPwm.d" "device\F2837xD_EQep.d" "device\F2837xD_GlobalVariableDefs.d" "device\F2837xD_Gpio.d" "device\F2837xD_Ipc.d" "device\F2837xD_PieCtrl.d" "device\F2837xD_PieVect.d" "device\F2837xD_SysCtrl.d" 
	-$(RM) "device\F2837xD_CodeStartBranch.d" "device\F2837xD_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

