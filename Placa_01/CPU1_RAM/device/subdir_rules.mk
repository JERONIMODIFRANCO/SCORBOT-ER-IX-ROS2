################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
device/F2837xD_CodeStartBranch.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/source/F2837xD_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/F2837xD_DefaultISR.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/source/F2837xD_DefaultISR.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/%.obj: ../device/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/F2837xD_GlobalVariableDefs.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/source/F2837xD_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/F2837xD_Gpio.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/source/F2837xD_Gpio.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/F2837xD_Ipc.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/source/F2837xD_Ipc.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/F2837xD_PieCtrl.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/source/F2837xD_PieCtrl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/F2837xD_PieVect.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/source/F2837xD_PieVect.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/F2837xD_SysCtrl.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/source/F2837xD_SysCtrl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

device/F2837xD_usDelay.obj: C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/source/F2837xD_usDelay.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/common/include" --include_path="C:/ti/c2000_v5/C2000Ware_5_01_00_00/device_support/f2837xd/headers/include" -g --define=CPU1 --define=_LAUNCHXL_F28379D --display_error_number --diag_suppress=10063 --diag_warning=225 --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


