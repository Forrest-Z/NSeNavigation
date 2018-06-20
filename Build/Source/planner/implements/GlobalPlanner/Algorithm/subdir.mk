################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/planner/implements/GlobalPlanner/Algorithm/Astar.cpp \
../Source/planner/implements/GlobalPlanner/Algorithm/Dijkstra.cpp \
../Source/planner/implements/GlobalPlanner/Algorithm/GradientPath.cpp \
../Source/planner/implements/GlobalPlanner/Algorithm/OrientationFilter.cpp \
../Source/planner/implements/GlobalPlanner/Algorithm/QuadraticCalculator.cpp 

OBJS += \
./Source/planner/implements/GlobalPlanner/Algorithm/Astar.o \
./Source/planner/implements/GlobalPlanner/Algorithm/Dijkstra.o \
./Source/planner/implements/GlobalPlanner/Algorithm/GradientPath.o \
./Source/planner/implements/GlobalPlanner/Algorithm/OrientationFilter.o \
./Source/planner/implements/GlobalPlanner/Algorithm/QuadraticCalculator.o 

CPP_DEPS += \
./Source/planner/implements/GlobalPlanner/Algorithm/Astar.d \
./Source/planner/implements/GlobalPlanner/Algorithm/Dijkstra.d \
./Source/planner/implements/GlobalPlanner/Algorithm/GradientPath.d \
./Source/planner/implements/GlobalPlanner/Algorithm/OrientationFilter.d \
./Source/planner/implements/GlobalPlanner/Algorithm/QuadraticCalculator.d 


# Each subdirectory must supply rules for building sources it contributes
Source/planner/implements/GlobalPlanner/Algorithm/%.o: ../Source/planner/implements/GlobalPlanner/Algorithm/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -I/root/tina/prebuilt/gcc/linux-x86/arm/toolchain-sunxi/toolchain/include -I$(STAGING_DIR)/usr/include/allwinner/include/ -I$(STAGING_DIR)/usr/include/libsgbot/ -I$(STAGING_DIR)/usr/include/allwinner -I/root/tina/out/astar-parrot/compile_dir/target/seeing-navigation/SeNaviCommon/Source -I/root/tina/out/astar-parrot/staging_dir/target/usr/include/allwinner/include/ -I/root/tina/out/astar-parrot/staging_dir/target/usr/include/libsgbot/ -I/root/tina/out/astar-parrot/staging_dir/target/usr/include/allwinner -O0 -g -Wall -DBOOST_LOG_DYN_LINK -D logLevel=0 -c -fmessage-length=0 -std=gnu++11 -DBOOST_LOG_DYN_LINK -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


