################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/CostmapModel.cpp \
../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/FootprintHelper.cpp \
../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/GoalFunctions.cpp \
../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/MapCell.cpp \
../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/MapGrid.cpp \
../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/OdometryHelper.cpp \
../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/Trajectory.cpp \
../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/TrajectoryPlanner.cpp 

OBJS += \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/CostmapModel.o \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/FootprintHelper.o \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/GoalFunctions.o \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/MapCell.o \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/MapGrid.o \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/OdometryHelper.o \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/Trajectory.o \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/TrajectoryPlanner.o 

CPP_DEPS += \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/CostmapModel.d \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/FootprintHelper.d \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/GoalFunctions.d \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/MapCell.d \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/MapGrid.d \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/OdometryHelper.d \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/Trajectory.d \
./Source/planner/implements/TrajectoryLocalPlanner/Algorithm/TrajectoryPlanner.d 


# Each subdirectory must supply rules for building sources it contributes
Source/planner/implements/TrajectoryLocalPlanner/Algorithm/%.o: ../Source/planner/implements/TrajectoryLocalPlanner/Algorithm/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -I/root/tina/prebuilt/gcc/linux-x86/arm/toolchain-sunxi/toolchain/include -I$(STAGING_DIR)/usr/include/allwinner/include/ -I$(STAGING_DIR)/usr/include/libsgbot/ -I$(STAGING_DIR)/usr/include/allwinner -I/root/tina/out/astar-parrot/compile_dir/target/seeing-navigation/SeNaviCommon/Source -I/root/tina/out/astar-parrot/staging_dir/target/usr/include/allwinner/include/ -I/root/tina/out/astar-parrot/staging_dir/target/usr/include/libsgbot/ -I/root/tina/out/astar-parrot/staging_dir/target/usr/include/allwinner -O0 -g -Wall -DBOOST_LOG_DYN_LINK -D logLevel=0 -c -fmessage-length=0 -std=gnu++11 -DBOOST_LOG_DYN_LINK -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


