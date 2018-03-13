################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/costmap/costmap_2d/CostMap2D.cpp \
../Source/costmap/costmap_2d/CostMapLayer.cpp \
../Source/costmap/costmap_2d/LayeredCostMap.cpp 

OBJS += \
./Source/costmap/costmap_2d/CostMap2D.o \
./Source/costmap/costmap_2d/CostMapLayer.o \
./Source/costmap/costmap_2d/LayeredCostMap.o 

CPP_DEPS += \
./Source/costmap/costmap_2d/CostMap2D.d \
./Source/costmap/costmap_2d/CostMapLayer.d \
./Source/costmap/costmap_2d/LayeredCostMap.d 


# Each subdirectory must supply rules for building sources it contributes
Source/costmap/costmap_2d/%.o: ../Source/costmap/costmap_2d/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	ccache g++ -I/usr/include/eigen3 -I"/root/git/libsgbot/include" -I"/home/cybernik/Development/Projects/NSeNavigation/Source" -I"/home/cybernik/Development/Projects/SeNaviCommon/Source" -O0 -g3 -Wall -D BOOST_LOG_DYN_LINK -D logLevel=0 -c -fmessage-length=0 -std=gnu++11 -DBOOST_LOG_DYN_LINK -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


