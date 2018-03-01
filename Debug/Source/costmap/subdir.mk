################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/costmap/CostmapWrapper.cpp 

OBJS += \
./Source/costmap/CostmapWrapper.o 

CPP_DEPS += \
./Source/costmap/CostmapWrapper.d 


# Each subdirectory must supply rules for building sources it contributes
Source/costmap/%.o: ../Source/costmap/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/usr/include/eigen3 -I"/root/git/libsgbot/include" -I"/home/cybernik/Development/Projects/SeNaviCommon/Source" -O0 -g3 -Wall -D BOOST_LOG_DYN_LINK -D logLevel=0 -c -fmessage-length=0 -std=gnu++11 -DBOOST_LOG_DYN_LINK -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


