################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/costmap/layers/InflationLayer.cpp \
../Source/costmap/layers/StaticLayer.cpp 

OBJS += \
./Source/costmap/layers/InflationLayer.o \
./Source/costmap/layers/StaticLayer.o 

CPP_DEPS += \
./Source/costmap/layers/InflationLayer.d \
./Source/costmap/layers/StaticLayer.d 


# Each subdirectory must supply rules for building sources it contributes
Source/costmap/layers/%.o: ../Source/costmap/layers/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -I$(STAGING_DIR)/usr/include/allwinner/include/ -I$(STAGING_DIR)/usr/include/libsgbot/ -I$(STAGING_DIR)/usr/include/allwinner -O0 -g3 -Wall -D BOOST_LOG_DYN_LINK -D logLevel=0 -c -fmessage-length=0 -std=gnu++11 -DBOOST_LOG_DYN_LINK -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


