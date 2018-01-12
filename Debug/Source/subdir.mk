################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Main.cpp \
../Source/NavigationApplication.cpp \
../Source/test.cpp 

OBJS += \
./Source/Main.o \
./Source/NavigationApplication.o \
./Source/test.o 

CPP_DEPS += \
./Source/Main.d \
./Source/NavigationApplication.d \
./Source/test.d 


# Each subdirectory must supply rules for building sources it contributes
Source/%.o: ../Source/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/home/cybernik/Development/Projects/SeNavigation/Source" -I/usr/include/eigen3 -O0 -g3 -Wall -D BOOST_LOG_DYN_LINK -D logLevel=0 -c -fmessage-length=0 -std=gnu++11 -DBOOST_LOG_DYN_LINK -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


