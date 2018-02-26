################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Source/planner/implements/TrajectoryLocalPlanner/Algorithm/subdir.mk
-include Source/planner/implements/TrajectoryLocalPlanner/subdir.mk
-include Source/planner/implements/GlobalPlanner/Algorithm/subdir.mk
-include Source/planner/implements/GlobalPlanner/subdir.mk
-include Source/costmap/utils/subdir.mk
-include Source/costmap/layers/subdir.mk
-include Source/costmap/costmap_2d/subdir.mk
-include Source/costmap/subdir.mk
-include Source/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: NSeNavigation

# Tool invocations
NSeNavigation: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross G++ Linker'
	g++ -L"/home/cybernik/Development/Projects/SeNaviCommon/Debug" -L/lib -o "NSeNavigation" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(C_DEPS)$(CPP_DEPS) NSeNavigation
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
