#Generated by VisualGDB project wizard. 
#Note: VisualGDB will automatically update this file when you add new sources to the project.
#All other changes you make in this file will be preserved.
#Visit http://visualgdb.com/makefiles for more details

#VisualGDB: AutoSourceFiles		#<--- remove this line to disable auto-updating of SOURCEFILES and EXTERNAL_LIBS

TARGETNAME := RTOSSTM32CubeHALTestProject.elf
#TARGETTYPE can be APP, STATIC or SHARED
TARGETTYPE := APP

to_lowercase = $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$1))))))))))))))))))))))))))

CONFIG ?= DEBUG

CONFIGURATION_FLAGS_FILE := $(call to_lowercase,$(CONFIG)).mak

include $(CONFIGURATION_FLAGS_FILE)
include $(ADDITIONAL_MAKE_FILES)

ifeq ($(BINARYDIR),)
error:
	$(error Invalid configuration, please check your inputs)
endif

SOURCEFILES := Lib/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/startup_stm32f0xx.c Lib/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c Lib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c Lib/Middlewares/Third_Party/FreeRTOS/Source/croutine.c Lib/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c Lib/Middlewares/Third_Party/FreeRTOS/Source/list.c Lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/port.c Lib/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c Lib/Middlewares/Third_Party/FreeRTOS/Source/queue.c Lib/Middlewares/Third_Party/FreeRTOS/Source/tasks.c Lib/Middlewares/Third_Party/FreeRTOS/Source/timers.c Src/freertos.c Src/main.c Src/stm32f0xx_hal_msp.c Src/stm32f0xx_it.c
EXTERNAL_LIBS := 
EXTERNAL_LIBS_COPIED := $(foreach lib, $(EXTERNAL_LIBS),$(BINARYDIR)/$(notdir $(lib)))

CFLAGS += $(COMMONFLAGS)
CXXFLAGS += $(COMMONFLAGS)
ASFLAGS += $(COMMONFLAGS)
LDFLAGS += $(COMMONFLAGS)

CFLAGS += $(addprefix -I,$(INCLUDE_DIRS))
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRS))

CFLAGS += $(addprefix -D,$(PREPROCESSOR_MACROS))
CXXFLAGS += $(addprefix -D,$(PREPROCESSOR_MACROS))
ASFLAGS += $(addprefix -D,$(PREPROCESSOR_MACROS))

CXXFLAGS += $(addprefix -framework ,$(MACOS_FRAMEWORKS))
CFLAGS += $(addprefix -framework ,$(MACOS_FRAMEWORKS))
LDFLAGS += $(addprefix -framework ,$(MACOS_FRAMEWORKS))

LDFLAGS += $(addprefix -L,$(LIBRARY_DIRS))

ifeq ($(GENERATE_MAP_FILE),1)
LDFLAGS += -Wl,-Map=$(BINARYDIR)/$(basename $(TARGETNAME)).map
endif

LIBRARY_LDFLAGS = $(addprefix -l,$(LIBRARY_NAMES))

ifeq ($(IS_LINUX_PROJECT),1)
	RPATH_PREFIX := -Wl,--rpath='$$ORIGIN/../
	LIBRARY_LDFLAGS += $(EXTERNAL_LIBS)
	LIBRARY_LDFLAGS += -Wl,--rpath='$$ORIGIN'
	LIBRARY_LDFLAGS += $(addsuffix ',$(addprefix $(RPATH_PREFIX),$(dir $(EXTERNAL_LIBS))))
	
	ifeq ($(TARGETTYPE),SHARED)
		CFLAGS += -fPIC
		CXXFLAGS += -fPIC
		ASFLAGS += -fPIC
		LIBRARY_LDFLAGS += -Wl,-soname,$(TARGETNAME)
	endif
	
	ifneq ($(LINUX_PACKAGES),)
		PACKAGE_CFLAGS := $(foreach pkg,$(LINUX_PACKAGES),$(shell pkg-config --cflags $(pkg)))
		PACKAGE_LDFLAGS := $(foreach pkg,$(LINUX_PACKAGES),$(shell pkg-config --libs $(pkg)))
		CFLAGS += $(PACKAGE_CFLAGS)
		CXXFLAGS += $(PACKAGE_CFLAGS)
		LIBRARY_LDFLAGS += $(PACKAGE_LDFLAGS)
	endif	
else
	LIBRARY_LDFLAGS += $(EXTERNAL_LIBS)
endif

LIBRARY_LDFLAGS += $(ADDITIONAL_LINKER_INPUTS)

all_make_files := $(firstword $(MAKEFILE_LIST)) $(CONFIGURATION_FLAGS_FILE) $(ADDITIONAL_MAKE_FILES)

ifeq ($(STARTUPFILES),)
	all_source_files := $(SOURCEFILES)
else
	all_source_files := $(STARTUPFILES) $(filter-out $(STARTUPFILES),$(SOURCEFILES))
endif

source_obj1 := $(all_source_files:.cpp=.o)
source_obj2 := $(source_obj1:.c=.o)
source_obj3 := $(source_obj2:.s=.o)
source_obj4 := $(source_obj3:.S=.o)
source_obj5 := $(source_obj4:.cc=.o)
source_objs := $(source_obj5:.cxx=.o)

all_objs := $(addprefix $(BINARYDIR)/, $(notdir $(source_objs)))

PRIMARY_OUTPUTS :=

ifeq ($(GENERATE_BIN_FILE),1)
PRIMARY_OUTPUTS += $(BINARYDIR)/$(basename $(TARGETNAME)).bin
endif

ifeq ($(GENERATE_IHEX_FILE),1)
PRIMARY_OUTPUTS += $(BINARYDIR)/$(basename $(TARGETNAME)).ihex
endif

ifeq ($(PRIMARY_OUTPUTS),)
PRIMARY_OUTPUTS := $(BINARYDIR)/$(TARGETNAME)
endif

all: $(PRIMARY_OUTPUTS)

$(BINARYDIR)/$(basename $(TARGETNAME)).bin: $(BINARYDIR)/$(TARGETNAME)
	$(OBJCOPY) -O binary $< $@

$(BINARYDIR)/$(basename $(TARGETNAME)).ihex: $(BINARYDIR)/$(TARGETNAME)
	$(OBJCOPY) -O ihex $< $@

ifeq ($(TARGETTYPE),APP)
$(BINARYDIR)/$(TARGETNAME): $(all_objs) $(EXTERNAL_LIBS)
	$(LD) -o $@ $(LDFLAGS) $(START_GROUP) $(all_objs) $(LIBRARY_LDFLAGS) $(END_GROUP)
endif

ifeq ($(TARGETTYPE),SHARED)
$(BINARYDIR)/$(TARGETNAME): $(all_objs) $(EXTERNAL_LIBS)
	$(LD) -shared -o $@ $(LDFLAGS) $(START_GROUP) $(all_objs) $(LIBRARY_LDFLAGS) $(END_GROUP)
endif
	
ifeq ($(TARGETTYPE),STATIC)
$(BINARYDIR)/$(TARGETNAME): $(all_objs)
	$(AR) -r $@ $^
endif

-include $(all_objs:.o=.dep)

clean:
ifeq ($(USE_DEL_TO_CLEAN),1)
	del /S /Q $(BINARYDIR)
else
	rm -rf $(BINARYDIR)
endif

$(BINARYDIR):
	mkdir $(BINARYDIR)

#VisualGDB: FileSpecificTemplates		#<--- VisualGDB will use the following lines to define rules for source files in subdirectories
$(BINARYDIR)/%.o : %.cpp $(all_make_files) |$(BINARYDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)

$(BINARYDIR)/%.o : %.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)

$(BINARYDIR)/%.o : %.S $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) $(ASFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)

$(BINARYDIR)/%.o : %.s $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) $(ASFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)

$(BINARYDIR)/%.o : %.cc $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) $(CXXFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)

$(BINARYDIR)/%.o : %.cxx $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) $(CXXFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)

#VisualGDB: GeneratedRules				#<--- All lines below are auto-generated


$(BINARYDIR)/startup_stm32f0xx.o : Lib/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/startup_stm32f0xx.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/system_stm32f0xx.o : Lib/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_cortex.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_dma.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_flash.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_flash_ex.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_gpio.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_pwr.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_pwr_ex.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_rcc.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_rcc_ex.o : Lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/cmsis_os.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/croutine.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/croutine.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/event_groups.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/list.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/list.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/port.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/port.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/heap_4.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/queue.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/queue.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/tasks.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/tasks.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/timers.o : Lib/Middlewares/Third_Party/FreeRTOS/Source/timers.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/freertos.o : Src/freertos.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/main.o : Src/main.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_hal_msp.o : Src/stm32f0xx_hal_msp.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)


$(BINARYDIR)/stm32f0xx_it.o : Src/stm32f0xx_it.c $(all_make_files) |$(BINARYDIR)
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MF $(@:.o=.dep)

