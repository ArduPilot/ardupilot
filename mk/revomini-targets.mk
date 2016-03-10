# main project target
$(BUILD_PATH)/main.o: main.cpp
	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBRARY_INCLUDES) -o $@ -c $< 

$(BUILD_PATH)/$(BOARD).elf: $(BUILDDIRS) $(TGT_BIN) $(BUILD_PATH)/main.o
	$(SILENT_LD) $(CXX) $(LDFLAGS) -o $@ $(TGT_BIN) $(BUILD_PATH)/main.o -Wl,-Map,$(BUILD_PATH)/$(BOARD).map

$(BUILD_PATH)/$(BOARD).bin: $(BUILD_PATH)/$(BOARD).elf
	$(SILENT_OBJCOPY) $(OBJCOPY) -v -Obinary $(BUILD_PATH)/$(BOARD).elf $@ 1>/dev/null
	$(OBJCOPY) -v -Oihex $(BUILD_PATH)/$(BOARD).elf $(BUILD_PATH)/$(BOARD).hex 1>/dev/null
	$(SILENT_DISAS) $(DISAS) -d $(BUILD_PATH)/$(BOARD).elf > $(BUILD_PATH)/$(BOARD).disas
	@echo " "
	@echo "Object file sizes:"
	@find $(BUILD_PATH) -iname *.o | xargs $(SIZE) -t > $(BUILD_PATH)/$(BOARD).sizes
	@cat $(BUILD_PATH)/$(BOARD).sizes
	@echo " "
	@echo "Final Size:"
	@$(SIZE) $<
	@echo $(MEMORY_TARGET) > $(BUILD_PATH)/build-type

$(BUILDDIRS):
	@mkdir -p $@

MSG_INFO:
	@echo "================================================================================"
	@echo ""
	@echo "  Path info:"
	@echo "     SRC Root:      " $(SRCROOT)
	@echo "     BUILD Path:    " $(BUILD_PATH)
	@echo "     SUPPORT Path:  " $(SUPPORT_PATH)
	@echo "     LD Path:       " $(LDDIR)
	@echo "     MAKE Path:     " $(MAKEDIR)
	@echo "     Hardware Path: "$(HARDWARE_PATH)
	@echo "     HAL Path:      "$(HAL_PATH)
	@echo "     STM32 Path:    "$(STM32_PATH)
	@echo "     Wirish Path:   "$(WIRISH_PATH)
	@echo "     Libraries Path:"$(LIBRARIES_PATH)
	@echo "     Firmware Path: "$(FIRMWARE_PATH)
	@echo "  Sketch info:"
	@echo "     SKETCH         " $(SKETCH)
	@echo "     SKETCHPDESRCS  " $(SKETCHPDESRCS)
	@echo "     SKETCHSRCS     " $(SKETCHSRCS)
	@echo "     SKETCHPDE      " $(SKETCHPDE)
	@echo "     SKETCHCPP      " $(SKETCHCPP)
	@echo "  Build Directories info:"
	@echo "     BUILD DIRS:    " $(BUILDDIRS)
	@echo "  Libraries Include info:"
	@echo "     INCLUDES:      " $(LIBRARY_INCLUDES)
	@echo ""
	@echo "================================================================================"
	@echo ""
	@echo "  Build info:"
	@echo "     BOARD:          " $(BOARD)
	@echo "     MCU:            " $(MCU)
	@echo "     MEMORY_TARGET:  " $(MEMORY_TARGET)
	@echo ""
	@echo "  See 'make help' for all possible targets"
	@echo ""
	@echo "================================================================================"
	@echo ""
