
#
# Build sketch objects
#

$(BUILDROOT)/%.o: $(BUILDROOT)/%.cpp $(GENERATE_TARGETS) $(MAVLINK_HEADERS)
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(BUILDROOT)/make.flags $(SRCROOT)/%.cpp $(GENERATE_TARGETS) $(MAVLINK_HEADERS)
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $*.cpp $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

#
# Build library objects from sources in the sketchbook
#

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.cpp $(GENERATE_TARGETS) $(MAVLINK_HEADERS)
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SLIB_INCLUDES)
