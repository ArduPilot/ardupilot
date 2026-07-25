#!/bin/ruby
require File.join(File.dirname(__FILE__),  'cpp_u_test_to_unity_utils.rb')
include CppUTestToUnityUtils
  
in_file = ARGV[0]
test_lines = File.open(in_file).collect
out_file = File.open(File.basename(in_file, ".c") + "_runner.c", 'w')

group_runner = generate_group_runner_plainUnity("unity", test_lines)
write_lines_to_file(out_file, group_runner)
out_file.close()

