#!/usr/bin/ruby
require File.join(File.dirname(__FILE__),  'cpp_u_test_to_unity_utils.rb')
include CppUTestToUnityUtils
  
in_file = ARGV[0]
create_group_runner_file(in_file)
unity_runner_filename = convert_test_filename_to_unity_testrunner_filename(in_file)
puts "Creating test runner for :" + in_file + "\n"
puts "              Generating :" + unity_filename + "\n"

test_lines = File.open(in_file).readlines
out_unity_runner_file = File.open(unity_runner_filename, 'w')

test_group = get_test_group(test_lines)
group_runner = generate_group_runner(test_group, test_lines)
write_lines_to_file(out_unity_runner_file, group_runner)
out_unity_runner_file.close()
