module CppUTestToUnityUtils
  
  def convert_test_filename_to_unity_filename(testpath)
    testpath.sub(/tests\/(.*)\.cpp/, "unity/\\1.c")
  end
  
  def convert_test_filename_to_unity_testrunner_filename(testpath)
    testpath.sub(/tests\/(.*)\.cpp/, "unity/\\1_runner.c")
  end
  
  def convert_one_liners(line, group)
    line.sub!(/#include "CppUTest\/TestHarness.h\"/, "#include \"unity_fixture.h\"" )
    line.sub!(/FAIL\(/, 'TEST_FAIL(')
    line.sub!(/CHECK\(/, "TEST_ASSERT_TRUE(")
    line.sub!(/CHECK_TRUE\(/, "TEST_ASSERT_TRUE(")
    line.sub!(/CHECK_FALSE\(/, "TEST_ASSERT_FALSE(")
    line.sub!(/LONGS_EQUAL\(/, "TEST_ASSERT_EQUAL(")
    line.sub!(/BYTES_EQUAL\(/, "TEST_ASSERT_EQUAL_HEX8(")
    line.sub!(/STRCMP_EQUAL\(/, "TEST_ASSERT_EQUAL_STRING(")
    line.sub!(/DOUBLES_EQUAL\(/, "TEST_ASSERT_FLOAT_WITHIN(")
    line.sub!(/ POINTERS_EQUAL\(/, " TEST_ASSERT_POINTERS_EQUAL(")
    line.sub!(/CHECK_EQUAL\(true,/, "TEST_ASSERT_TRUE(")
    line.sub!(/CHECK_EQUAL\(false,/, "TEST_ASSERT_FALSE(")
    line.sub!(/CHECK_EQUAL\(/, "TEST_ASSERT_EQUAL(")
    #line.sub!(/static void setup\(/, "TEST_SETUP(" + group)
    #line.sub!(/static void teardown\(/, "TEST_TEAR_DOWN(" + group)
  end

  def convert_setup(lines, group)
    lines.each do | line |
        if line.sub!(/static void setup\(/, "TEST_SETUP(" + group)
          return 
        end
    end
  end

  def convert_teardown(lines, group)
    lines.each do | line |
      if line.sub!(/static void teardown\(/, "TEST_TEAR_DOWN(" + group)
        return
      end
    end
  end

  def convert_macros(lines, groups)
    groups.each do | group |
      lines.each do | line |
        convert_one_liners(line, group)
      end
      convert_setup(lines, group)
      convert_teardown(lines, group)
    end
  end

  def get_test_group(lines)
    @test_group = "None"
    lines.each do | line |
      if /TEST_GROUP/ =~ line
        @test_group = line.split(/[()]/)[1]
      end
    end
    @test_group
  end

  def get_test_groups(lines)
    @test_groups = []
    i = 0
    lines.each do | line |
      if /TEST_GROUP/ =~ line
        @test_groups[i] = line.split(/[()]/)[1]
        i = i + 1
      end
    end
    @test_groups
  end

  def adjust_tabs(lines)
    lines.each do | line |

    line.gsub!(/^ {2,3}(\w+)/,  "    \\1")
    line.gsub!(/\t/, "    ")
  end
  end

  def include_line?(line)
    /\#include/ =~ line
  end

  def convert_member_to_static(line)
    line.gsub!(/^\s*(\w)/, "static \\1")
  end 
 
  def add_semicolon_to_end_of_test_group_line(line)
    line.gsub!(/\)/, ");")
  end 
 
  def consume_closing_curley_brace(line)
    line.gsub!(/\}\;*[ \t]*(.*)/, "\\1")
  end

  def outdent(line)
    line.gsub!(/^    /, "")
  end

  def consume_opening_curley_brace(line)
   line.gsub!(/\{/, "")
  end

  def demote_group(group, lines)
    test_group = "None"
    scope_level = 0
    in_test_group = false
    lines.each do | line |
      next if include_line?(line)

      if !in_test_group
        if line.match(group)
          add_semicolon_to_end_of_test_group_line(line)
          in_test_group = true
        end
        next
      end 

      if line.include?("}")
        scope_level -= 1
      elsif line.include?("{")
        scope_level += 1
      end

      outdent(line)     
   
      if scope_level == 1
        convert_member_to_static(line)
        consume_opening_curley_brace(line)
      end
      if scope_level == 0
        consume_closing_curley_brace(line)
        in_test_group = false
      end
    end
  end
  
def search_and_destroy(pattern, lines)
  lines.each do | line |
    line.gsub!(pattern, "")
    end 
end

def extern_c_line?(line)
  line.match(/extern \"C\"/)
end
  
def  remove_extern_c(lines)
  in_extern_c = false;
  scope_level = 0
  
  lines.each do | line |
    if !in_extern_c
      if extern_c_line?(line)
        in_extern_c = true
        line.gsub!(/extern \"C\"/, "")
      end
      next
    end

    # next if ! in_extern_c and ! extern_c_line?(line)
    # 
    # in_extern_c = true
    if line.include?("{")
      scope_level += 1
    elsif line.include?("}")
      scope_level -= 1
    end

    if scope_level == 1
      consume_opening_curley_brace(line)
    end
    
    if scope_level == 0 and line.include?("}")
      in_extern_c = false
      consume_closing_curley_brace(line)
    end
  end
end
  

  def test_declaration?(line, group)
    /TEST\(#{group}/ =~ line
  end

  def generate_group_runner(group, lines)
    group_runner = []
    group_runner << "/* Make sure you invoke RUN_TEST_GROUP(" + group + ") from unity main */\n\n"
    group_runner << "TEST_GROUP_RUNNER(" + group + ")\n"
    group_runner << "{\n"
    lines.each do | line |
      if test_declaration?(line, group)
         temp = line.clone
         temp.sub!(/\n/, "")
         temp.sub!(/^IGNORE_/, "")
         temp.sub!(/^TEST/, "    RUN_TEST_CASE")
         group_runner <<  temp + ";\n"
      end
    end
      group_runner << "}\n\n"
  end
  
  def generate_group_runners(groups, lines)
    group_runners = []
    group_runners << "/* Generated code, edit at your own risk */\n\n"
    group_runners << "#include \"unity_fixture.h\"\n\n"
    groups.each do | group |
      group_runners.concat generate_group_runner(group, lines)
    end
    group_runners
  end

  def generate_group_runner_plainUnity(group, lines)
    prototypes = []
    callers = []
    group_runner = []
    lines.each do | line |
      if /void test.*\(.*\)/ =~ line
        temp = line.clone
        temp.sub!(/\n/, "")
        prototypes << temp + ";"
        temp.sub!(/void /, "    ")
        temp.sub!(/\(void\)/, "()")
        callers <<  temp
      end
    end
    group_runner << "\n"
    group_runner << "//Generated code, edit at your own risk\n\n"
    group_runner << "\#include \"unity_fixture.h\"\n\n"
    group_runner << prototypes
    group_runner << "\n\nTEST_GROUP_RUNNER(" + group + ")\n"
    group_runner << "{\n"
    group_runner << callers
    group_runner << "}\n"
  end

  def write_lines_to_file(file, lines)
    lines.each do | line |
      file.write(line)
    end
  end
end