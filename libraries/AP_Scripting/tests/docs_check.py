'''
Reads two lua docs files and checks for differences

python ./libraries/AP_Scripting/tests/docs_check.py "./libraries/AP_Scripting/docs/docs expected.lua" "./libraries/AP_Scripting/docs/docs.lua"

AP_FLAKE8_CLEAN
'''

import optparse, sys

class method(object):
    def __init__(self, global_name, local_name, num_args, full_line):
        self.global_name = global_name
        self.local_name = local_name
        self.num_args = num_args
        self.full_line = full_line

    def __str__(self):
        ret_str = "%s\n" % (self.full_line)
        if len(self.local_name):
            ret_str += "\tClass: %s\n" % (self.global_name)
            ret_str += "\tFunction: %s\n" % (self.local_name)
        else:
            ret_str += "\tGlobal: %s\n" % (self.global_name)
        ret_str +=  "\tNum Args: %s\n\n" % (self.num_args)
        return ret_str

    def __eq__(self, other):
        return (self.global_name == other.global_name) and (self.local_name == other.local_name) and (self.num_args == other.num_args)

def parse_file(file_name):
    methods = []
    with open(file_name) as fp:
        while True:
            line = fp.readline()
            if not line:
                break

            # only consider functions
            if not line.startswith("function"):
                continue

            # remove any comment
            line = line.split('--', 1)[0]

            # remove any trailing whitespace
            line = line.strip()

            # remove "function"
            function_line = line.split(' ', 1)[1]

            # remove "end"
            function_line = function_line.rsplit(' ', 1)[0]

            # remove spaces
            function_line = function_line.replace(" ", "")

            # get arguments
            function_name, args = function_line.split("(",1)
            args = args[0:args.find(")")-1]

            if len(args) == 0:
                num_args = 0
            else:
                num_args = args.count(",") + 1

            # get global/class name and function name
            local_name = ""
            if function_name.count(":") == 1:
                global_name, local_name = function_name.split(":",1)
            else:
                global_name = function_name

            methods.append(method(global_name, local_name, num_args, line))

    return methods

def compare(expected_file_name, got_file_name):

    # parse in methods
    expected_methods = parse_file(expected_file_name)
    got_methods = parse_file(got_file_name)

    pass_check = True

    # make sure each expected method is included once
    for meth in expected_methods:
        found = False
        for got in got_methods:
            if got == meth:
                if found:
                    print("Multiple definitions of:")
                    print(meth)
                    pass_check = False
                found = True

        if not found:
            print("Missing definition of:")
            print(meth)
            pass_check = False


    # make sure no unexpected methods are included
    for got in got_methods:
        found = False
        for meth in expected_methods:
            if got == meth:
                found = True
                break
        if not found:
            print("Unexpected definition of:")
            print(got)
            pass_check = False

    if not pass_check:
        raise Exception("Docs do not match")
    else:
        print("Docs check passed")

if __name__ == '__main__':

    parser = optparse.OptionParser(__file__)

    opts, args = parser.parse_args()

    if len(args) != 2:
        print('Usage: %s "expected docs path" "current docs path"' % parser.usage)
        sys.exit(0)

    compare(args[0], args[1])


