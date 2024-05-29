'''
Reads two lua docs files and checks for differences

python ./libraries/AP_Scripting/tests/docs_check.py "./libraries/AP_Scripting/docs/docs expected.lua" "./libraries/AP_Scripting/docs/docs.lua"

AP_FLAKE8_CLEAN
'''

import optparse, sys, re

class method(object):
    def __init__(self, global_name, local_name, num_args, full_line, returns, params):
        self.global_name = global_name
        self.local_name = local_name
        self.num_args = num_args
        self.full_line = full_line
        self.returns = returns
        self.params = params

    def __str__(self):
        ret_str = "%s\n" % (self.full_line)
        if len(self.local_name):
            ret_str += "\tClass: %s\n" % (self.global_name)
            ret_str += "\tFunction: %s\n" % (self.local_name)
        else:
            ret_str += "\tGlobal: %s\n" % (self.global_name)
        ret_str +=  "\tNum Args: %s\n" % (self.num_args)

        ret_str +=  "\tParams:\n"
        for param_type in self.params:
            ret_str += "\t\t%s\n" % param_type

        ret_str +=  "\tReturns:\n"
        for return_type in self.returns:
            ret_str += "\t\t%s\n" % return_type

        ret_str +="\n"
        return ret_str

    def type_compare(self, A, B):
        if (((len(A) == 1) and (A[0] == 'UNKNOWN')) or
            ((len(B) == 1) and (B[0] == 'UNKNOWN'))):
            # UNKNOWN is a special case used for manual bindings
            return True

        if len(A) != len(B):
            return False

        for i in range(len(A)):
            if A[i] != B[i]:
                return False

        return True

    def types_compare(self, A, B):
        if len(A) != len(B):
            return False

        for i in range(len(A)):
            if not self.type_compare(A[i], B[i]):
                return False

        return True
    
    def check_types(self, other):
        if not self.types_compare(self.returns, other.returns):
            return False

        if not self.types_compare(self.params, other.params):
            return False

        return True

    def __eq__(self, other):
        return (self.global_name == other.global_name) and (self.local_name == other.local_name) and (self.num_args == other.num_args)

def get_return_type(line):
    try:
        match = re.findall("^---@return (\w+(\|(\w+))*)", line)
        all_types = match[0][0]
        return all_types.split("|")

    except:
        raise Exception("Could not get return type in: %s" % line)

def get_param_type(line):
    try:
        match = re.findall("^---@param \w+\?? (\w+(\|(\w+))*)", line)
        all_types = match[0][0]
        return all_types.split("|")

    except:
        raise Exception("Could not get param type in: %s" % line)

def parse_file(file_name):
    methods = []
    returns = []
    params = []
    with open(file_name) as fp:
        while True:
            line = fp.readline()
            if not line:
                break

            # Acuminate return and params to associate with next function
            if line.startswith("---@return"):
                returns.append(get_return_type(line))

            if line.startswith("---@param"):
                params.append(get_param_type(line))

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
            args = args[0:args.find(")")]

            if len(args) == 0:
                num_args = 0
            else:
                num_args = args.count(",") + 1
                # ... shows up in arg list but not @param, add a unknown param
                if args.endswith("..."):
                    params.append(["UNKNOWN"])

            if num_args != len(params):
                raise Exception("Missing \"---@param\" for function: %s", line)

            # get global/class name and function name
            local_name = ""
            if function_name.count(":") == 1:
                global_name, local_name = function_name.split(":",1)
            else:
                global_name = function_name

            methods.append(method(global_name, local_name, num_args, line, returns, params))
            returns = []
            params = []

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

                elif not meth.check_types(got):
                    print("Type error:")
                    print("Want:")
                    print(meth)
                    print("Got:")
                    print(got)
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


