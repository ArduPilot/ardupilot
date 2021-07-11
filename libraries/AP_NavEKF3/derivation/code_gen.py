#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Taken from https://github.com/PX4/ecl/commit/264c8c4e8681704e4719d0a03b848df8617c0863
# and modified for ArduPilot
from sympy import ccode
from sympy.codegen.ast import float32, real

class CodeGenerator:
    def __init__(self, file_name):
        self.file_name = file_name
        self.file = open(self.file_name, 'w')

    def print_string(self, string):
        self.file.write("// " + string + "\n")

    def get_ccode(self, expression):
        return ccode(expression, type_aliases={real:float32})

    def write_subexpressions(self,subexpressions):
        write_string = ""
        for item in subexpressions:
            write_string = write_string + "const ftype " + str(item[0]) + " = " + self.get_ccode(item[1]) + ";\n"

        write_string = write_string + "\n\n"
        self.file.write(write_string)

    def write_matrix(self, matrix, variable_name, is_symmetric=False, pre_bracket="[", post_bracket="]", separator="]["):
        write_string = ""

        if matrix.shape[0] * matrix.shape[1] == 1:
            write_string = write_string + variable_name + " = " + self.get_ccode(matrix[0]) + ";\n"
        elif matrix.shape[0] == 1 or matrix.shape[1] == 1:
            for i in range(0,len(matrix)):
                write_string = write_string + variable_name + pre_bracket + str(i) + post_bracket + " = " + self.get_ccode(matrix[i]) + ";\n"

        else:
            for j in range(0, matrix.shape[1]):
                for i in range(0, matrix.shape[0]):
                    if j >= i or not is_symmetric:
                        write_string = write_string + variable_name + pre_bracket + str(i) + separator + str(j) + post_bracket + " = " + self.get_ccode(matrix[i,j]) + ";\n"

        write_string = write_string + "\n\n"
        self.file.write(write_string)

    def close(self):
        self.file.close()
