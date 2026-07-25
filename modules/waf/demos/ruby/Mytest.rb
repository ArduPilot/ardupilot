
require 'mytest_ext'

module Mytest
	def hello
		puts MytestInt::hello()
	end
	module_function :hello
end
