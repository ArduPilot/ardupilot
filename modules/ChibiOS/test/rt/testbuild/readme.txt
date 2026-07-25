This test performs 5 distinct operations on the RT code base. Each phase
writes a log file where errors can be found if the execution stops.

Step 1: Build

This step makes sure that there aren't compilation errors nor warnings in all
the defined configurations.

Step 2: Execute

The test suite is executed in the simulator in order to make sure that all
the defined test cases succeed in all the defined configurations.
Coverage data is collected during the execution for use by step 3.

Step 3: Coverage

The utility gcov is ran on the generate data and the coverage information is
stored in reports under ./reports.

Step 4: Analysis

PC-Lint is run on the codebase in order to detect MISRA violations or other
problems under the current analyser rules set (PC-Lint 9.0L is required).

Step 5: Clearing

The compilation products are cleared and the system is restored to original
state except for the generated reports and logs.
