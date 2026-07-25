#!/bin/bash

function run_cmake() {
    # remove build directory if it exists
    rm -rf build
    # # make build directory
    mkdir build
    # echo with highlighted text
    echo -e "\e[1;32mRunning cmake with options: $*\e[0m"
    cmake $* -DMEMORYCHECK_COMMAND_OPTIONS="--error-exitcode=1 --leak-check=full --track-origins=yes" -S . -B build || exit 1
    cmake --build build --parallel `nproc`
    if ! ctest --test-dir build -D ExperimentalMemCheck --output-on-failure; then
        exit 1
    fi

    pushd build/
    # ctest -T memcheck || exit 1
    # also run the coverage if coverage is enabled
    if [[ $* == *-DCANARD_ENABLE_COVERAGE=1* ]]; then
        echo -e "\e[1;32mRunning coverage test\e[0m"
        make coverage || exit 1
        # copy and merge the coverage.info file to ../coverage.info if it exists
        if [ -f ../coverage.info ]; then
            echo -e "\e[1;32mMerging coverage.info to ../coverage.info\e[0m"
            lcov --add-tracefile coverage.info --add-tracefile ../coverage.info --output-file ../coverage.info
        else
            echo -e "\e[1;32mCopying coverage.info to ../coverage.info\e[0m"
            cp coverage.info ../coverage.info
        fi
        # print the coverage
        echo -e "\e[1;32mCoverage:\e[0m"
        lcov --list ../coverage.info
    fi
    popd
}

OPTIONS=( CMAKE_32BIT CANARD_ENABLE_CANFD CANARD_ENABLE_DEADLINE CANARD_MULTI_IFACE )

# if no argument is given, run all possible combinations
if [ $# -eq 0 ]; then
    # remove existing coverage report
    rm -f coverage.info
    for (( i = 0; i < 2 ** ${#OPTIONS[@]}; i++ )); do
        OPTS=""
        for (( j = 0; j < ${#OPTIONS[@]}; j++ )); do
            if [ $(( i & ( 1 << j ) )) -ne 0 ]; then
                OPTS="$OPTS -D${OPTIONS[j]}=1"
            fi
        done
        run_cmake -DBUILD_TESTING=1 -DCANARD_ENABLE_COVERAGE=1 $OPTS
    done
else
    run_cmake $*
fi

