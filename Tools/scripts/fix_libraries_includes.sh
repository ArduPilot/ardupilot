#!/usr/bin/env bash

src=$(realpath $(dirname $BASH_SOURCE)/../../)
base=$src/libraries
declare -A header_dirs

arg_verbose=false
arg_create_commits=false

usage(){
    cat <<EOF
Usage: $(basename $BASH_SOURCE) [OPTIONS] [--] [<pathspec>...]

Fix includes of libraries headers in source files to be as the following:

 - If the header is in the same directory the source belongs to, then the
 notation #include "" is used with the path relative to the directory
 containing the source.

 - If the header is outside the directory containing the source, then we use
 the notation #include <> with the path relative to libraries folder.

If pathspec is given then it's an argument passed directly to git-grep. See
git-grep(1) for more information on its format. In this case the changes will
apply only to files that match the pathspec. Otherwise changes will be made to
the entire repository.

The output is a log of the process.

OPTIONS:
    -h,--help
        Display this help message.

    -v,--verbose
        Not only log errors and warnings but also substitutions.

    -c,--create-commits
        Create commits in the end.

    --commit
        Assume that the user have run the substitutions beforehand - only
        create the commits.
EOF
}

create_commits(){
    for f in $(git diff-files --name-only); do
        if [[ ${f%%/*} == "libraries" ]]; then
            echo $f | sed "s,\(libraries/[^/]*\)/.*,\1,"
        else
            echo ${f%%/*}
        fi
    done | uniq | while read d; do
        if [[ $d == libraries/* ]]; then
            commit_base=${d#libraries/}
        else
            commit_base=$d
        fi
        cat >/tmp/commit_msg <<EOF
$commit_base: standardize inclusion of libraries headers

This commit changes the way libraries headers are included in source files:

 - If the header is in the same directory the source belongs to, so the
 notation '#include ""' is used with the path relative to the directory
 containing the source.

 - If the header is outside the directory containing the source, then we use
 the notation '#include <>' with the path relative to libraries folder.

Some of the advantages of such approach:

 - Only one search path for libraries headers.

 - OSs like Windows may have a better lookup time.
EOF
        git add -u $d
        git commit -F /tmp/commit_msg
    done
}

replace_include(){
    local file=$1
    local n=$2
    local new_path=$3
    local old_path=$4
    local regex="\(#\s*include\s*\)[<\"].\+[>\"]"

    [[ $new_path == $old_path ]] && return

    $arg_verbose && echo "$file:$n: $old_path -->  $new_path"
    if ! sed -i "${n}s,$regex,\1$new_path," $file; then
        echo Error on executing command: sed -i "${n}s,$regex,\1$new_path," $file >&2
        kill -SIGINT $$
    fi
}

fix_includes(){
    local file=$1
    local header=$2
    local dirs=(${header_dirs[$header]})
    local num_dirs=${#dirs[@]}
    local regex="^\s*#\s*include\s*[<\"]\(.*/\)\?$header[>\"]"

    grep -ahno $regex $file | while IFS=":" read n match; do
        path=$(echo $match | sed "s/^\s*#\s*include\s*//g")
        delim=${path:0:1}
        path=${path:1:(${#path}-2)}
        file_dir=$(realpath $(dirname $file))

        if [[ $delim == "\"" ]]; then
            localpath=$file_dir/$path
            if [[ -f $localpath ]]; then
                # verify if file is under to the file dir
                localpath=$(realpath $localpath)
                [[ $localpath == $file_dir* ]] && continue

                # if not under file dir, check if $localpath is under $base
                if [[ $localpath == $base* ]]; then
                    new_path=${localpath#$base/}
                    replace_include $file $n \<$new_path\> \"$path\"
                    continue
                fi
            fi
        fi

        match_count=0
        possible_paths=()
        for dir in "${dirs[@]}"; do
            if [[ $dir/$header == *$path ]]; then
                ((match_count++))
                new_path=$dir/$header
                possible_paths[${#possible_paths[@]}]=$new_path
            fi
        done

        if [[ $match_count -eq 0 ]]; then
            echo "$file:$n: couldn't find a match for inclusion of $path"
        elif [[ $match_count -eq 1 ]]; then
            # check if included header is under file dir
            if [[ -f $file_dir/$path ]]; then
                new_path=\"$(realpath $file_dir/$path --relative-to $file_dir)\"
            else
                new_path=\<$new_path\>
            fi
            if [[ $delim == '"' ]]; then path=\"$path\"; else path=\<$path\>; fi
            replace_include $file $n $new_path $path
        else
            echo "$file:$n: more than one match for inclusion of $path"
            echo "    possible paths:"
            for p in "${possible_paths[@]}"; do
                echo "    $p"
            done
        fi
    done
}

trap_reset_tree(){
    echo
    echo Process killed or interrupted! Reseting tree...
    git -C $src reset --hard
    exit 1
}

# parse args
while [[ -n $1 ]]; do
    case "$1" in
    -h|--help)
        usage
        exit 0
        ;;
    -v|--verbose)
        arg_verbose=true
        ;;
    -c|--create-commits)
        arg_create_commits=true
        ;;
    --commit)
        create_commits
        exit $?
        ;;
    --)
        # remaining args are pathspecs
        shift
        break
        ;;
    -*)
        usage >&2
        exit 1
        ;;
    *)
        # this and the remaining args are pathspecs
        break
    esac
    shift
done

trap trap_reset_tree SIGINT SIGKILL

if ! git -C $src diff-files --quiet --exit-code; then
    echo You have unstaged changes, please commit or stash them beforehand >&2
    exit 1
fi

pushd $src > /dev/null

# collect all headers
git -C $base ls-files *.h > /tmp/headers
total=$(cat /tmp/headers | wc -l)
header_max_len=0
while read f; do
    header=$(basename $f)
    dir=$(dirname $f)
    if [[ -z ${header_dirs[$header]} ]]; then
        header_dirs[$header]=$dir
    else
        header_dirs[$header]+=" $dir"
    fi
    printf "\rCollecting header files paths... $((++i))/$total" >&2
    [[ ${#header} -gt $header_max_len ]] && header_max_len=${#header}
done </tmp/headers
echo

total=${#header_dirs[@]}
i=0
for header in "${!header_dirs[@]}"; do
    regex="#\s*include\s*[<\"]\(.*/\)\?$header[>\"]"
    printf "\r($((++i))/$total) Fixing includes for header %-${header_max_len}s" $header >&2

    # for each file that includes $header
    git grep -l $regex -- "$@" | while read f; do
        fix_includes $f $header
    done
done

$arg_create_commits && create_commits

popd > /dev/null
