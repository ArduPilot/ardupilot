# setup output directory

[ -z "$DOCS_OUTPUT_BASE" ] && {
    export DOCS_OUTPUT_BASE=docs
}

mkdir -p $DOCS_OUTPUT_BASE/tags

