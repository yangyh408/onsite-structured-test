#!/bin/sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
chmod -R 777 "$SCRIPT_DIR"/*
var="/TessNG/DLLs"
export LD_LIBRARY_PATH="$SCRIPT_DIR${var}:$LD_LIBRARY_PATH"

cd $SCRIPT_DIR

CONDA_PATH=$(which conda)
if [ -n "$CONDA_PATH" ]; then
    ONSITE_PYTHON_PATH="$("$CONDA_PATH" run -n onsite which python)"
    if [ -n "$ONSITE_PYTHON_PATH" ]; then
        echo "onsite python path: $ONSITE_PYTHON_PATH"
        "$ONSITE_PYTHON_PATH" "$SCRIPT_DIR/main.py"
    else
        echo "onsite environment not found. Please follow README.md to create onsite env"
    fi
else
    echo "Conda not found. Please install Consa and setup your environment."
fi
