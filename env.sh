# Get the path of the current conda environment
CONDA_ENV_PATH=$(conda info --base)/envs/$(basename "$CONDA_PREFIX")

export LD_LIBRARY_PATH="$CONDA_ENV_PATH/lib/:$LD_LIBRARY_PATH"

export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
