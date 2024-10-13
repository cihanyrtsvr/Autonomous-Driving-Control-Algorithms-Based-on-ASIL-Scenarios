#!/bin/bash

export LUXAD_TOOLKIT_VERSION="0.1.0"
export CARLA_ROOT_FOLDER=lux_ad_carla
export CARLA_BINARIES=$CARLA_ROOT_FOLDER/CarlaUE4/Binaries
export CARLA_PYTHON_API=$CARLA_ROOT_FOLDER/PythonAPI
export PYTHONPATH=$CARLA_PYTHON_API/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg:$CARLA_PYTHON_API/carla:$CARLA_PYTHON_API/carla/agents


function luxad_run_server
{
    "$CARLA_BINARIES/Linux/CarlaUE4-Linux-Shipping" \
        CarlaUE4 \
        -prefernvidia \
        -quality-level=Low \
        "$@" 2>&1 > /dev/null &
}

function luxad_run_traffic
{
    python "$CARLA_PYTHON_API/examples/generate_traffic.py" --asynch -s 2 -n 40 -w 0 &
}

function luxad_run_client {
    # Check if an argument is provided
    if [ -z "$1" ]; then
        echo "❗ Usage: luxad_run_client <controller_type>"
        return 1
    fi

    # Run plots.py in the background and store its process ID
    python plots.py "$1" &

    # Run client.py in the background and store its process ID
    python client.py "$1"
}

function luxad_live_plote
{
    python plots.py
}

function luxad_linting
{   
    
    pylint  --ignore=lux_ad_carla *.py
}

function luxad_format_code
{
     black .  --exclude lux_ad_carla
}

function luxad_version
{


echo "  ______ _____  ______  _________ __________________       _______ ________  "
echo "  ___  / __  / / /__  |/ /__  __ \___  ____/___  __/       ___    |___  __ \ "
echo "  __  /  _  / / / __    / _  / / /__  /_    __  /          __  /| |__  / / / "
echo "  _  /___/ /_/ /  _    |  / /_/ / _  __/    _  /           _  ___ |_  /_/ /  "
echo "  /_____/\____/   /_/|_|  \____/  /_/       /_/            /_/  |_|/_____/   "
echo "                                                                             "
echo "         _______________ _______ ______ ______ __________________            "
echo "         ___  __/__  __ \__  __ \___  / ___  //_/____  _/___  __/            "
echo "         __  /   _  / / /_  / / /__  /  __  ,<    __  /  __  /               "
echo "         _  /    / /_/ / / /_/ / _  /____  /| |  __/ /   _  /                "
echo "         /_/     \____/  \____/  /_____//_/ |_|  /___/   /_/                 "
echo "                                                                             "
echo Luxoft Autonomous drive toolchain $LUXAD_TOOLKITxit_VERSION
}

function luxad_check_python
{
    python_version=$(python --version 2>&1)
    major_version=$(echo $python_version | cut -d ' ' -f 2 | cut -d '.' -f 1)
    minor_version=$(echo $python_version | cut -d ' ' -f 2 | cut -d '.' -f 2)

    if [ "$major_version" -eq 3 ] && [ "$minor_version" -eq 7 ]; then
        echo "✅ Python version 3.7 is compatible with CARLA"
    else
        echo "❗ Wrong Python version $python_version, please use 3.7"
    fi
}

echo "✅ Luxoft AD toolkit activated"
luxad_check_python
chmod +x "$CARLA_BINARIES/Linux/CarlaUE4-Linux-Shipping"