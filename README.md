 
# LUXOFT AD CONTROL


Welcome to the LUX_AD_CONTROL repository! This repository includes a control algorithm and a CARLA setup submodule (`lux_ad_carla`). Follow the instructions below to clone the repository, initialize submodules, and set up your environment to run CARLA properly.

## Prerequisites

### SSH Key Setup

Ensure you have an SSH key set up on your local machine. If not, create it as described [here](https://confluence.atlassian.com/bitbucket/set-up-an-ssh-key-728138079.html).

### Anaconda Installation

Install Anaconda with Python 3.7. You can download Anaconda from [here](https://www.anaconda.com/products/distribution#download-section).

## Cloning the Repository

1. **Clone the Repository with SSH**:
   ```bash
   git clone git@github.com:cihanyrtsvr/Autonomous-Driving-Control-Algorithms-Based-on-ASIL-Scenarios.git

2. **Navigate to the Directory**:
    ```bash
    cd lux_ad_control

3. **Initialize Submodules**:
    There are two solution for this section 
    1. The carla depending on its version can be fetched from on Carla [Website](https://carla.readthedocs.io/en/0.9.15)

    2. If the access is granted directly take the repo from Luxoft directory. 
        ```bash
        git submodule update --init --recursive --progress

## Setting Up Environment

1. **Configure Anaconda with Python 3.7**:
    ```bash
    conda create -n lux_ad_env python=3.7
    conda activate lux_ad_env


2. **Run the Setup Script**:
    ```bash
    source luxad_toolkit.sh
    ```
## Running CARLA

1. **Start Carla Server**:
    ```bash
    luxad_run_server
    ```
    * Once the server screen opens, press ENTER.

2. **Start Carla Client**:
    ```bash
    luxad_run_client
    ```
    * This will start the client and allow you to see the implemented code in action.
