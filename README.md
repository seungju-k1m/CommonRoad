# CommonRoad-Simulator-MMC



<h2> Install </h2>

    # First, install SUMO version 1.9.2
    sudo apt-get install software-propertise-common
    sudo add-apt-repository ppa:sumo/stable
    sudo apt-get update
    sudo apt-get install sumo sumo-tools sumo-doc

    # Check version of SUMO
    sumo --version
    

    # Second, specify environment variable SUMO_HOME
    # if you find /your/path/to/sumo, then

    vim ~/.bashrc
    export SUMO_HOME=/your/path/to/sumo

    # if you does not find path of sumo, then
    # In your own directory,

    git clone --recursive https://github.com/DLR-TS/SUMOLibraries
    
    vim ~/.bashrc
    export SUMO_HOME=/your/path/to/sumo

    # Third, install git

    mkdir simulator
    cd simulator
    git clone https://github.com/seungju-mmc/CommonRoad.git

    # Fourth, configure workspace for simulator

    conda create -n py37 python=3.7
    conda activate py37
    pip install -r requirements.txt
    python main.py