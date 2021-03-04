1. You can download the original RUBis simulator from the link: https://github.com/cps-sei/swim


2. Running the simulation
  - Unzip the files "queueinglib.zip" and "swim.zip" in RUBiS directory, then

  - Install to /usr/include/eigen3/
      sudo apt-get install libeigen3-dev

  - Run our simulation:
      cd RUBiS/swim/simulations
      ./runexp.sh


3. Additional structure:
 - The src/managers/plan directory contains designed control-based optimal controller
 - The supervision/ directory contains model deviation detector, mandatory controller, and a switcher.


4. Requirements:
 - Unbuntu 18.04
 - OMNeT++ 5.5.1 (https://omnetpp.org/omnetpp)
 - Eigen 3.3.4 
