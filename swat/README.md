### You can download the original swat simulator from the link: 
https://sav.sutd.edu.sg/research/physical-attestation/sp-2018-paper-supplementary-material/


### Running the simulation
The main function of this software is swat.py: you can run it in a terminal.

	python3 swat.py


### The structure is as follows:
  - The IO.py file instantiates the input output classes for communication between PLC and plant.
  - The SCADA.py file instantiates the input output classes for communication between PLC and HMI.
  - The plc/ directory contains 6 PLC classes files.
  - The HMI/ directory contains HMI classes to support SCADA.py.
  - The controlblocks/ directory contains PLC subfunction classes to support building the PLC classes.
  - The io_plc/ directory contains classes to support IO.py.
  - The logicblock/ directory contains useful functions like Timers and Alarms used in PLC class.
  - The plant/ directory contains the plant model and mandatory controller.
  - The supervision/ directory contains model deviation detector and a switcher.


### Requirements
  - python version 3.7
  - scipy
  - numpy
