==============
SWaT Simulator
==============

You can download the original simulator from the link: 
https://sav.sutd.edu.sg/research/physical-attestation/sp-2018-paper-supplementary-material/


The main function of this software is swat.py: you can run it by typing:

	python3 swat.py
	
in a terminal.

The structure is as follows:

1. The IO.py file instantiates the input output classes for communication between PLC and plant.
2. The SCADA.py file instantiates the input output classes for communication between PLC and HMI.
3. The plc/ directory contains 6 PLC classes files.
4. The HMI/ directory contains HMI classes to support SCADA.py.
5. The controlblocks/ directory contains PLC subfunction classes to support building the PLC classes.
6. The io_plc/ directory contains classes to support IO.py.
7. The logicblock/ directory contains useful functions like Timers and Alarms used in PLC class.
8. The plant/ directory contains the plant model and mandatory controller.
9. The supervision/ directory contains model deviation detector and a switcher.

================
Pre-requirements
================

python version 3.7
scipy
numpy