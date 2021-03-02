You can download the original RUBis simulator from the link: https://github.com/cps-sei/swim


Unzip the files "queueinglib.zip" and "swim.zip" in RUBiS directory, then run our simulation:

    cd RUBiS/swim/simulations
    ./runexp.sh


Additional structure:
    1.The src/managers/plan directory contains designed control-based optimal controller
    2.The supervision/ directory contains model deviation detector, mandatory controller, and a switcher.


Requirements:
    Unbuntu 18.04
    Python3.7 (http://www.python.org).
    OMNeT++ 5.5.1 (https://omnetpp.org/omnetpp)