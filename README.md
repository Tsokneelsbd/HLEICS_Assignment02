# HLEICS_Assignment02

This is the helics co-simulation assignment with two python federates.

Installing HELICS for Anaconda

Use the following steps to install HELICS. These are also available in the lecture slides.

First, install HELICS for Python:
!conda install -c gmlc-tdc helics

Next, you need to install the HELICS command line interpreter (helics-cli), this is needed no matter if you are using Anaconda:
!pip install git+git://github.com/GMLC-TDC/helics-cli.git@main


Then, you need to install the helics-apps package:
!pip install helics-apps --upgrade --upgrade-strategy only-if-needed


Finally, download the example files. 
