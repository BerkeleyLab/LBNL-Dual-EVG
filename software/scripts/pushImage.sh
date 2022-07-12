scp download.bit programFlash.sh DefaultSequence.csv enorum@access.als.lbl.gov:
ssh enorum@access.als.lbl.gov "scp download.bit programFlash.sh DefaultSequence.csv bpm01:/usr/local/epics/R3.15.4/modules/instrument/dualEventGenerator/head/FPGA/ ; rm download.bit programFlash.sh"

