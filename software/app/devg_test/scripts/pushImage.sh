scp BOOT.bin programFlash.sh enorum@access.als.lbl.gov:
ssh enorum@access.als.lbl.gov "scp BOOT.bin programFlash.sh bpm01:/vxboot/siochsd/head/FPGA/ ; rm BOOT.bin programFlash.sh"

