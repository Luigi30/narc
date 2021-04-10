# narc

- run fixer.py to update old GSP assembler format directives and constants to ones that work with 6.10 (WIP, probably doesn't catch everything)
- in the NARC folder, run gmake allnarc -m
- once you've fixed all the syntax problems and gotten it to assemble and link, run NARCROMS.BAT
- now you have NARCFC and NARCFE files. these are 27010 program ROM images.
- for MAME, rename the ROMs as follows:  
	NARCFC.0 -> NARCREV7.U42, NARCFC.1 -> NARCREV7.U24,
	NARCFE.0 -> NARCREV7.U41, NARCFE.1 -> NARCREV7.U23