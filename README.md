- run fixer.py in SYS and NARC to update old GSP assembler format directives and constants to ones that work with 6.10
-- this will fix up the source files in the OLD folder and put them in the main folder
- run gmake allnarc -m in the SYS folder
- run gmake allnarc -m in the NARC folder
- run NARCROMS.BAT, now you have NARCFC and NARCFE files. these are 27010 ROM images.
- for MAME, rename the ROMs as follows:  
	NARCFC.0 -> NARCREV7.U42, NARCFC.1 -> NARCREV7.U24,
	NARCFE.0 -> NARCREV7.U41, NARCFE.1 -> NARCREV7.U23

current problems:
- the DMA1 version of LOAD doesn't seem to be available anywhere, so I can't produce graphics ROMs until
  that's reverse-engineered or found
- fortunately the graphics and sound are pre-built...
- ...except for the mugshots, title screen, and FBI logo which are all linked into the program ROMs by LOAD.