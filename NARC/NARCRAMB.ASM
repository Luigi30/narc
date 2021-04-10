 	.FILE	"NARCRAMB.ASM"
	.TITLE	" <<< N  A  R  C  -- RAMBO >>>"

**************************************************************************
*                                                                        *
* 	COPYRIGHT (C) 1988 WILLIAMS ELECTRONICS GAMES, INC. 		 *
* 	ALL RIGHTS RESERVED.						 *
*                                                                        *
**************************************************************************

	.WIDTH	132
	.OPTION	B,D,L
	.MNOLIST


*	GET THE SYSTEM STUFF
	.INCLUDE	"\video\mprocEQU.ASM"	;MPROC equates
	.INCLUDE	"\video\DISPEQU.ASM"	;Display processor equates
	.INCLUDE	"\video\GSP.INC"	;GSP assembler equates
	.INCLUDE	"\video\SYS.INC"	;Zunit system equates
	.INCLUDE	"\video\MACROS.HDR"	;Macros, Yeah!
*	LOCAL STUFF
	.INCLUDE	"NARCEQU.ASM"		;NARC Equates
	.INCLUDE	"NARCTEQU.ASM"
	.include	"imgtbl.glo"
  	.INCLUDE	"NARCFRAG.GLO"
	.include	"narcramb.tbl"

	.GLOBL	RAMBCRM1
	.GLOBL	MAKEFLAM,GROWFLAM


	.GLOBL	BCOLDDMP,BCOLDCAN,BCOLDPST,BCOLDSID,BCOLL1

	.bss	rambcol,16,1

DIR 		.set	ZPER		;RAMBO

********** GEORGE -- COMMENT THIS STUFF OUT **************
;* Also uncomment RETS in GVARINIT
;	.BSS	rambmax,16,16		;U - max # of active rambmen
;	.BSS	rambbrain,16,16		;U - sleep between homing on players
;	.BSS	rambdrug,16,16		;U - chance/1000 of rambman having drugs
	.BSS	rambclip,16,16		;U - chance/1000 of rambman having clips

	.TEXT
VARINIT:
	movi	150,A0
	move	A0,@rambclip,W
	rets		   ;* <---------------------- remove ;
	movi	1,A0
	move	A0,@rambmax,W
	movi	4,A0
	move	A0,@rambbrain,W
	movi	250,A0
	move	A0,@rambdrug,W
	rets
**********************************************************

***************************************************************************
; RAMBO SUB_TYPES
; 0 - w/ no coors starts at PLZMIN and 1 to 2 full screens away from
;	players in X.  90% chance of coming up to the right.
; 1 - FlameThrower - GRAY
;
DTBL:
	.long	D0,D1
	.long	D0,D0
	.long	D0,D0,D0,D0,D0,D0
	.long	D0,D0,D0,D0,D0,D0
***************************************************************************

************************* FUNCTION TABLE ***************************
RAMBFUNCS:
	.long	BOOF
	.long	GoDestV
	.long	GetXVel
	.long	0B000H	;MaxXZAngle
	.long	30	;FlipTiks
********************************************************************

********************************************************************
;To catch a nulled dereference through JR_FUNCS
BOOF:	jruc	$
********************************************************************

**************************************************************************
*INIT TABLE
*	.LONG	OXVEL,OYVEL,OXVAL,XYVAL
*	.WORD	OZPOS,OZVEL
*	.LONG	OIMG,OZSIZ,OCVECT
*	.WORD	OFLAGS,OID
RAMBZSIZ	.set	0000C000CH
LegsInit:
	.LONG	0,0,0,0
	.WORD	0,0
	.LONG	rmrun1,RAMBZSIZ,RAMBCOLL
	.WORD	DMAWNZ,RAMBEOID
	.LONG	RAMBBLIP
BodyInit:
	.LONG	0,0,0,0
	.WORD	0,0
	.LONG	rmsh0a,RAMBZSIZ,RAMBCOLL
	.WORD	DMAWNZ,RAMBEOID
	.LONG	0
**************************************************************************

**************************************************************************
FRANCHEK:
	cmpi	FRANMIN,A9
	jrlt	$
	cmpi	FRANMAX,A9
	jrgt	$
	rets
**************************************************************************

**************************************************************************
RAMBCOLD:
;* PARMS
; A5  - Y:X
; A6  - 0:Z	;if A5 and A6=0 then Random Position
; A14 - 0.8:#.4:rand#.4:00EH.8:OIDtype.4:OIDsubtype.4

	mmtm	SP,A0,A1,A2,A7,A9,A10,A11
	callr	VARINIT
	
	move	@SLPRAMBS,A0,W
	cmpi	5,A0
	jrge	ColdX

	move	A5,A9		;Y:X in A9
	move	A6,A10		;0:Z in A10
	move	A14,A11		;ID in A11

	move	A14,A0
	sll	12,A0
	srl	28,A0
	jrz	SkRndCnt
	calla	RANDU
SkRndCnt:
	move	A14,A2
	sll	8,A2
	srl	28,A2
	add	A0,A2
ColdLp:	
	CREATE	RAMBEPID,RAMBO ;allocate process?	(bashes A1,A7)
	jrz	ColdX
	dsjs	A2,ColdLp
ColdX:
	mmfm	SP,A0,A1,A2,A7,A9,A10,A11
	rets
**************************************************************************

**************************************************************************
CkPal:
	move	@rambcol,A0,W
	jrz	CkPalX
	sll	5,A0
	addi	RAMBPALS,A0
	move	*A0,A0,L
	calla	GETFPAL
	jrz	CkPalX
	move	A0,A1
	move	*A8(OPAL),A0,W
	calla	FREEPAL
	move	A1,*A8(OPAL),W
CkPalX:
	rets
**************************************************************************

*********************  START OF RAMBOMAN PROCESS  *************************
RAMBO:
* A9  - Y:X
* A10 - 0:Z	;if A9,A10 = 0 generate random start pos.
* A11 - 0.8:#.4:rand#.4:00EH.8:OIDtype.4:OIDsubtype.4
;based on sub-type, jump to appropriate routine

	INCW	@SLPRAMBS
MaxWait:
	move	@rambmax,A0,W
	move	@RAMBCNT,A1,W
	cmp	A0,A1
	jrlt	MaxWaitX
	SLEEP	30
	jruc	MaxWait
MaxWaitX:
	DECW	@SLPRAMBS
	INCW	@RAMBCNT

	clr	A0
	move	A0,@rambcol,W

	move	A11,A0
	sll	28,A0
	srl	28,A0
	sll	5,A0
	addi	DTBL,A0
	move	*A0,A0,L
	jump	A0

**************************************************************************
D1:	
	movk	1,A0
	move	A0,@rambcol,W

**************************************************************************
D0:	
	move	A9,A0
	or	A10,A0
	jrnz	GotCoors

*Enemy object initialization
	move	@(WORLDTLX+16),A9,W				 
	subi	500,A9
	movi	950,A0
	calla	RANDPER
	jrnc	StrtLft
	addi	1500,A9
StrtLft:
	movi	90,A0
	calla	RANDU
	add	A0,A9

	move	@PLYMIN,A1,W									 
	movi	45,A0
	calla	RANDU
	add	A0,A1
	sll	16,A1
	movy	A1,A9	;* initial position							 
	clr	A10

GotCoors:
	move	A9,A1
	movi	RAMBTAB,A6
	movi	LegsInit,A14	
	mmtm	SP,A9,A10,A11
	calla	ENINITLL		;CREATE RAMBO Legs
	mmfm	SP,A9,A10,A11
	jrz	RAMBDIE1		;create bombed
;Y:X in A9
;0:Z in A10
	move	A10,A10
	jrnz	GotZC
	movy	A9,A10
	srl	16,A10
	subi	ZORIGIN,A10
GotZC:
	move	A9,*A8(OXPOS),W
	move	A10,*A8(OZPOS),W
	addi	ZORIGIN,A10
	move	*A8(OSIZEY),A0,W
	sub	A0,A10
	move	A10,*A8(OYPOS),W
	move	A11,*A8(OID),W

	callr	CkPal
;	jruc	InitEnmy


****************************************************************************
InitEnmy:
	calla	GetBody
	jrz	RAMBDIE2
	calla	ClrVel
	clr	A0
	move	A0,*A13(COLLOBJ),L
	move	A0,*A13(DEST),L
	move	A0,*A13(TEMPDEST),L
	movb	A0,*A13(DEST_TAG)
	movb	A0,*A13(SNOOZE)
	movb	A0,*A13(FLIPTIKS)
	move	A0,*A13(TIKCTRS),L
	move	A0,*A13(EBYTES),L
	movi	RAMBFUNCS,A0
	move	A0,*A13(FUNCS),L
	movk	7,A0
	movb	A0,*A13(DIR)
	clr	A0
	callr	AniBod

	clr	A1
DrgLp:
	move	@rambdrug,A0,W
	calla	RANDPER
	jrnc	DrgX
	inc	A1
	cmpi	5,A1
	jrlt	DrgLp
DrgX:
	move	A1,A0
	jrz	NoDrg
	calla	ADDDRUGS
NoDrg:

	clr	A1
ClpLp:
	move	@rambclip,A0,W
	calla	RANDPER
	jrnc	ClpX
	inc	A1
	cmpi	3,A1
	jrlt	ClpLp
ClpX:
	move	A1,A0
	jrz	NoClp
	calla	ADDCLIPS
NoClp:
	
	movi	RAMBCOLL,A0
	move	A0,*A8(OCVECT),L

	callr	GoRun
	callr	Charge
	jruc	RAMBO_MV

	.even
**************************************************************************

**************************************************************************
GetBody:
	calla	GETOBJ
	jrz	GetBodyX
	movi	BodyInit,A14
	calla	STFOBJ
	move	A13,*A0(OPLINK),L
	move	A0,A7
	movi	ram1,A0
	calla	GETFPAL
	jrz	NoBodPal
	move	A0,*A7(OPAL),W
	move	A7,A0
	calla	INSOBJ
	movi	ENEMIES,A1
	calla	ADDSUPP
	move	A7,A10
GetBodyX:
	rets
NoBodPal:
	move	A1,A0
	calla	FREEOBJ
	clr	A0
	jruc	GetBodyX
**************************************************************************

**************************************************************************
DelBody:
	mmtm	SP,A0,A1
	move	A10,A0
	calla	DELPAL
	movi	ENEMIES,A1
	calla	DELSUPP
	calla	DELSOBJ
	mmfm	SP,A0,A1
	rets
**************************************************************************

**************************************************************************
AnimRst:
	clr	A0
	movb	A0,*A13(DEST_TAG)
	movb	A0,*A13(BRAINTIKS)
	movb	A0,*A13(ANIMTIKS)
;	callr	FRANCHEK
	rets
**************************************************************************

**************************************************************************
DO_JUMP:
	movi	-090000H,A0
	move	A0,*A8(OYVEL),L

	move	*A13(COLLOBJ),A0,L
	jrz	SkXBump
	calla	FACEOBJ
	move	*A13(COLLOBJ),A0,L
	move	*A0(OXVEL),A0,L
	sra	1,A0
	move	*A8(OXVEL),A1,L
	add	A0,A1
	move	A1,*A8(OXVEL),L
SkXBump:

	movi	RAMBJMP,A9,L

	movk	4,A1
	jsrp	FRANIM
RAMBFLY0:
	callr	ZCHECK
	callr	UpdTorso
	movk	1,A0
	calla	PRCSLP
	calla	DFRMGRND
	cmpi	30,A1
	jrgt	RAMBFLY1a
	move	*A8(OYVEL),A0,L
	jrp	RAMBFLY1a
	addi	0C000H,A0
	move	A0,*A8(OYVEL),L
	jruc	RAMBFLY0
RAMBFLY1a:
	movk	4,A1
	jsrp	FRANIM	   ;* throw it
RAMBFLY1:
	callr	ZCHECK
	callr	UpdTorso
	movk	1,A0
	calla	PRCSLP
	calla	DFRMGRND
	cmpi	45,A1
	jrgt	RAMBFLY2a
	move	*A8(OYVEL),A0,L
	jrp	RAMBFLY2a
	addi	0A000H,A0
	move	A0,*A8(OYVEL),L
	jruc	RAMBFLY1
RAMBFLY2a:
	movk	4,A1
	jsrp	FRANIM
RAMBFLY2:
	callr	ZCHECK
	callr	UpdTorso
	movk	1,A0
	calla	PRCSLP
	move	*A8(OYVEL),A0,L
	addi	0A000H,A0
	move	A0,*A8(OYVEL),L
	jrn	RAMBFLY2
	calla	DFRMGRND
	cmpi	30,A1
	jrlt	RAMBFLY4a
	jruc	RAMBFLY2
RAMBFLY4a:
	movk	4,A1
	jsrp	FRANIM
RAMBFLY4:
	callr	ZCHECK
	callr	UpdTorso
	movk	1,A0
	calla	PRCSLP
	move	*A8(OYVEL),A0,L
	addi	08000H,A0
	move	A0,*A8(OYVEL),L
	calla	DFRMGRND
;	subk	5,A1
	jrp	RAMBFLY4

;* we have hit the ground..
	move	*A8(OZVEL),*A8(OYVEL),L
	move	A8,A0
	calla	SETZPOS
	callr	AnimRst
	callr	GoRun
	clr	A0
	move	A0,*A13(COLLOBJ),L
	retp
**************************************************************************

**************************************************************************
ZCHECK:
	move	*A8(OZPOS),A1,W
	move	@PLZMIN,A0,W
	addk	5,A0
	sub	A0,A1
	jrnn	ZMinOK
	move	A0,*A8(OZPOS),W
	move	*A8(OYPOS),A0,W
	sub	A1,A0
	move	A0,*A8(OYPOS),W
	move	*A8(OZVEL),A0,L	;* want to check sign
	jrnn	ZMaxOK
	jruc	SETZV
ZMinOK:
	move	*A8(OZPOS),A1,W
	move	@PLZMAX,A0,W
	subk	3,A0
	sub	A0,A1
	jrn	ZMaxOK
	move	A0,*A8(OZPOS),W
	move	*A8(OYPOS),A0,W
	sub	A1,A0
	move	A0,*A8(OYPOS),W
	move	*A8(OZVEL),A0,L	;* want to check sign
	jrn	ZMaxOK
	jrz	ZMaxOK
SETZV:
	move	*A8(OYVEL),A1,L
	sub	A0,A1
	move	A1,*A8(OYVEL),L
	clr	A0
	move	A0,*A8(OZVEL),L	;set Z velocity
ZMaxOK:
	rets
**************************************************************************


**************************************************************************
CkXDIST:
	move	@(WORLDTLX+16),A1,W
	addi	256,A1
	calla	GETANIX
	sub	A1,A0
	cmpi	600,A0
	jrgt	DeepRght
	cmpi	300,A0
	jrgt	NearRght
	cmpi	-600,A0
	jrlt	DeepLeft
	cmpi	-300,A0
	jrlt	NearLeft

	;* on screen
	clr	A0
	movb	A0,*A13(SNOOZE)
	jruc	CkXDISTX

DeepLeft:
	jsrp	CkQuit
	movb	*A13(SNOOZE),A0
	jrnz	CkXDISTX
	movk	1,A0
	movb	A0,*A13(SNOOZE)
	calla	ShadOff
	jruc	CkXDISTX
DeepRght:
	movb	*A13(SNOOZE),A0
	jrnz	CkXDISTX
	movk	1,A0
	movb	A0,*A13(SNOOZE)
	calla	ShadOff
	jruc	CkXDISTX
NearRght:
	clr	A0
	movb	A0,*A13(SNOOZE)
	calla	ShadOn
	jruc	CkXDISTX
NearLeft:
	clr	A0
	movb	A0,*A13(SNOOZE)
	calla	ShadOn
;	jruc	CkXDISTX
CkXDISTX:
	RETP
**************************************************************************

**************************************************************************
CkQuit:
	move	@SLPRAMBS,A0,W
	jrz	CkQuitX
	jruc	RAMBDONE
CkQuitX:
	RETP
**************************************************************************

**************************************************************************
UpdTorso:
	mmtm	SP,A0
	move	*A8(OXVEL),*A10(OXVEL),L
	move	*A8(OYVEL),*A10(OYVEL),L
	move	*A8(OZVEL),*A10(OZVEL),L
	move	*A8(OZPOS),A0,L
	inc	A0
	move	A0,*A10(OZPOS),L
	MOVE	*A8(OIMG),A1,L
	MOVE	*A8(OSIZE),A2,L
	MOVE	*A8(OFLAGS),A4,W
	CALLA	GANIOF
	MOVE	*A8(OXVAL),A3,L
	MOVE	*A8(OYVAL),A5,L
	ADD	A6,A3
	ADD	A7,A5
	MOVE	*A10(OIMG),A1,L
	MOVE	*A10(OSIZE),A2,L
	MOVE	*A10(OFLAGS),A4,W
	CALLA	GANIOF
	SUB	A6,A3
	SUB	A7,A5
	MOVE	A3,*A10(OXVAL),L
	MOVE	A5,*A10(OYVAL),L
	mmfm	SP,A0
	rets
**************************************************************************

**************************************************************************
RAMBO_MV:
	callr	ZCHECK

	move	*A13(COLLOBJ),A0,L
	jrz	NOCOLL
	jsrp	DO_JUMP
NOCOLL:

;********  CHECK if a DESTINATION has been reached, and what to do if so..
	movb	*A13(DEST_TAG),A0
	jrz	SkCkDest
	movb	*A13(DESTTIKS),A0
	jrnz	SkCkDest
	calla	CkDest
	jrnc	NotYet

	;* we have reached our destination
	clr	A0
	movb	A0,*A13(BRAINTIKS)	;Allow new course of action
	movb	A0,*A13(DEST_TAG)
NotYet:
SkCkDest:

**** BRAIN functions *****
DoBrain:
	movb	*A13(BRAINTIKS),A0
	jrnz	BrainX

	movk	15,A0
	movb	*A13(SNOOZE),A1
	jrnz	SetBTiks
	move	@rambbrain,A0,W
SetBTiks:
	movb	A0,*A13(BRAINTIKS)		;reset brain wait

	jsrp	CkXDIST		;routine can DIE

	movb	*A13(SNOOZE),A0
	jrnz	SkDngrCk

	movb	*A13(DEST_TAG),A0
	cmpi	BULL_T,A0
	jrge	BrainX

	jsrp	CkDngr
	jrc	BrainX
SkDngrCk:

	move	*A8(OID),A0,W
	btst	0,A0
	jrnz	FRAMB
	callr	ATTACK
	callr	DoBod
	jruc	BrainX
FRAMB:
	callr	ATTACK2
	callr	DoBod2
BrainX:


**** Animation check ****
	movb	*A13(SNOOZE),A0
	jrnz	AnimX
	movb	*A13(ANIMTIKS),A0
	jrnz	AnimX

	movk	4,A1			;* one frame, no sleepin
	jsrp	FRANIM
	move	*A9,A1,L
	jrnz	ListLeft
	callr	AnimCycle
ListLeft:
	movb	A0,*A13(ANIMTIKS)
AnimX:

**** Find Next Event ****
	movb	*A13(BRAINTIKS),A0
	jrp	SkBrain1
	movk	1,A0
SkBrain1:
	move	A0,A5			;current min
	
	movb	*A13(DEST_TAG),A1
	jrz	SkMin1
	movb	*A13(DESTTIKS),A1
	sll	24,A1
	srl	24,A1
	jrnz	SkDest1
	movk	1,A1
SkDest1:
	cmp	A5,A1
	jrge	SkMin1
	move	A1,A5
SkMin1:

	;**** The following are only done when not snoozing
	movb	*A13(SNOOZE),A7
	jrnz	GotMin

	movb	*A13(ANIMTIKS),A2
	cmp	A5,A2
	jrge	SkMin2
	move	A2,A5
SkMin2:

GotMin:
	sub	A5,A0
	movb	A0,*A13(BRAINTIKS)
	move	A1,A1
	jrz	SkDestDc
	sub	A5,A1
	movb	A1,*A13(DESTTIKS)
SkDestDc:
	move	A7,A7	;ck snooze
	jrnz	MoveSlp
	sub	A5,A2
	movb	A2,*A13(ANIMTIKS)

MoveSlp:
	movb	*A13(FLIPTIKS),A1
	jrz	SkFlipDc
	andi	0FFH,A1	;No o' this sign extended byte shit
	sub	A5,A1
	jrp	SkClrFlp
	clr	A1
SkClrFlp:
	movb	A1,*A13(FLIPTIKS)
SkFlipDc:

	move	A5,A0
	callr	UpdTorso
	calla	PRCSLP

	jruc	RAMBO_MV
**************************************************************************

**************************************************************************
ATTACK:
	move	A8,A6
	calla	ClosestP
	move	A0,A8
	calla	GETANIX
	move	A8,A7
	move	A0,A1			;A1 = Xpos of Player
	move	A6,A8
	calla	GETANIX
	sub	A0,A1		;A1 = dist to closest player
	move	*A8(OXVEL),A2,L
	xor	A1,A2		;A2 neg means heading away from player
	move	A2,A2
	jrnn	AttackX		;if heading toward player, keep going..
	;* Should We Change X Dir?
	abs	A1
	movi	800,A0		
	calla	RANDU
	addi	50,A0
	cmp	A0,A1
	jrle	AttackX
	move	A7,A0
	calla	FACEOBJ

	;set ZVEL
	move	*A8(OXVEL),A0,L
	abs	A0
	calla	RANDU
	movb	@RAND,A1
	jrn	SkNegZ
	neg	A0
SkNegZ:
	move	A0,*A8(OZVEL),L
	move	A0,*A8(OYVEL),L
AttackX:
	rets
**************************************************************************


**************************************************************************
DoBod:
	movb	*A13(SNOOZE),A0
	jrnz	DoBodX
	move	A8,A6
	calla	ClosestP
	move	A0,A8
	calla	GETANIX
	move	A8,A7
	move	A0,A1			;A1 = Xpos of Player
	move	A6,A8
	calla	GETANIX
	sub	A0,A1			;A1 = X dist to closest player
	move	*A8(OZPOS),A3,W
	move	*A7(OZPOS),A2,W
	sub	A3,A2			;A2 = Z dist to closest player

	move	A2,A3
	move	A1,A4
	jrz	ZEROX
	abs	A3
	abs	A4
	sll	16,A3
	divu	A4,A3
GotZdivX:

	move	A1,A1
	jrn	LftDir
;* facing right, 0<=dir<=4
	move	A2,A2
	jrn	RgtUDir
;**facing right and down
	cmpi	01665H,A3	;ANG < 5 deg		
	jrlt	Dir2
	cmpi	04498H,A3	;ANG < 15 deg		
	jrlt	Dir3
	cmpi	0224FEH,A3	;ANG < 65 deg		
	jrlt	Dir4
	jruc	Dir5
RgtUDir:
;**facing right and up
	cmpi	01665H,A3	;ANG < 5 deg		
	jrlt	Dir2
	cmpi	04498H,A3	;ANG < 15 deg		
	jrlt	Dir1
	jruc	Dir0

LftDir:
;* facing left, 5<=dir<=9
	move	A2,A2
	jrn	LftUDir
;**facing left and down
	cmpi	01665H,A3	;ANG < 5 deg		
	jrlt	Dir8
	cmpi	04498H,A3	;ANG < 15 deg		
	jrlt	Dir7
	cmpi	0224FEH,A3	;ANG < 65 deg		
	jrlt	Dir6
	jruc	Dir5

LftUDir:
;**facing left and up
	cmpi	01665H,A3	;ANG < 5 deg		
	jrlt	Dir8
	cmpi	04498H,A3	;ANG < 15 deg		
	jrlt	Dir9
	jruc	Dir10

Dir0:	clr	A0
	jruc	SetBDir
Dir1:	movk	1,A0
	jruc	SetBDir
Dir2:	movk	2,A0
	jruc	SetBDir
Dir3:	movk	3,A0
	jruc	SetBDir
Dir4:	movk	4,A0
	jruc	SetBDir
Dir5:	movk	5,A0
	jruc	SetBDir
Dir6:	movk	6,A0
	jruc	SetBDir
Dir7:	movk	7,A0
	jruc	SetBDir
Dir8:	movk	8,A0
	jruc	SetBDir
Dir9:	movk	9,A0
	jruc	SetBDir
Dir10:	movk	10,A0
;	jruc	SetBDir

SetBDir:
	movb	*A13(DIR),A4
	cmp	A0,A4
	jrlt	IncDir
	jrgt	DecDir
	jruc	TryShot1
IncDir:
	inc	A4
	jruc	GotBDir
DecDir:
	dec	A4
GotBDir:
TryShot:
	cmp	A0,A4
	jrnz	SkShot
TryShot1:
	movb	*A13(CANFIRE),A1
	jrz	CantFire
	clr	A1
	movb	A1,*A13(CANFIRE)
	sll	5,A0
	addi	SDIRTBL,A0
	move	*A0,A0,L
	jump	A0

SDIRTBL:
	.long	SDir0,SDir1,SDir2,SDir3,SDir4,SDir5
	.long	SDir6,SDir7,SDir8,SDir9,SDir10
SDir5:
	cmpi	05ABD9H,A3	;ANG > 80 deg
	jrlt	SkShot
	jruc	DoShot
SDir2:
SDir8:
	cmpi	01665H,A3	;ANG < 4 deg
	jrgt	SkShot
	jruc	DoShot
SDir1:
SDir3:
SDir7:
SDir9:
	cmpi	02AD7H,A3	;6 deg < ANG < 14
	jrlt	SkShot
	cmpi	04235H,A3
	jrgt	SkShot
	jruc	DoShot
SDir0:
SDir4:
SDir6:
SDir10:
	cmpi	02AD7H,A3	;16 deg < ANG < 24
	jrlt	SkShot
	cmpi	04235H,A3
	jrgt	SkShot
;	jruc	DoShot

DoShot:
	callr	FIRE
	movb	A4,*A13(DIR)
	movk	1,A0
	callr	AniBod
	movk	5,A0
	movb	A0,*A13(BRAINTIKS)
	jruc	DoBodX

CantFire:
	movk	1,A1
	movb	A1,*A13(CANFIRE)
SkShot:
	movb	A4,*A13(DIR)
	clr	A0
	callr	AniBod
DoBodX:	
	rets

ZEROX:
	movi	07FFFFFFFH,A3
	jruc	GotZdivX
**************************************************************************

**************************************************************************
MinXDelt	.set	75
ATTACK2:
;* For flame thrower rambo, move toward nearest player
	move	A8,A6
	calla	ClosestP
	move	A0,A8
	calla	GETANIX
	move	A8,A7
	move	A0,A1			;A1 = Xpos of Player
	move	*A7(OZPOS),A3,W
	sll	16,A3
	movx	A1,A3
	move	A6,A8
	calla	GETANIX
	sub	A0,A1		;A1 = dist to closest player
	move	*A8(OXVEL),A2,L
	xor	A1,A2		;A2 neg means heading away from player
	move	A2,A2
	jrn	Going	;heading away from player, keep going

	;* heading toward the player
	abs	A1
	cmpi	MinXDelt+40,A1
	jrlt	Attack2X
	movi	650,A0
	calla	RANDPER
	jrc	Attack2X
	move	A3,*A13(DEST),L
	movi	MinXDelt,A0
	calla	ShortXDest
	calla	GoDestV
	jruc	Attack2X

Going:
	;* Should We Change X Dir?
	abs	A1
	cmpi	MinXDelt,A1
	jrlt	Attack2X
	movi	800,A0		
	calla	RANDU
	addi	MinXDelt,A0
	cmp	A0,A1
	jrle	Attack2X
	move	A7,A0
	calla	FACEOBJ
	move	A3,*A13(DEST),L
	movi	MinXDelt,A0
	calla	ShortXDest
	calla	GoDestV

Attack2X:
	rets
**************************************************************************

**************************************************************************
DoBod2:
	movb	*A13(SNOOZE),A0
	jrnz	DoBod2X
	move	A8,A6
	calla	ClosestP
	move	A0,A8
	calla	GETANIX
	move	A8,A7
	move	A0,A1			;A1 = Xpos of Player
	move	A6,A8
	calla	GETANIX
	sub	A0,A1			;A1 = X dist to closest player
	jrn	DirF4
	movk	2,A0
	jruc	StBDir
DirF4:
	movk	8,A0

StBDir:
	movb	*A13(DIR),A4
	cmp	A0,A4
	jrlt	IncDir2
	jrgt	DecDir2
	jruc	TryFlam1
IncDir2:
	inc	A4
	jruc	SkFlam
DecDir2:
	dec	A4
	jruc	SkFlam

TryFlam1:
	movb	*A13(CANFIRE),A3
	jrz	CantFlam

	move	*A8(OZPOS),A3,W
	move	*A7(OZPOS),A2,W
	sub	A2,A3			;A2 = Z dist to closest player
	abs	A1	;X delta
	cmpi	270,A1
	jrgt	SkFlam
	abs	A3	;Z delta
	srl	2,A1
	cmp	A1,A3
	jrgt	SkFlam

	clr	A1
	movb	A1,*A13(CANFIRE)

	callr	FLAMER
	movb	A4,*A13(DIR)
	movk	1,A0
	callr	AniBod
	movk	8,A0
	movb	A0,*A13(BRAINTIKS)
	jruc	DoBod2X

CantFlam:
	movk	1,A1
	movb	A1,*A13(CANFIRE)
SkFlam:
	movb	A4,*A13(DIR)
	clr	A0
	callr	AniBod
DoBod2X:	
	rets
**************************************************************************


**************************************************************************
AniBod:
;* PARMS
; A0 - 0 -> not firing
; A0 - 1 -> firing
	mmtm	SP,A1
	movb	*A13(DIR),A1
	sll	6,A1
	move	A0,A0
	jrnz	GetFTor
	addi	TORSOS,A1
	jruc	GotTor
GetFTor
	addi	FTORSOS,A1
GotTor:
	PUSHP	A9
	move	A1,A9
	SWAP	A8,A10
	movk	4,A1
	jsrp	FRANIM
	SWAP	A8,A10
	PULLP	A9
	mmfm	SP,A1
	rets
**************************************************************************


**************************************************************************
Charge:
	calla	ClosestP
	calla	FACEOBJ
	SWAP	A8,A10
	calla	FACEOBJ
	SWAP	A8,A10
	movi	020000H,A0
	calla	RANDU
	subi	010000H,A0
	move	A0,*A8(OZVEL),L
	move	A0,*A8(OYVEL),L
	rets
**************************************************************************

************************** Animation Cycler ******************************
AnimCycle:
;Resets animation loop
	move	*A13(ANIMPTR),A9,L
	rets
**************************************************************************

**************************** GetXVel *************************************
GetXVel:
	cmpi	MOVETBL,A9
	jrlt	NoXvel
	cmpi	MOVEEND,A9
	jrge	NoXvel
	cmpi	RUN,A9
	jrge	IsRun
	movi	WALKVEL+01000H,A0
	jruc	GotXvel
IsRun:
	movi	RUNVEL+01000H,A0
	jruc	GotXvel
NoXvel:
	clr	A0
GotXvel:
	rets
**************************************************************************

***************************** GoWalk *************************************
GoWalk:
	mmtm	SP,A0
	cmpi	WALK,A9
	jrlt	StrtWalk
	cmpi	WALKEND,A9
	jrge	StrtWalk

	jruc	SetVel1

StrtWalk:
	movi	WALK,A9
	move	A9,*A13(ANIMPTR),L

SetVel1:
	;* set veloctiy for skaters
	move	*A8(OFLAGS),A0,W
	btst	B_FLIPH,A0
	jrz	VelPos1
	movi	-WALKVEL,A0
	jruc	SetXV1
VelPos1:
	movi	WALKVEL,A0
SetXV1:
	move	A0,*A8(OXVEL),L
	calla	UpdShadV
	SWAP	A8,A10
	move	A0,*A8(OXVEL),L
	calla	UpdShadV
	SWAP	A8,A10

GoWalkX:
	mmfm	SP,A0
	rets
**************************************************************************

***************************** GoRun *************************************
GoRun:
	mmtm	SP,A0
	cmpi	RUN,A9
	jrlt	StrtRUN
	cmpi	RUNEND,A9
	jrge	StrtRUN

	jruc	SetVel2

StrtRUN:
	movi	RUN,A9
	move	A9,*A13(ANIMPTR),L

SetVel2:
	;* set veloctiy for skaters
	move	*A8(OFLAGS),A0,W
	btst	B_FLIPH,A0
	jrz	VelPos2
	movi	-RUNVEL,A0
	jruc	SetXV2
VelPos2:
	movi	RUNVEL,A0
SetXV2:
	move	A0,*A8(OXVEL),L
	calla	UpdShadV
	SWAP	A8,A10
	move	A0,*A8(OXVEL),L
	calla	UpdShadV
	SWAP	A8,A10

GoRUNX:
	mmfm	SP,A0
	rets
**************************************************************************


***************************** CHECK DANGER *******************************
* if in danger of being shot, dodge or jump
* if a destination is set, set the carry to prevent MODE stuff
CkDngr:
	calla	NEAR_BUL
	move	A5,A5	;is there a closest bullet
	jrz	NoDngr

	move	*A8(OYPOS),A1,W
	move	*A8(OSIZEY),A2,W
	srl	1,A2		;want to find belly
	subk	20,A2	;* encourage jumping
	add	A2,A1
	move	*A5(OYPOS),A3,W
	sub	A3,A1
	jrn	BULL_LOW
	;* bullet heading at head
	cmpi	200,A6		;is bullet real close?
	jrgt HGH_NEAR
;* bullet is very close and high
	;* shoot toward where bullet came from 
HGH_NEAR:
	cmpi	500,A6	;is bullet near?
	jrgt	DNGRX

	;* bullet is near - move down, down and away!
	;* find Z destination
	move	*A8(OZPOS),A1,W
	movk	30,A0
	calla	RANDU
	addk	15,A0
	add	A0,A1
	move	@PLZMAX,A0,W
	cmp	A0,A1
	jrlt	ZisGood
	move	@PLZMIN,A1,W
	add	A0,A1
	srl	1,A1
ZisGood:
	sll	16,A1	;A1 is z portion of destination

	;* generate X destination
	movk	30,A0
	calla	RANDU
	addk	30,A0		;A0 is magnitude of X distance to run away
	move	*A5(OXVEL+16),A2,W		;Get sign of Bullets velocity
	jrp	XdirOK
	neg	A0
XdirOK:
	move	A0,A2
	calla	GETANIX
	add	A0,A2
	movy	A1,A2
	move	A2,*A13(DEST),L
	jruc	DNGRDEST

BULL_LOW:
	;* bullet heading at feet
	cmpi	200,A6	;is bullet real close?
	jrgt	LOW_NEAR
	;* bullet is very close - jump and fire
	jsrp	DO_JUMP
	jruc	DNGRX
LOW_NEAR:
	cmpi	500,A6	;is bullet near?
	jrgt	DNGRX

	;* bullet is near - move up, up and away
	;* find Z destination
	move	*A8(OZPOS),A1,W
	movk	30,A0
	calla	RANDU
	addk	15,A0
	sub	A0,A1
	move	@PLZMIN,A0,W
	cmp	A0,A1
	jrgt	ZisGood2
	move	@PLZMAX,A1,W
	add	A0,A1
	srl	1,A1
ZisGood2:
	sll	16,A1	;A1 is z portion of destination

	;* generate X destination
	movk	30,A0
	calla	RANDU
	addk	30,A0		;A0 is magnitude of X distance to run away
	move	*A5(OXVEL+16),A2,W		;Get sign of Bullets velocity
	jrp	XdirOK2
	neg	A0
XdirOK2:
	move	A0,A2
	calla	GETANIX
	add	A0,A2
	movy	A1,A2
	move	A2,*A13(DEST),L

DNGRDEST:
	callr	AnimRst
	callr	GoRun
	movk	BULL_T,A0
	movb	A0,*A13(DEST_TAG)

	calla	GoDestV
	setc
	retp

DNGRX:
NoDngr:
	clrc
	retp
**************************************************************************

**************************************************************************
RAMBDONE:
	callr	DelBody
RAMBDIE2:
	move	A8,A0		;GET THE POINTER TO THE OBJECT
	calla	DELENEMY			;DELETE THE ENEMY
RAMBDIE1:
	DECW	@RAMBCNT
	DIE
**************************************************************************


**************************************************************************
*WALKING RAMBO, AS OBJECT 0, COLLSION ROUTINE
*A0 = PTR TO JR OBJECT
*A8 = PTR TO OBJECT THAT SMASHED HIM
CARCOL0
	MOVE	A8,A6		;KILLERS OBJ
	MOVE	A0,A8		;JR OBJ
	move	*A6(OXVEL),A0,L
	move	*A8(OXVEL),A1,L
	sub	A1,A0
	cmpi	070000H,A0
	jrlt	NotHurt
	movi	060000H,A0
	calla	RANDU
	subi	030000H,A0
	move	*A6(OXVEL),A1,L
	sra	1,A1
	move	A1,A2
	add	A1,A0
	move	A0,*A8(OXVEL),L
	abs	A2
	neg	A2
	move	A2,*A8(OYVEL),L
	jruc	Wasted
NotHurt:
	move	*A8(OPLINK),A0,L
;	move	A6,*A0(COLLOBJ),L
	clr	A0	;stop collision scan
	RETS

RAMBCOL0
	MOVE	A8,A6			;STORE KILLERS OBLOCK
	MOVE	A0,A8       		;PASS WASTED GUYS POINTER
Wasted:
	move	*A8(OPLINK),A0,L
	calla	GETA8
	calla	GETA10
	calla	DelBody

	MOVI	SPLTCOLBOX,A14
	MOVE	A14,*A8(OCVECT),L	;COLLISION POINTER

	MOVE	*A8(OPLINK),A0,L
	CALLA	KILL			;KILL CONTROL
	MOVE	A0,A14			;STORE THIS RELIC
	
	MOVE	*A6(OXVEL),A9,L		;GET PROJECTILE DATA
	MOVE	*A6(OYVEL),A10,L
	MOVE	*A6(OID),A11,W		
	MOVI	0EDh,A1
	MOVE	A11,A3		  	;COPY OF I.D.
	SLL	28,A3
	SRL	28,A3
	CMPI	ROCKETID,A3,W
	JREQ	BlowUp			;DEATH BY ROCKET
	CMPI	EXPLODID,A3,W
	JREQ	BlowUp			;DEATH BY EXPLOSION
	MOVI	WENDIE,A7
	JRUC	DoDth
BlowUp:
	MOVI	BLOWUP,A7
DoDth:
	CALLA	GETPRC			;CREATE A NEW ENEMY DEATH PROCESS
	jrz	SkWendie
	MOVE	*A6(ODMAXY),*A0(PDATA),L	;PASS KILLERS DMA XY POSITION
	MOVE	*A6(OSIZE),*A0(PDATA+20H),L	;AND KILLERS XY SIZE
	move	*A14(EBCNTRS),*A0(EBCNTRS),L	;and the booty counters
	MOVE	*A14(EMAINT),*A0(EMAINT),L

	move	*A6(OYPOS),A9,W
	MOVI	0EDh,A1
	movi	SPLATBOX,A7
	CALLA	GETPRC

DoDie:
	move	@RAMBCNT,A0,W
	dec	A0
	move	A0,@RAMBCNT,W
	calla	CARDCHEK

	movi	1,A0	;stop collision scan
	RETS
SkWendie:
	move	A8,A0
	calla	DELENEMY
	jruc	DoDie
*************************************************************************

LFALSND
	.WORD	0F000H|EXPLOPRI+1,018H,08007H,08016H,0  ;MISSLE LARGE EXPLOSION

**************************************************************************
BLOWUP:
	movi	LFALSND,A0
	calla	ONESND

;PROCESS TO BLOW RAMBO UP INTO AIR
	movi	-0A0000H,A0
	move	A0,*A8(OYVEL),L

	movi	040000H,A0
	calla	SRAND
	move	A0,*A8(OXVEL),L
	movi	020000H,A0
	calla	SRAND
	move	A0,*A8(OZVEL),L

	movi	RAMBBLOW,A9

	movk	4,A1
	jsrp	FRANIM
RAMBBLW0:
	callr	ZCHECK
	movk	1,A0
	calla	PRCSLP
	calla	DFRMGRND
	cmpi	80,A1
	jrgt	RAMBBLW1a
	move	*A8(OYVEL),A0,L
	jrp	RAMBBLW1a
	addi	05000H,A0
	move	A0,*A8(OYVEL),L
	jruc	RAMBBLW0
RAMBBLW1a:
	movk	4,A1
	jsrp	FRANIM	   ;* throw it
RAMBBLW1:
	callr	ZCHECK
	movk	1,A0
	calla	PRCSLP
	calla	DFRMGRND
	cmpi	180,A1
	jrgt	RAMBBLW2a
	move	*A8(OYVEL),A0,L
	jrp	RAMBBLW2a
	addi	05000H,A0
	move	A0,*A8(OYVEL),L
	jruc	RAMBBLW1
RAMBBLW2a:
	CALLA	DROPSTUF		;DROP THE BOOTY
	movk	4,A1
	jsrp	FRANIM
RAMBBLW2:
	callr	ZCHECK
	movk	1,A0
	calla	PRCSLP
	move	*A8(OYVEL),A0,L
	addi	05000H,A0
	move	A0,*A8(OYVEL),L
	jrn	RAMBBLW2
	calla	DFRMGRND
	cmpi	140,A1
	jrlt	RAMBBLW4a
	jruc	RAMBBLW2
RAMBBLW4a:
	movk	4,A1
	jsrp	FRANIM
RAMBBLW4:
	callr	ZCHECK
	movk	1,A0
	calla	PRCSLP
	move	*A8(OYVEL),A0,L
	addi	05000H,A0
	move	A0,*A8(OYVEL),L
	calla	DFRMGRND
;	subk	5,A1
	jrp	RAMBBLW4

;* we have hit the ground..
	calla	ClrVel
	move	A8,A0
	calla	SETZPOS

	movi	-048000H,A0
	move	A0,*A8(OYVEL),L
	clr	A0
	movb	A0,*A13(ANIMTIKS)
BnceLp:
	movb	*A13(ANIMTIKS),A0
	jrnz	SkBncAni
	movk	4,A1
	jsrp	FRANIM
SkBncAni:
	dec	A0
	movb	A0,*A13(ANIMTIKS)

	movk	1,A0
	calla	PRCSLP

	move	*A8(OYVEL),A0,L
	addi	06000H,A0
	move	A0,*A8(OYVEL),L
	jrn	BnceLp

	calla	DFRMGRND
	cmpi	5,A1
	jrgt	BnceLp

	calla	ClrVel
	move	A8,A0
	calla	DELSHAD
	
	movb	*A13(ANIMTIKS),A0
	calla	PRCSLP

	movk	1,A1
	jsrp	FRANIM

	movk	20,A0
	calla	PRCSLP
	MOVI	[1,1],A9
	CREATE	0DFh,ESINK		;CREATE THE DEADMAN SINK	 
	DIE				;AND ME									 
;	move	A8,A0
;	calla	DELENEMY
;	DIE
**************************************************************************

*************************************************************************
RAMBCOLL:
	.WORD	B1OID|BULLETID
	.LONG	RAMBCOL0
	.WORD	B1OID|EXPLODID
	.LONG	RAMBCOL0
	.WORD	B1OID|ROCKETID
	.LONG	RAMBCOL0
	.WORD	B2OID|BULLETID
	.LONG	RAMBCOL0
	.WORD	B2OID|EXPLODID
	.LONG	RAMBCOL0
	.WORD	B2OID|ROCKETID
	.LONG	RAMBCOL0
	.WORD	BEOID|EXPLODID
	.LONG	RAMBCOL0
	.word	C1OID		;ENEMY CAR #1 "CADILLAC" OBJECT I.D.
	.LONG	CARCOL0
	.word	CAR2OID		;ENEMY CAR #2 "PORSCHE" OBJECT I.D.
	.LONG	CARCOL0
	.word	CAR3OID		;ENEMY CAR #3 "BELL-JET" OBJECT	I.D.
	.LONG	CARCOL0
	.word	TRAN1OID	;PLAYER TRANSPORT #1 "PORSCHE" OBJECT I.D.
	.LONG	CARCOL0
	.WORD	0
	.LONG	DUMRTSG
**************************************************************************


**************************************************************************
FIRE:
	mmtm	SP,A4
	CREATE	MBBULPID,BigBull
	jrz	NoFire
	move	A0,A1
	SWAP	A8,A10
	callr	BullSet
	SWAP	A8,A10
NoFire:
	mmfm	SP,A4
	rets
**************************************************************************

**************************************************************************
FLAMER:
	mmtm	SP,A0,A1,A2,A3,A4,A5,A6,A7

	movi	BullOff,A7
	movb	*A13(DIR),A0
	clr	A6		;A6 - FLAMEDIR
	cmpi	2,A0
	jreq	SkFDir
	movk	1,A6
SkFDir:
	sll	5,A0		;Each entry is 4 bytes
	add	A0,A7

	move	*A10(OXPOS),A3,W
	movb	*A7,A2
	addk	8,A7
	add	A2,A3		;A3 - Xpos

	move	*A10(OYPOS),A4,W
	movb	*A7,A2
	add	A2,A4		;A4 - Ypos

	move	*A10(OZPOS),A5,W	;A5 - Zpos

	move	*A10(OXVEL),A0,L
	move	*A10(OZVEL),A2,L
;	move	*A10(OYVEL),A1,L
	move	A2,A1

	calla	MAKEFLAM

	mmfm	SP,A0,A1,A2,A3,A4,A5,A6,A7
	rets
**************************************************************************

**************************************************************************
BullOff:
	;offsets X,Y(from TL of frame),Z,pad
	.byte	60,19,-15,0	;Dir = 0
	.byte	63,29,-8,0	;Dir = 1
	.byte	72,39, 0,0	;Dir = 2
	.byte	64,47, 8,0	;Dir = 3
	.byte	60,53,15,0	;Dir = 4
	.byte	6 ,55,20,0	;Dir = 5
	.byte	0 ,53,15,0	;Dir = 6
	.byte	0 ,47, 8,0	;Dir = 7
	.byte	3, 39, 0,0	;Dir = 8
	.byte	0 ,29,-8,0	;Dir = 9
	.byte	0 ,19,-15,0	;Dir = 10

**************************************************************************
FLAMINIT:
	.LONG	0,0,0,0
	.WORD	0,0
	.LONG	SFLAME,00H,DUMCOLL	;ZSIZ
	.WORD	DMAWNZ,EFF0OID	;OBJ ID
	.LONG	0
**************************************************************************

**************************************************************************
BullSet:
;A1 - ptr to bullet process
;* set following fields in proc block
;ETEMP1-Xvel	
;ETEMP2-Zvel
;ETEMP3-Xpos
;ETEMP3+16-Ypos
;ETEMP4-Zpos

	movi	BullOff,A7
	movb	*A13(DIR),A0
	sll	5,A0		;Each entry is 4 bytes
	add	A0,A7

	callr	BullXV0
	move	A0,*A1(ETEMP1),L	;set XVEL
	callr	BullZV0
	move	A0,*A1(ETEMP2),L	;set ZVEL

	move	*A8(OXPOS),A0,W
	movb	*A7,A2
	addk	8,A7
	add	A2,A0
	move	A0,A4
	move	A0,*A1(ETEMP3),W	;set XPOS

	move	*A8(OYPOS),A0,W
	movb	*A7,A2
	addk	8,A7
	add	A0,A2
	subk	4,A2	;height of bullet
	move	A2,*A1(ETEMP3+16),W	;set YPOS
	addk	4,A2

	movb	*A7,A0
	move	*A8(OZPOS),A3,W
	add	A0,A3
	move	A3,*A1(ETEMP4),W	;set ZPOS

	;* Make A Flash For The Bullet
	move	A8,A6	;save A8
	calla	GETOBJ
	jrz	NoFObj
	move	A0,A8
	movi	FLAMINIT,A14
	calla	STFOBJ
	movb	*A13(DIR),A11	;pass direction of flash
	CREATE	0EFFH,Flash	;SOMETHING TO WATCH THE FLASH
	jrz	NoFProc

	move	A4,*A8(OXPOS),W
	move	A3,*A8(OZPOS),W
	move	A2,*A8(OYPOS),W
	move	*A6(OXVEL),*A8(OXVEL),L
	move	*A6(OZVEL),*A8(OZVEL),L
	move	*A6(OYVEL),*A8(OYVEL),L
NoFObj:
	move	A6,A8   ;restore A8
	rets

NoFProc:
	calla	FREEOBJ
	jruc	NoFObj
**************************************************************************


**************************************************************************
Flash:
	MOVI	NARCS,A0
	CALLA	GETFPAL			;GET A COLOR MAP ASSIGNMENT
	jrz	PalX
	MOVE	A0,*A8(OPAL),W

	move	A13,*A8(OPLINK),L

	movi	SFLAMES,A9
	sll	6,A11		;shift by 2 longwords
	add	A11,A9
	movk	4,A1
	jsrp	FRANIM

	move	*A8(OYPOS),A0,W
	move	*A8(OIMG),A1,L
	move	*A1(IANIOFFY),A1,W
	sub	A1,A0
	move	A0,*A8(OYPOS),W

	move	*A8(OFLAGS),A0,W
	btst	M_FLIPH,A0
	jrnz	SkFlX
	move	*A8(OXPOS),A0,W
	move	*A8(OSIZEX),A1,W
	sub	A1,A0
	move	A0,*A8(OXPOS),W
SkFlX:

	move	A8,A0
	calla	INSOBJ

	SLEEP	4

	move	A8,A0			;GET THE BLOCK
	CALLA	DELPAL
PalX:
	move	A8,A0
	calla	DELOBJ
	DIE
**************************************************************************


**************************************************************************
BullXV0:
	mmtm	SP,A3
	movb	*A13(DIR),A0
	sll	4,A0
	addi	BXVDTbl,A0
	move	*A0,A0,W
	movi	0E00H,A3		;0E00H << 7  =  070000H
	mpys	A0,A3
	move	A3,A0
	mmfm	SP,A3
	rets
BXVDTbl:
	;07fH = 1 in .8 fmt, 078H = .94 = cos(20) in .8 fmt
	.word	078H,07EH,07FH,07EH,078H,0,-078H,-07EH,-07FH,-07EH,-078H
**************************************************************************

**************************************************************************
BullZV0:
	mmtm	SP,A3
	movb	*A13(DIR),A0
	sll	4,A0
	addi	BZVDTbl,A0
	move	*A0,A0,W
	movi	0E00H,A3
	mpys	A0,A3
	move	A3,A0
	mmfm	SP,A3
	rets
BZVDTbl:
	;021H = .26 = sin(12) in .8 fmt
	.word	-02CH,-016H,0,016H,02CH,07FH,02CH,016H,0,-016H,-02CH
	rets
**************************************************************************


**************************************************************************
*INIT TABLE
*	.LONG	OXVEL,OYVEL,OXVAL,XYVAL
*	.WORD	OZPOS,OZVEL
*	.LONG	OIMG,OZSIZ,OCVECT
*	.WORD	OFLAGS,OID
*	.LONG	SCANNERBLIP
BullInit:
	.LONG	0,0,0,0
	.WORD	0,0
	.LONG	RAMBBULL,000020002H,BullCol
	.WORD	DMAWNZ,BEOID
	.LONG	0
**************************************************************************

**************************************************************************
BigBull:
;* Mr. Big Bullet Process
	MOVI	EBSND,A0
	CALLA	ONESND		;make bullet fire sound
	calla	GETOBJ
	jrz	BigBullX
	move	A0,A8
	movi	BullInit,A14
	calla	STFOBJ

	;* get parms from creator
	move	*A13(ETEMP1),*A8(OXVEL),L
	move	*A13(ETEMP2),A1,L
	move	A1,*A8(OYVEL),L
	movi	04000H,A0
	calla	RANDU
	subi	02000H,A0
	add	A0,A1
	move	A1,*A8(OZVEL),L
	move	*A13(ETEMP3),*A8(OXPOS),W
	move	*A13(ETEMP3+16),*A8(OYPOS),W
	move	*A13(ETEMP4),*A8(OZPOS),W

	move	*A8(OXVEL),A0,L
	jrn	NoFlp
	calla	YFLP
	jruc	FlpSet
NoFlp:
	calla	NOYFLP
FlpSet:

	;* allocate palette
	MOVI	NARCS,A0
	CALLA	GETFPAL			;GET A COLOR MAP ASSIGNMENT
	jrz	NoPal
	MOVE	A0,*A8(OPAL),W

	move	A13,*A8(OPLINK),L
	move	A8,A0
	calla	INSOBJ
	movi	ENEMY_BULLETS,A1
	move	A1,A10
	calla	ADDSUPP			;ADD TO SUPPLEMENTAL List
	calla	SHADST

	movi	70,A0		;dflt max sleep for bullets
	calla	BullSlp
	PUSHP	A11
	move	A0,A11

BullFly0:
	movi	BIGBULL,A9
BullFly:
	move	A11,A11
	jrz	BullDone
	movk	4,A1
	jsrp	FRANIM
	cmp	A0,A11
	jrge	SlpOk
	move	A11,A0
SlpOk:
	sub	A0,A11
	calla	PRCSLP
	move	*A9,A1,L
	jrnz	BullFly
	jruc	BullFly0

BullDone:
	PULLP	A11
	move	A11,A11
	jrz	SkWallHit
	MOVI	ENEMY_BULLETS,A1
	MOVE	A8,A0
	CALLA	DELSUPP			;DELETE FROM THE SUPPLEMENTAL LIST
	jauc	DINGANI
SkWallHit:
	move	A8,A0			;GET THE BLOCK
	CALLA	DELPAL
	movi	ENEMY_BULLETS,A1
	calla	DELSUPP			;DELETE FROM THE SUPPLEMENTAL LIST
	calla	DELSOBJ			;ZAP IT
BigBullX:
	DIE
NoPal:
	calla	FREEOBJ
	DIE
**************************************************************************

**************************************************************************
BullCol:
	.WORD	P1OID
	.LONG	bcol
	.WORD	P2OID
	.LONG	bcol
	.WORD	B1OID|BULLETID
	.LONG	bcol
	.WORD	B1OID|EXPLODID
	.LONG	bcol
	.WORD	B1OID|ROCKETID
	.LONG	bcol
	.WORD	B2OID|BULLETID
	.LONG	bcol
	.WORD	B2OID|EXPLODID
	.LONG	bcol
	.WORD	B2OID|ROCKETID
	.LONG	bcol

	.WORD	BGGID
	.LONG	BCOLDCAN
	.WORD	BGLID
	.LONG	BCOLDPST
	.WORD	BGDMPID
	.LONG	BCOLDDMP
	.WORD	C1OID|CBODYID
	.LONG	BCOLDDMP
	.WORD	C1OID|WSHLDID
	.LONG	BCOLDDMP
	.WORD	C1OID|TRUNKID
	.LONG	BCOLDDMP
	.WORD	B1OID|BULLETID
	.LONG	BCOLDPST
	.WORD	B1OID|ROCKETID
	.LONG	BCOLDPST
	.WORD	B1OID|FLAMEID
	.LONG	BCOLL1
	.WORD	B2OID|BULLETID
	.LONG	BCOLDPST
	.WORD	B2OID|ROCKETID
	.LONG	BCOLDPST
	.WORD	B2OID|FLAMEID
	.LONG	BCOLL1
	.WORD	INTR1OID|CBODYID
	.LONG	BCOLDDMP
	.WORD	INTR1OID|WSHLDID
	.LONG	BCOLDDMP
	.WORD	INTR1OID|TRUNKID
	.LONG	BCOLDDMP
	.WORD	TRAN1OID|CBODYID
	.LONG	BCOLDDMP
	.WORD	TRAN1OID|WSHLDID
	.LONG	BCOLDDMP
	.WORD	TRAN1OID|TRUNKID
	.LONG	BCOLDDMP
	.WORD	CSTR1OID|CBODYID
	.LONG	BCOLDDMP
	.WORD	CSTR1OID|WSHLDID
	.LONG	BCOLDDMP
	.WORD	CSTR1OID|TRUNKID
	.LONG	BCOLDDMP
	.WORD	0
	.LONG	DUMRTSG
**************************************************************************

**************************************************************************
bcol:
	MOVI	DUMCOLL,A14
	MOVE	A14,*A0(OCVECT),L	;COLLISION POINTER
	CALLA	DELPAL
	movi	ENEMY_BULLETS,A1
	CALLA	DELSUPP			;DELETE FROM THE SUPPLEMENTAL LIST
	CALLA	DELSOBJ
	MOVE	*A0(OPLINK),A0,L
	CALLA	KILL			;KILL CONTROL
	
	clr	A0	;don't teminate collision scan
	rets
**************************************************************************


**************************************************************************
;WALKING JUMPING DEATH FRAGMENT STANDING-SHOOT KNEELING-SHOOT
;STANDING KNEELING

RAMBOFF	.WORD	1,3,4,9,18,34,50,50
*ANIMATION PTR TABLE, FIRST LONG WORD IS THE OFFSET PTR
*
RAMBTAB	.LONG	RAMBOFF
	.LONG	WALK,WALK,WALK,RAMBDBUL,RAMBDFLA,RAMBDEXP,RAMBDEXP1,RAMBDROCK

	.LONG	RAMBRAG1,RAMBRAG2,RAMBRAG2,RAMBRAG3
	.LONG	RAMBRAG1,RAMBRAG2,RAMBRAG3,RAMBRAG4,0
*************************************************************************



SFLAMES:
	.long	SFLAME20
	.word	FLIPBITS|1,(M_FLIPV)
	.long	SFLAME10
	.word	FLIPBITS|1,(M_FLIPV)
	.long	SFLAME
	.word	FLIPBITS|1,0
	.long	SFLAME10
	.word	FLIPBITS|1,0
	.long	SFLAME20
	.word	FLIPBITS|1,0
	.long	SFLAME90
	.word	FLIPBITS|1,0
	.long	SFLAME20
	.word	FLIPBITS|1,(M_FLIPH)
	.long	SFLAME10
	.word	FLIPBITS|1,(M_FLIPH)
	.long	SFLAME
	.word	FLIPBITS|1,(M_FLIPH)
	.long	SFLAME10
	.word	FLIPBITS|1,(M_FLIPH|M_FLIPV)
	.long	SFLAME20
	.word	FLIPBITS|1,(M_FLIPH|M_FLIPV)

BIGBULL:
	.long	RAMBBULL
	.word	200
	LONGNULL


* RAMBO FRAME TABLES	
* NORMAL TABLE ENTRY:
* 	AFRAME	FRAME,SLEEPT,CONTROL_WORD
* FIRE FRAME TABLE ENTRY:		    
* 	FAFRAME	FRAME,SLEEPT,XOFFSET,YOFFSET,PROCID,PROCESS_ADDRESS,CW

FRANMIN:

;TORSO FRAMES
TORSOS:
	.long	rmshb20
	.word	FLIPBITS|1,0
	.long	rmshb10
	.word	FLIPBITS|1,0
	.long	rmsh0a
	.word	FLIPBITS|1,0
	.long	rmshf10
	.word	FLIPBITS|1,0
	.long	rmshf20
	.word	FLIPBITS|1,0
	.long	rmshf90
	.word	FLIPBITS|1,0
	.long	rmshf20
	.word	FLIPBITS|1,(M_FLIPH)
	.long	rmshf10
	.word	FLIPBITS|1,(M_FLIPH)
	.long	rmsh0a
	.word	FLIPBITS|1,(M_FLIPH)
	.long	rmshb10
	.word	FLIPBITS|1,(M_FLIPH)
	.long	rmshb20
	.word	FLIPBITS|1,(M_FLIPH)
FTORSOS:
	.long	rmshb20
	.word	FLIPBITS|1,0
	.long	rmshb10
	.word	FLIPBITS|1,0
	.long	rmsh0b
	.word	FLIPBITS|1,0
	.long	rmshf10
	.word	FLIPBITS|1,0
	.long	rmshf20
	.word	FLIPBITS|1,0
	.long	rmshf90
	.word	FLIPBITS|1,0
	.long	rmshf20
	.word	FLIPBITS|1,(M_FLIPH)
	.long	rmshf10
	.word	FLIPBITS|1,(M_FLIPH)
	.long	rmsh0b
	.word	FLIPBITS|1,(M_FLIPH)
	.long	rmshb10
	.word	FLIPBITS|1,(M_FLIPH)
	.long	rmshb20
	.word	FLIPBITS|1,(M_FLIPH)


WALKDIST	.set	9+13+16+11+6+7+17+8	
WALKVEL	.set	018000H
RUNVEL	.set	038000H
MOVETBL:
WALK:	  ;* sleep is 95% of delta
;	.long	legs1
;	.word	6
;	.long	legs2
;	.word	6
;	.long	legs3
;	.word	6
;	.long	legs4
;	.word	6
;	.long	legs5
;	.word	6
;	.long	legs6
;	.word	6
;	.long	legs7
;	.word	6
;	LONGNULL
WALKEND:
RUN:	  ;* sleep is 45% of delta
	.long	rmrun4
	.word	5
	.long	rmrun5
	.word	5
	.long	rmrun6
	.word	5
	.long	rmrun7
	.word	5
	.long	rmrun8
	.word	5
	.long	rmrun1
	.word	5
	.long	rmrun2
	.word	5
	.long	rmrun3
	.word	5
	LONGNULL
RUNEND:
MOVEEND:

RAMBJMP:
	.long	rmrun7
	.word	1
	.long	rmrun8
	.word	1
	.long	rmrun1
	.word	1
	.long	rmrun3
	.word	1
	LONGNULL

RAMBDBUL:
	.long	rmdead1
	.word	9
	.long	rmdead6
	.word	8
	.long	rmdead7
	.word	7
	.long	rmdead8
	.word	7
	.long	rmdead9
	.word	7
	LONGNULL

RAMBBLOW:
	.long	rmdth1
	.word	20
	.long	rmdth2
	.word	20
	.long	rmdth3
	.word	20
	.long	rmdth4
	.word	10
	.long	rmdth5
	.word	4
	.long	rmdth4
	.word	4
	.long	rmdth3
	.word	12
	.long	rmdth4
	.word	10
	.long	rmdth5
	.word	10
	LONGNULL

RAMBDFLA:				;DEATH BY FLAME
	.long	BGDTHF1
	.word	NEWPALET|6
	.LONG	NARCS
	.long	BGDTHF2
	.word	4
	.long	BGDTHF3
	.word	4
	.long	BGDTHF4
	.word	4
	.LONG	0

RAMBDROCK:				;RAMB HIT BY ROCKET
	.long	LCREOSOTE
	.word	NEWPALET|5
	.LONG	NARCS
	.LONG	0

RAMBRAG1
	.long	LLEG1
	.word	NEWPALET|2
	.LONG	NARCS
	.long	LLEG2
	.word	2
	.long	LLEG3
	.word	2
	.long	LLEG4
	.word	2
	.long	LLEG5
	.word	2
	.long	LLEG6
	.word	2
	.long	LLEG7
	.word	2
	.long	LLEG8
	.word	2
	.LONG	0

RAMBRAG2
	.long	L4ARM1
	.word	NEWPALET|2
	.LONG	NARCS
	.long	L4ARM2
	.word	2
	.long	L4ARM3
	.word	2
	.long	L4ARM4
	.word	2
	.long	L4ARM5
	.word	2
	.long	L4ARM6
	.word	2
	.long	L4ARM7
	.word	2
	.long	L4ARM8
	.word	2
	.LONG	0

RAMBRAG3
	.long	LSHIN1
	.word	NEWPALET|2
	.LONG	NARCS
	.long	LSHIN2
	.word	2
	.long	LSHIN3
	.word	2
	.long	LSHIN4
	.word	2
	.long	LSHIN5
	.word	2
	.long	LSHIN6
	.word	2
	.long	LSHIN7
	.word	2
	.long	LSHIN8
	.word	2
	.LONG	0

RAMBRAG4
	.long	LGOAT1
	.word	NEWPALET|2
	.LONG	NARCS
	.long	LGOAT2
	.word	2
	.long	LGOAT3
	.word	2
	.long	LGOAT4
	.word	2
	.long	LGOAT5
	.word	2
	.long	LGOAT6
	.word	2
	.long	LGOAT7
	.word	2
	.long	LGOAT8
	.word	2
	.long	LGOAT9
	.word	2
	.long	LGOAT10
	.word	2
	.LONG	0

RAMBDEXP:				;DEATH BY EXPLOSION CLOCKWISE SPIN
	.long	BGDF1C
	.word	FLIPBITS|NEWPALET|2
	.WORD	0
	.LONG	NARCS
	.long	BGDF2C
	.word	FLIPBITS|2,0
	.long	BGDF3C
	.word	FLIPBITS|2,0
	.long	BGDF4C
	.word	FLIPBITS|2,0
	.long	BGDF3C
	.word	FLIPBITS|2,M_FLIPV
	.long	BGDF2C
	.word	FLIPBITS|2,M_FLIPV
	.long	BGDF1C
	.word	FLIPBITS|2,M_FLIPV
	.long	BGDF1C
	.word	FLIPBITS|2,(M_FLIPH+M_FLIPV)
	.long	BGDF2C
	.word	FLIPBITS|2,(M_FLIPH+M_FLIPV)
	.long	BGDF3C
	.word	FLIPBITS|2,(M_FLIPH+M_FLIPV)
	.long	BGDF4C
	.word	FLIPBITS|2,(M_FLIPH)
	.long	BGDF3C
	.word	FLIPBITS|2,(M_FLIPH)
	.long	BGDF2C
	.word	FLIPBITS|2,(M_FLIPH)
	.long	BGDF1C
	.word	FLIPBITS|2,(M_FLIPH)
	.LONG	0

RAMBDEXP1:				;DEATH BY EXPLOSION COUNTER-CLOCK SPIN
	.long	BGDF1C
	.word	FLIPBITS|NEWPALET|2
	.WORD	M_FLIPH
	.LONG	NARCS
	.long	BGDF2C
	.word	FLIPBITS|2,(M_FLIPH)
	.long	BGDF3C
	.word	FLIPBITS|2,(M_FLIPH)
	.long	BGDF4C
	.word	FLIPBITS|2,(M_FLIPH)
	.long	BGDF3C
	.word	FLIPBITS|2,(M_FLIPH+M_FLIPV)
	.long	BGDF2C
	.word	FLIPBITS|2,(M_FLIPH+M_FLIPV)
	.long	BGDF1C
	.word	FLIPBITS|2,(M_FLIPH+M_FLIPV)
	.long	BGDF1C
	.word	FLIPBITS|2,M_FLIPV
	.long	BGDF2C
	.word	FLIPBITS|2,M_FLIPV
	.long	BGDF3C
	.word	FLIPBITS|2,M_FLIPV
	.long	BGDF4C
	.word	FLIPBITS|2,0
	.long	BGDF3C
	.word	FLIPBITS|2,0
	.long	BGDF2C
	.word	FLIPBITS|2,0
	.long	BGDF1C
	.word	FLIPBITS|2,0
	.LONG	0
FRANMAX:

RAMBNUL
	.LONG	0

RAMBPALS:
	.long	ram1
	.long	grnram

grnram:	;PAL #0
	.word	240	;pal size
	.word 0460H,0460H,0480H,0480H,0480H,0480H,04A0H,04A1H,04A0H,04A1H
	.word 04A0H,04A0H,04C1H,04C1H,04C0H,04C1H,04E2H,04C0H,04E1H,04E2H
	.word 0502H,0CC0H,014A0H,014C0H,0CE0H,014E0H,0CE1H,0CE1H,0CE2H,014E1H
	.word 0D02H,01500H,01501H,01502H,01522H,01501H,01502H,01521H,01522H,0D03H
	.word 0D03H,01503H,01523H,01503H,01523H,01543H,01544H,01565H,01CE0H,01D01H
	.word 01D01H,01D21H,01D21H,04061H,02500H,02521H,01D02H,01D22H,01D23H,01D24H
	.word 01D02H,01D22H,01D23H,02502H,02522H,02D22H,01D41H,01D42H,01D43H,01D43H
	.word 01D44H,01D44H,02542H,02542H,02D42H,02543H,02544H,02543H,02D44H,01D63H
	.word 01D64H,02563H,02564H,02563H,02564H,02584H,02D62H,02D63H,02D63H,02D64H
	.word 02D83H,02D84H,02DA4H,01D65H,01D65H,02565H,02566H,02585H,02D85H,02D85H
	.word 02D86H,025A7H,02DA6H,02DA7H,02DA6H,02DC6H,04CA2H,03542H,03562H,03583H
	.word 03583H,035A3H,03584H,03584H,03585H,035A4H,035A4H,035A6H,035A6H,060A2H
	.word 060E3H,05CE5H,05944H,06524H,05D46H,07483H,07103H,07D04H,074E6H,07CE6H
	.word 07527H,07D27H,035C5H,03DC4H,035C6H,03DE6H,035C7H,035E8H,03DE7H,03E07H
	.word 045C5H,03DE6H,045E6H,03DE7H,045E7H,04606H,04607H,04628H,04648H,04DE5H
	.word 055E5H,04DE7H,05607H,04E27H,04E48H,05627H,05668H,065E4H,06605H,05E26H
	.word 06646H,05E08H,06647H,05E68H,06668H,0360AH,03E2AH,04629H,0464AH,04E0AH
	.word 04E4AH,0466AH,0468CH,04E6AH,04E8AH,04E8BH,04EABH,04E8CH,05DE9H,05E2AH
	.word 05E4AH,05669H,06669H,05E6BH,0666BH,056AAH,05E8AH,06689H,0668AH,066AAH
	.word 056ABH,056ABH,056ADH,066ABH,066ADH,04ECEH,056CCH,056CCH,05ECCH,05F0FH
	.word 05EEBH,05EEEH,05F0FH,066CBH,066CCH,066EEH,07ECDH,07ECFH,07ED1H,07EF1H
	.word 07F31H,06EF3H,07B13H,07ED3H,07F34H,07F54H,07FB5H,07F78H,07FBCH,07FF9H
	.word 07FFBH,07FFEH,07FFFH,02DE9H,05441H,06021H,06842H,07421H,07942H,07086H
	.word 06906H,07145H,06948H,07E68H,07F2EH,07F91H,07FD2H,04821H,05C64H,068C5H


CRMRUN:	  ;* sleep is 45% of delta
	.long	rmrun4
	.word	FLIPBITS|1,0
	.long	rmrun5
	.word	5
	.long	rmrun6
	.word	5
	.long	rmrun7
	.word	5
	.long	rmrun8
	.word	5
	.long	rmrun1
	.word	5
	.long	rmrun2
	.word	5
	.long	rmrun3
	.word	5
	.long	rmrun4
	.word	5
	.long	rmrun5
	.word	5
	.long	rmrun6
	.word	5
	.long	rmrun7
	.word	5
	LONGNULL

CRMSHT:	  ;* sleep is 45% of delta
	.long	rmsh0a
	.word	FLIPBITS|1,0
	.long	rmsh0b
	.word	5
	.long	rmsh0a
	.word	5
	.long	rmsh0b
	.word	5
	.long	rmsh0a
	.word	5
	.long	rmsh0b
	.word	5
	.long	rmsh0a
	.word	5
	.long	rmsh0b
	.word	5
	.long	rmsh0a
	.word	5
	.long	rmsh0b
	.word	5
	.long	rmsh0a
	.word	5
	.long	rmsh0b
	.word	5
	.long	rmsh0a
	.word	5
	.long	rmsh0b
	.word	5
	LONGNULL


****************************************************************************
RAMBCRM1:
	move	@(WORLDTLX+16),A0,W				 
	addi	130,A0
	move	@PLYMIN,A1,W									 
	addi	10,A1
	sll	16,A1
	movx	A0,A1
	movi	BodyInit,A14	
	calla	ESINITLL
	PUSHP	A8
	movi	LegsInit,A14	
	calla	ESINITLL
	PULLP	A10

	clr	A0
	move	A0,*A8(OBLIPIMG),L

	clr	A0
	movb	A0,*A13(SNOOZE)

	calla	ClrVel
	calla	NOYFLP
	movi	RUNVEL,A0
	move	A0,*A8(OXVEL),L

	movi	CRMRUN,A9	
	movi	CRMSHT,A11	
	movk	10,A0
CrmLp:
	PUSHP	A0
	movk	4,A1
	jsrp	FRANIM
	SWAP	A9,A11
	SWAP	A8,A10
	movk	4,A1
	jsrp	FRANIM
	SWAP	A9,A11
	SWAP	A8,A10
	callr	UpdTorso
	movk	5,A0
	calla	PRCSLP
	PULLP	A0
	dec	A0
	jrp	CrmLp

	MOVI	ENEMIES,A1
	MOVE	A8,A0
	CALLA	DELSUPP
	move	*A8(OPAL),A1,W
	move	A1,@CRIMEPAL,W
	CALLA	DELSOBJ

	MOVE	A10,A0
	MOVI	ENEMIES,A1
	CALLA	DELSUPP
	CALLA	DELSOBJ

	retp
****************************************************************************

