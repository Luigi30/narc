
	.include	imgtbl.glo
	.sect	"img_tbl"
	.even
RRD0:
	.word	10,12,0,0
	.long	02000000H,NARCS
RRD1:
	.word	10,12,0,0
	.long	02000480H,NARCS
RRD2:
	.word	10,12,0,0
	.long	02000900H,NARCS
RRD3:
	.word	10,12,0,0
	.long	02000D80H,NARCS
RRD4:
	.word	10,12,0,0
	.long	02001200H,NARCS
RRD5:
	.word	10,12,0,0
	.long	02001680H,NARCS
RRD6:
	.word	10,12,0,0
	.long	02001B00H,NARCS
RRD7:
	.word	10,12,0,0
	.long	02001F80H,NARCS
RRD8:
	.word	10,12,0,0
	.long	02002400H,NARCS
RRD9:
	.word	10,12,0,0
	.long	02002880H,NARCS

	.include NARCDRIV.GLO
DRIVER1:
	.word	38,17,10,-133
	.long	02046CA0H,NARCS
DRIVER2:
	.word	38,18,12,-132
	.long	020481E0H,NARCS
DRIVER3:
	.word	38,17,12,-133
	.long	02049860H,NARCS
DRIVER4:
	.word	38,17,10,-133
	.long	0204ADA0H,NARCS
DRIVER5:
	.word	39,17,10,-133
	.long	0204C2E0H,NARCS
DRVDTH1:
	.word	37,16,9,-134
	.long	0204D820H,NARCS
DRVDTH2:
	.word	36,19,10,-130
	.long	0204EC20H,NARCS
DRVDTH3:
	.word	48,26,34,-124
	.long	02050180H,NARCS
DRVDTH4:
	.word	59,19,51,-131
	.long	02052880H,NARCS
DRVDTH5:
	.word	86,26,102,-140
	.long	02054C20H,NARCS
DRIVEF1:
	.word	37,18,11,-132
	.long	020593A0H,NARCS
DRIVEF2:
	.word	47,20,11,-130
	.long	0205AA20H,NARCS
DRIVEF3:
	.word	43,28,12,-130
	.long	0205C820H,NARCS
DRIVEF4:
	.word	48,31,13,-124
	.long	0205EEA0H,NARCS
SCORESIDE:
	.word	169,81,0,0
	.long	02061D20H,NARCS
SCORETOP:
	.word	172,17,0,0
	.long	0207D080H,NARCS
SCOREBOT:
	.word	172,14,0,0
	.long	02082BE0H,NARCS
CAN1:
	.word	31,47,0,0
	.long	02087720H,NARCS
CAN2:
	.word	32,48,0,0
	.long	0208A620H,NARCS
CAN3:
	.word	58,31,0,0
	.long	0208D620H,NARCS
DUMP:
	.word	91,73,0,0
	.long	02091040H,NARCS
DSLIME:
	.word	63,20,0,0
	.long	0209E220H,NARCS
LAMPTOP:
	.word	33,30,0,0
	.long	020A0A20H,NARCS
LAMPOST:
	.word	9,175,0,0
	.long	020A2BE0H,NARCS
FLHIT1:
	.word	16,36,15,19
	.long	020A6D80H,NARCS
FLHIT2:
	.word	14,36,13,18
	.long	020A7F80H,NARCS
FLHIT3:
	.word	16,36,14,17
	.long	020A9180H,NARCS
ROCKET1:
	.word	20,13,7,6
	.long	020AA380H,NARCS
ROCKET2:
	.word	19,27,7,13
	.long	020AABA0H,NARCS
ROCKET3:
	.word	24,47,12,25
	.long	020ABC80H,NARCS
ROCKET4:
	.word	14,3,0,1
	.long	020ADFC0H,NARCS
ROCKBIG:
	.word	29,6,14,3
	.long	020AE140H,NARCS
PUF1:
	.word	43,38,20,18
	.long	020AE740H,NARCS
PUF2:
	.word	61,16,37,9
	.long	020B1B80H,NARCS
PUF3:
	.word	63,19,42,8
	.long	020B3B80H,NARCS
PUF4:
	.word	44,17,32,9
	.long	020B6180H,NARCS
PUF5:
	.word	23,14,14,8
	.long	020B78E0H,NARCS
TPART1:
	.word	408,18,407,10
	.long	020B8360H,NARCS
TPART2:
	.word	394,18,392,9
	.long	020C68E0H,NARCS
BLAM1:
	.word	20,21,10,10
	.long	020D47A0H,NARCS
BLAM2:
	.word	40,39,20,20
	.long	020D54C0H,NARCS
BLAM3:
	.word	59,57,30,28
	.long	020D8580H,NARCS
BLAM4:
	.word	84,82,42,39
	.long	020DF060H,NARCS
BLAM5:
	.word	128,125,65,57
	.long	020EC7A0H,NARCS
BLAM6:
	.word	138,138,70,68
	.long	0210BBA0H,NARCS
BLAM7:
	.word	150,141,72,69
	.long	02131760H,NARCS
BLAM8:
	.word	162,160,79,77
	.long	0215B520H,NARCS
GETCASH1:
	.word	24,6,12,2
	.long	0218E920H,NARCS
GETCASH2:
	.word	19,14,9,6
	.long	0218EDA0H,NARCS
GETROCK:
	.word	14,3,7,1
	.long	0218F660H,NARCS
GETCLIP:
	.word	19,14,0,0
	.long	0218F7E0H,NARCS
LDING1:
	.word	2,5,1,2
	.long	021900A0H,NARCS
LDING2:
	.word	6,5,5,2
	.long	02190140H,NARCS
LDING3:
	.word	8,10,7,4
	.long	02190280H,NARCS
LDING4:
	.word	5,9,4,4
	.long	02190500H,NARCS
LDING5:
	.word	4,9,4,4
	.long	02190740H,NARCS
DING45A:
	.word	27,17,13,-34
	.long	02190860H,NARCS
DING45B:
	.word	41,22,17,-33
	.long	02191740H,NARCS
DING90A:
	.word	19,26,11,-28
	.long	02193580H,NARCS
DING90B:
	.word	36,32,20,-25
	.long	021945C0H,NARCS
DOPE1:
	.word	13,19,6,1
	.long	021969C0H,NARCS
DOPE2:
	.word	12,22,5,4
	.long	02197340H,NARCS
DOPE3:
	.word	22,8,11,-10
	.long	02197B80H,NARCS
GETDOPE:
	.word	17,13,8,-5
	.long	02198180H,NARCS
DOPEICON:
	.word	26,15,0,0
	.long	021989A0H,NARCS
BUSTICON:
	.word	32,14,0,0
	.long	021996C0H,NARCS
BUSTED:
	.word	75,53,36,26
	.long	0219A4C0H,NARCS
POTICON:
	.word	14,14,0,0
	.long	021A22A0H,NARCS
LANDMINE:
	.word	29,13,0,0
	.long	021A29A0H,NARCS
FLHD1A:
	.word	30,16,1,9
	.long	021A36A0H,NARCS
FLHD1B:
	.word	30,12,2,5
	.long	021A46A0H,NARCS
FLHD1C:
	.word	29,12,1,6
	.long	021A52A0H,NARCS
FLHD2A:
	.word	86,17,2,9
	.long	021A5EA0H,NARCS
FLHD2B:
	.word	85,15,1,7
	.long	021A8D60H,NARCS
FLHD2C:
	.word	86,15,2,8
	.long	021AB6A0H,NARCS
FLTLA:
	.word	305,17,1,7
	.long	021ADFE0H,NARCS
FLTLB:
	.word	304,22,2,10
	.long	021B8380H,NARCS
FLTLC:
	.word	304,18,1,10
	.long	021C5480H,NARCS

	.include NARCFRAG.GLO
BGDF1C:
	.word	65,87,33,43
	.long	021CFF80H,NARCS
BGDF2C:
	.word	64,80,34,41
	.long	021DB860H,NARCS
BGDF3C:
	.word	66,73,32,49
	.long	021E5860H,NARCS
BGDF4C:
	.word	83,72,37,47
	.long	021EF380H,NARCS
LLEG1:
	.word	23,35,6,16
	.long	021FB080H,NARCS
LLEG2:
	.word	31,27,16,10
	.long	021FCAC0H,NARCS
LLEG3:
	.word	37,18,23,4
	.long	021FE5C0H,NARCS
LLEG4:
	.word	36,26,29,14
	.long	021FFC40H,NARCS
LLEG5:
	.word	21,34,15,29
	.long	02201980H,NARCS
LLEG6:
	.word	30,32,11,26
	.long	02203300H,NARCS
LLEG7:
	.word	37,22,12,18
	.long	02205300H,NARCS
LLEG8:
	.word	33,30,10,16
	.long	02206E80H,NARCS
L4ARM1:
	.word	10,26,2,19
	.long	02209040H,NARCS
L4ARM2:
	.word	21,18,5,12
	.long	02209A00H,NARCS
L4ARM3:
	.word	27,9,11,5
	.long	0220A780H,NARCS
L4ARM4:
	.word	21,18,11,10
	.long	0220AF60H,NARCS
L4ARM5:
	.word	10,26,4,18
	.long	0220BCE0H,NARCS
L4ARM6:
	.word	21,18,9,10
	.long	0220C6A0H,NARCS
L4ARM7:
	.word	27,9,12,5
	.long	0220D420H,NARCS
L4ARM8:
	.word	21,18,13,11
	.long	0220DC00H,NARCS
LSHIN1:
	.word	21,32,9,11
	.long	0220E980H,NARCS
LSHIN2:
	.word	23,31,14,14
	.long	02210180H,NARCS
LSHIN3:
	.word	32,22,18,10
	.long	022118C0H,NARCS
LSHIN4:
	.word	32,23,18,14
	.long	02212EC0H,NARCS
LSHIN5:
	.word	21,33,12,20
	.long	022145C0H,NARCS
LSHIN6:
	.word	24,34,8,20
	.long	02215E80H,NARCS
LSHIN7:
	.word	34,22,13,12
	.long	02217800H,NARCS
LSHIN8:
	.word	32,28,12,9
	.long	022190C0H,NARCS
LCREOSOTE:
	.word	94,95,46,-31
	.long	0221ACC0H,NARCS
BGDTHF1:
	.word	65,87,27,-33
	.long	021CFF80H,NARCS
BGDTHF2:
	.word	64,80,47,-37
	.long	021DB860H,NARCS
BGDTHF3:
	.word	66,73,65,-38
	.long	021E5860H,NARCS
BGDTHF4:
	.word	83,72,89,-51
	.long	021EF380H,NARCS
BGDTHF5:
	.word	104,82,113,-56
	.long	0222C9C0H,NARCS
BGDTHF6:
	.word	110,108,113,-28
	.long	0223D440H,NARCS
BGDTHF7:
	.word	81,77,97,-49
	.long	02254E40H,NARCS
BGDTHF8:
	.word	85,78,99,-48
	.long	02261860H,NARCS
BGDTHF9:
	.word	87,63,103,-63
	.long	0226EEE0H,NARCS
BGDTHF10:
	.word	87,52,102,-74
	.long	02279C20H,NARCS
BGDTHF11:
	.word	90,39,105,-87
	.long	02282B20H,NARCS
BGDTHF12:
	.word	87,49,102,-76
	.long	02289B40H,NARCS
BGDTHF13:
	.word	98,75,107,-50
	.long	02292200H,NARCS
BGDTHF14:
	.word	96,92,107,-33
	.long	022A0C60H,NARCS
BGDTHF15:
	.word	90,17,105,-109
	.long	022B2060H,NARCS
LGOAT1:
	.word	10,12,5,5
	.long	022B5140H,NARCS
LGOAT2:
	.word	12,11,5,5
	.long	022B55C0H,NARCS
LGOAT3:
	.word	14,11,6,5
	.long	022B59E0H,NARCS
LGOAT4:
	.word	14,13,5,7
	.long	022B5F60H,NARCS
LGOAT5:
	.word	12,12,5,6
	.long	022B65E0H,NARCS
LGOAT6:
	.word	12,11,5,6
	.long	022B6A60H,NARCS
LGOAT7:
	.word	13,14,6,9
	.long	022B6E80H,NARCS
LGOAT8:
	.word	12,13,6,7
	.long	022B7580H,NARCS
LGOAT9:
	.word	12,11,6,5
	.long	022B7A60H,NARCS
LGOAT10:
	.word	12,10,5,4
	.long	022B7E80H,NARCS
GOLDcard:
	.word	44,16,0,0
	.long	022B8240H,NARCS
BLUEcard:
	.word	44,16,0,0
	.long	022B9840H,NARCS
REDcard:
	.word	44,16,0,0
	.long	022BAE40H,NARCS
SILVcard:
	.word	44,16,0,0
	.long	022BC440H,NARCS
GREENcard:
	.word	44,16,0,0
	.long	022BDA40H,NARCS
GOLDTYPE:
	.word	30,28,15,-28
	.long	022BF040H,NARCS
BLUETYPE:
	.word	30,28,15,-28
	.long	022C0C40H,NARCS
REDTYPE:
	.word	30,28,15,-28
	.long	022C2840H,NARCS
SILVTYPE:
	.word	36,28,18,-28
	.long	022C4440H,NARCS
cardslot2:
	.word	24,74,23,0
	.long	022C63C0H,metal1
OPENTYPE:
	.word	32,8,16,-9
	.long	022C9B40H,NARCS
safedrl:
	.word	61,119,0,0
	.long	022CA340H,metal1
safedrr:
	.word	61,119,0,0
	.long	022D9140H,metal1
safedrlr:
	.word	61,119,0,0
	.long	022CA340H,redsafe
safedrrr:
	.word	61,119,0,0
	.long	022D9140H,redsafe
safedrlb:
	.word	61,119,0,0
	.long	022CA340H,blusafe
safedrrb:
	.word	61,119,0,0
	.long	022D9140H,blusafe
safedrlg:
	.word	61,119,0,0
	.long	022CA340H,goldsafe
safedrrg:
	.word	61,119,0,0
	.long	022D9140H,goldsafe
ARROWDN:
	.word	33,35,16,0
	.long	022E7F40H,NARCS
BIGDOOR1:
	.word	40,94,0,0
	.long	022EA6A0H,tileb
BIGDOOR2:
	.word	40,94,0,0
	.long	022F1C20H,tileb
BIGDOOR3:
	.word	40,94,0,0
	.long	022F91A0H,tileb
safdrlgr:
	.word	61,119,0,0
	.long	022CA340H,greensaf
safdrrgr:
	.word	61,119,0,0
	.long	022D9140H,greensaf
goldbr1:
	.word	51,24,0,0
	.long	02300720H,goldesca
goldbr2:
	.word	34,40,0,0
	.long	02302E20H,goldesca
GRNTYPE:
	.word	35,28,17,-28
	.long	02305B20H,NARCS
INSTYPE:
	.word	35,8,18,-9
	.long	02307AA0H,NARCS
SMblucrd:
	.word	11,4,0,0
	.long	023083A0H,NARCS
SMgldcrd:
	.word	11,4,0,0
	.long	02308520H,NARCS
SMredcrd:
	.word	11,4,0,0
	.long	023086A0H,NARCS
SMsilvcrd:
	.word	11,4,0,0
	.long	02308820H,NARCS
SMgrncrd:
	.word	11,4,0,0
	.long	023089A0H,NARCS
SEWERCAP:
	.word	32,21,0,0
	.long	02308B20H,NARCS
COLUMN1:
	.word	40,101,0,0
	.long	0230A020H,COLMNPAL
COLUMNB:
	.word	44,37,0,0
	.long	02311E60H,COLMNPAL
CHINADR1:
	.word	40,90,19,-15
	.long	02315140H,CHINAPAL
CHINADR2:
	.word	40,90,19,-15
	.long	0231C1C0H,CHINAPAL
CHINADR3:
	.word	40,90,19,-15
	.long	02323240H,CHINAPAL
ARMDR1:
	.word	40,94,19,-11
	.long	0232A2C0H,ARMDRPAL
ARMDR2:
	.word	40,94,19,-11
	.long	02331840H,ARMDRPAL
ARMDR3:
	.word	40,94,19,-11
	.long	02338DC0H,ARMDRPAL
GARAGE1:
	.word	100,26,0,0
	.long	02340340H,ARMDRPAL
HOTWIN1:
	.word	24,59,0,0
	.long	02345480H,HWINPAL
EXITSIGN:
	.word	59,33,29,20
	.long	023480C0H,NARCS
K2x5IMG:
	.word	31,10,0,0
	.long	0234BEA0H,NARCS
K10IMG:
	.word	36,10,0,0
	.long	0234C8A0H,NARCS
K1IMG:
	.word	29,10,0,0
	.long	0234D3E0H,NARCS
K5IMG:
	.word	31,10,0,0
	.long	0234DDE0H,NARCS
K25IMG:
	.word	38,10,0,0
	.long	0234E7E0H,NARCS
WEED:
	.word	62,115,0,0
	.long	0234F460H,WEEDPAL
CASHMACH:
	.word	16,44,15,0
	.long	0235DA60H,lbrass1
CWRECK:
	.word	250,71,126,-138
	.long	023FEB20H,caddy
CSHRAPA1:
	.word	3,5,1,3
	.long	02421A40H,NARCS
CSHRAPA2:
	.word	4,3,2,1
	.long	02421AE0H,NARCS
CSHRAPA3:
	.word	3,5,0,2
	.long	02421B40H,NARCS
CSHRAPA4:
	.word	3,2,1,1
	.long	02421BE0H,NARCS
CSHRAPA5:
	.word	3,2,1,0
	.long	02421C20H,NARCS
CSHRAPB1:
	.word	5,6,3,3
	.long	02421C60H,NARCS
CSHRAPB2:
	.word	6,4,3,2
	.long	02421DE0H,NARCS
CSHRAPB3:
	.word	5,6,2,3
	.long	02421EE0H,NARCS
CSHRAPB4:
	.word	6,4,2,2
	.long	02422060H,NARCS
CSHRAPB5:
	.word	6,4,2,2
	.long	02422160H,NARCS
CSHRAPC1:
	.word	7,8,4,4
	.long	02422260H,NARCS
CSHRAPC2:
	.word	7,6,3,3
	.long	02422460H,NARCS
CSHRAPC3:
	.word	6,9,3,4
	.long	024225E0H,NARCS
CSHRAPC4:
	.word	7,5,3,2
	.long	02422820H,NARCS
CSHRAPC5:
	.word	7,6,3,3
	.long	02422960H,NARCS
CSHRAPD1:
	.word	10,10,5,4
	.long	02422AE0H,NARCS
CSHRAPD2:
	.word	10,7,6,3
	.long	02422EA0H,NARCS
CSHRAPD3:
	.word	10,11,5,5
	.long	02423140H,NARCS
CSHRAPD4:
	.word	10,8,5,4
	.long	02423560H,NARCS
CSHRAPD5:
	.word	11,7,5,3
	.long	02423860H,NARCS
CSHRAPE1:
	.word	12,13,7,6
	.long	02423B00H,NARCS
CSHRAPE2:
	.word	14,11,7,6
	.long	02423FE0H,NARCS
CSHRAPE3:
	.word	13,16,6,7
	.long	02424560H,NARCS
CSHRAPE4:
	.word	13,11,6,6
	.long	02424D60H,NARCS
CSHRAPE5:
	.word	13,10,6,5
	.long	024252E0H,NARCS
CFRAG1:
	.word	13,22,7,-43
	.long	024257E0H,caddy
CFRAG2:
	.word	10,27,4,-41
	.long	024262E0H,caddy
CFRAG3:
	.word	14,21,7,-46
	.long	02426D00H,caddy
CFRAG4:
	.word	8,26,4,-41
	.long	02427780H,caddy
CFRAG5:
	.word	8,26,5,-41
	.long	02427E00H,caddy
CSHRAP1:
	.word	12,15,6,7
	.long	02428480H,caddy
CSHRAP2:
	.word	14,11,7,5
	.long	02428A20H,caddy
CSHRAP3:
	.word	11,17,4,7
	.long	02428FA0H,caddy
CSHRAP4:
	.word	13,9,6,3
	.long	02429600H,caddy
CSHRAP5:
	.word	13,10,4,2
	.long	02429A80H,caddy
CDOOR1:
	.word	64,51,31,24
	.long	02429F80H,caddy
CDOOR2:
	.word	63,21,31,10
	.long	02430580H,caddy
CDOOR3:
	.word	64,51,31,24
	.long	02432F80H,caddy
CDOOR4:
	.word	64,34,32,15
	.long	02439580H,caddy
CDOOR5:
	.word	66,17,31,8
	.long	0243D980H,caddy
CSHELLL:
	.word	99,64,-21,-130
	.long	0243FDA0H,caddy
CSHELLR:
	.word	114,57,124,-136
	.long	0244C5A0H,caddy

	.include NARCHYPO.GLO
HYPOB0:
	.word	29,7,16,3
	.long	02BE4580H,JRwalk
HYPOB02:
	.word	24,7,11,3
	.long	02BE4C80H,JRwalk
HYPOB03:
	.word	20,7,7,3
	.long	02BE51C0H,JRwalk
HYPOB30:
	.word	26,20,15,11
	.long	02BE5620H,JRwalk
HYPOB60:
	.word	17,29,9,16
	.long	02BE67A0H,JRwalk
HYPOB90:
	.word	8,32,4,18
	.long	02BE79C0H,JRwalk
HYPOT0:
	.word	11,1,0,0
	.long	02BE81C0H,JRwalk
HYPOT30:
	.word	9,6,0,0
	.long	02BE8220H,JRwalk
HYPOT60:
	.word	6,11,0,0
	.long	02BE8460H,JRwalk
HYPOT90:
	.word	1,11,0,0
	.long	02BE8720H,JRwalk
blood1:
	.word	10,10,5,5
	.long	02BE8880H,NARCS
blood2:
	.word	22,17,12,7
	.long	02BE8C40H,NARCS
blood3a:
	.word	35,26,18,12
	.long	02BE9900H,NARCS
nb2a:
	.word	87,66,43,38
	.long	02E6D420H,newbom
nb3a:
	.word	79,52,42,36
	.long	02E789A0H,newbom
nb4:
	.word	58,34,30,31
	.long	02E80BA0H,newbom
nb1:
	.word	53,33,28,19
	.long	02E84B60H,newbom
nb5:
	.word	47,30,28,35
	.long	02E88520H,newbom
nb6:
	.word	47,26,25,42
	.long	02E8B220H,newbom
nb7:
	.word	50,24,25,47
	.long	02E8D920H,newbom
nb8:
	.word	59,29,31,54
	.long	02E90020H,newbom
hiro9air:
	.word	40,19,21,58
	.long	02E93680H,newbom
hiro10air:
	.word	38,16,20,59
	.long	02E94E40H,newbom
hiro11air:
	.word	38,13,18,61
	.long	02E96240H,newbom
hiro1:
	.word	87,60,43,52
	.long	02E97280H,newbom
hiro2:
	.word	96,67,45,58
	.long	02EA1780H,newbom
hiro3:
	.word	60,60,31,57
	.long	02EAE080H,newbom
hiro4:
	.word	42,62,22,66
	.long	02EB5100H,newbom
hiro5:
	.word	50,61,27,70
	.long	02EBA640H,newbom
hiro6:
	.word	49,34,26,73
	.long	02EC0960H,newbom
nb1grnd:
	.word	53,33,28,26
	.long	02E84B60H,newbom
nb5grnd:
	.word	47,30,26,81
	.long	02E88520H,newbom
nb6grnd:
	.word	47,26,28,89
	.long	02E8B220H,newbom
nb7grnd:
	.word	50,24,28,93
	.long	02E8D920H,newbom
nb8grnd:
	.word	59,29,33,100
	.long	02E90020H,newbom
hiro9:
	.word	40,19,22,104
	.long	02E93680H,newbom
hiro10:
	.word	38,16,18,106
	.long	02E94E40H,newbom
hiro11:
	.word	38,13,20,113
	.long	02E96240H,newbom
dstk0:
	.word	4,17,2,8
	.long	02EC40A0H,NARCS
dstk30:
	.word	11,16,5,8
	.long	02EC42C0H,NARCS
dstk60:
	.word	15,13,7,7
	.long	02EC48C0H,NARCS
dstk90:
	.word	15,4,8,2
	.long	02EC4F40H,NARCS
spk0a:
	.word	17,28,8,26
	.long	02EC5140H,NARCS
spk30a:
	.word	19,27,2,25
	.long	02EC62C0H,NARCS
spk60a:
	.word	22,21,1,20
	.long	02EC73A0H,NARCS
spk90a:
	.word	25,16,0,8
	.long	02EC8360H,NARCS
spk0b:
	.word	14,28,7,27
	.long	02EC9160H,NARCS
spk30b:
	.word	17,27,0,25
	.long	02EC9F60H,NARCS
spk60b:
	.word	23,19,0,18
	.long	02ECB040H,NARCS
spk90b:
	.word	24,16,0,9
	.long	02ECBE80H,NARCS
spk0c:
	.word	16,29,8,28
	.long	02ECCA80H,NARCS
spk30c:
	.word	21,31,0,29
	.long	02ECD900H,NARCS
spk60c:
	.word	27,21,0,20
	.long	02ECF040H,NARCS
spk90c:
	.word	25,18,0,9
	.long	02ED02A0H,NARCS
hook4:
	.word	28,81,8,-54
	.long	02F0C840H,hookpal
hkfgt4:
	.word	36,84,4,-52
	.long	02F75420H,hookpal
hkfgt5:
	.word	29,86,6,-50
	.long	02F7B2A0H,hookpal
hkfgt6:
	.word	31,85,6,-50
	.long	02F808A0H,hookpal
hkfgt7:
	.word	27,88,12,-49
	.long	02F85DA0H,hookpal
hkfgt8:
	.word	35,88,11,-49
	.long	02F8AAA0H,hookpal
hkdie1:
	.word	31,83,14,-54
	.long	02F90DA0H,hookpal
hkdie2:
	.word	33,70,15,-67
	.long	02F960A0H,hookpal
hkdie3:
	.word	38,56,17,-81
	.long	02F9AF60H,hookpal
hkdie4:
	.word	45,48,17,-89
	.long	02F9F560H,hookpal
hkdie5:
	.word	51,40,18,-98
	.long	02FA3D60H,hookpal
hkdie6:
	.word	68,25,27,-117
	.long	02FA7E60H,hookpal
DRUM:
	.word	32,53,0,0
	.long	03107160H,tpl
SUBCOLM1:
	.word	52,32,0,0
	.long	0310A660H,subpls
SUBCOLM2:
	.word	52,30,0,0
	.long	0310DA60H,subpls
SUBCOLM3:
	.word	52,24,0,0
	.long	03110B20H,subpls
SUBCOLM4:
	.word	36,22,0,0
	.long	03113220H,subpls
SUBCOLM5:
	.word	36,21,0,0
	.long	03114AE0H,subpls
SUBCOLM6:
	.word	36,36,0,0
	.long	03116280H,subpls
TELEPHON:
	.word	28,93,0,0
	.long	03118B00H,phns
NEWCAN:
	.word	28,56,0,0
	.long	0311DC60H,can
BENCHL:
	.word	40,45,0,0
	.long	03120D60H,itdrfr
BENCHM:
	.word	44,35,0,0
	.long	031245A0H,itdrfr
BENCHR:
	.word	32,45,0,0
	.long	031275C0H,itdrfr
ADDISON:
	.word	32,55,0,0
	.long	0312A2C0H,sgns
TRNDR1:
	.word	40,100,39,-1
	.long	0312D9C0H,TRNPAL
BULLET:
	.word	19,2,0,0
	.long	031356C0H,NARCS
BULLETE:
	.word	19,1,0,0
	.long	03135800H,NARCS
BULLET45:
	.word	8,8,4,3
	.long	031358A0H,NARCS
SFLAME:
	.word	16,9,0,4
	.long	03135AA0H,NARCS
jtdrive1:
	.word	27,30,13,-121
	.long	03144980H,redman
jtdrive2:
	.word	26,30,13,-121
	.long	031463C0H,bluenc
jtdrive3:
	.word	22,31,12,-120
	.long	03147E00H,bluenc
jtdrive4:
	.word	33,37,19,-112
	.long	03149540H,bluenc
jtdrive5:
	.word	36,13,19,-136
	.long	0314BEE0H,bluenc
jtpassen1:
	.word	21,32,20,-124
	.long	0314CD80H,redman
jtpassen2:
	.word	26,30,20,-124
	.long	031463C0H,bluenc
jtpassen3:
	.word	22,31,20,-123
	.long	03147E00H,bluenc
jtpassen4:
	.word	33,37,24,-113
	.long	03149540H,bluenc
jtpassen5:
	.word	36,13,24,-137
	.long	0314BEE0H,bluenc
ytdrive1:
	.word	27,30,13,-121
	.long	03144980H,bluenc
ytpassen1:
	.word	21,32,20,-124
	.long	0314CD80H,bluenc
jtsstdrst:
	.word	51,40,8,-55
	.long	0314E580H,redman
jlsw4:
	.word	31,51,16,-87
	.long	03152680H,redman
ytsstdrst:
	.word	51,40,8,-55
	.long	0314E580H,bluenc
ylsw4:
	.word	31,51,16,-87
	.long	03152680H,bluenc
P1LIFE:
	.word	12,15,0,0
	.long	03155980H,redman
P2LIFE:
	.word	12,15,0,0
	.long	03155980H,bluenc
jrpilot:
	.word	29,39,-34,-133
	.long	03155F20H,bluenc
jrhelpas:
	.word	29,39,-28,-135
	.long	03155F20H,redman
jrpilohit:
	.word	33,37,-22,-130
	.long	03149540H,bluenc
jrhelphit:
	.word	33,37,-13,-133
	.long	03149540H,bluenc
flail1:
	.word	54,91,26,-47
	.long	031A82C0H,bluenc
flail2:
	.word	59,80,25,-56
	.long	031B2200H,bluenc
flail3:
	.word	48,101,17,-38
	.long	031BB800H,bluenc
flail4:
	.word	58,87,27,-51
	.long	031C4F80H,bluenc
mbigblp1a:
	.word	7,8,0,0
	.long	037256A0H,NARCS

	.include BGNDTBL.GLO
