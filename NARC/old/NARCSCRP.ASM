**************************************************************************
*                                                                        *
*          NARC (R)                                                      *
*                                                                        *
*          Copyright 1988 Williams Electronics Games Inc.                *
*          All Rights Reserved                                           *
*                                                                        *
**************************************************************************
*
*        CODES FOR SCRIPT DATA
*
END_OF_SCRIPT    EQU     0
COLOR_FOR_BLACK  EQU     1
COLOR_FOR_WALL   EQU     2
SPRAY_SCORE_A0   EQU     3
CALL_SCRIPT      EQU     4
PROCESS_WORD     EQU     5
PROCESS_LONG     EQU     6
SPRAY_CHAR       EQU     7        ;(ASCII CHARACTER FOLLOWS). 
SPRAY_CHAR_A0    EQU     8        ;ASCII CHARACTER IN A0
SPRAY_FORK       EQU     9        ;FORK TO SCRIPT THAT FOLLOWS
SPRAY_SLEEP      EQU     10
SPRAY_ENTRY      EQU     11
BLOB_ON_BLACK    EQU     12
BLOB_ON_WALL     EQU     13
GLOBAL_PALETTE   EQU     14
DELTA_Y          EQU     15
LAST_OPERATOR    EQU     15       ;NO OPERATOR GREATER THAN THIS
*
*        THESE COLOR MACROS MAY ONLY BE USED IN SCRIPTS
*        ONCE THE PROCESS PALETTE HAS BEEN SET.
*
S_DY     $MACRO DY
         .BYTE   DELTA_Y
         .LONG   :DY:
         $ENDM

S_COLOR  $MACRO COLOR     ;SET FOR PAINTING ON BLACK
         .BYTE   COLOR_FOR_BLACK
         .BYTE   :COLOR:
         $ENDM

S_WALL   $MACRO COLOR_TABLE
         .BYTE   COLOR_FOR_WALL
         .LONG   :COLOR_TABLE:
         $ENDM

S_CALL   $MACRO ADDRESS
         .BYTE   CALL_SCRIPT
         .LONG   :ADDRESS:
         $ENDM

S_CHAR   $MACRO  CHARACTER
         .BYTE   SPRAY_CHAR
         .BYTE   :CHARACTER:
         $ENDM

S_END    $MACRO
         .BYTE   END_OF_SCRIPT
         $ENDM

S_X      $MACRO  X
         .BYTE   PROCESS_LONG
         .WORD   SPR_X
         .LONG   STRUCT_X_UNIT*:X:
         $ENDM

S_Y      $MACRO  Y
         .BYTE   PROCESS_LONG
         .WORD   SPR_Y
         .LONG   STRUCT_Y_UNIT*:Y:
         $ENDM

S_RATE   $MACRO  TIME
         .BYTE   PROCESS_WORD
         .WORD   SPR_SLEEP
         .WORD   :TIME:
         $ENDM

S_SCALER $MACRO    SCALE
         .BYTE   PROCESS_WORD
         .WORD   SPR_SCALER
         .WORD   :SCALE:
         $ENDM

S_BRUSH  $MACRO   BRUSH_POINTER
         .BYTE   PROCESS_LONG
         .WORD   SPR_POINTER
         .LONG   :BRUSH_POINTER:
         $ENDM

S_PAL    $MACRO   PALETTE
         .BYTE   PROCESS_WORD
         .WORD   SPR_PALETTE
         .WORD   :PALETTE:
         $ENDM

S_FORK   $MACRO  SCRIPT
         .BYTE   SPRAY_FORK
         .LONG   :SCRIPT:
         $ENDM

S_SLEEP  $MACRO  TIME
         .BYTE   SPRAY_SLEEP
         .BYTE   :TIME:
         $ENDM

S_ENTRY  $MACRO
         .BYTE   SPRAY_ENTRY
         $ENDM

S_NEW_C  $MACRO  COLOR
         .BYTE   PROCESS_WORD
         .WORD   SPR_COL_BASE
         .WORD   :COLOR:*32
         .BYTE   PROCESS_WORD
         .WORD   SPR_COL_MAX
         .WORD   (:COLOR:*32)+31
         $ENDM

S_ON_WAL $MACRO  BLOB_R
         .BYTE   BLOB_ON_WALL
         $ENDM

S_ON_BLK $MACRO  BLOB_R
         .BYTE   BLOB_ON_BLACK
         $ENDM

S_GPAL   $MACRO  
         .BYTE   GLOBAL_PALETTE
         $ENDM


