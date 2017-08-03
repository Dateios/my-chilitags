

'******** METEL_FIGHTER STAND PROGRAM ********

DIM I AS BYTE
DIM J AS BYTE
DIM pose AS BYTE
DIM Action_mode AS BYTE
DIM A AS BYTE
DIM A_old AS BYTE
DIM B AS BYTE
DIM C AS BYTE
DIM walk_speed AS BYTE
DIM L_R_speed AS BYTE
DIM L_R_speed2 AS BYTE
DIM walk_order AS BYTE
DIM narrow_walk_times AS BYTE
DIM now_volt AS BYTE
DIM mirror_check AS BYTE
DIM motorONOFF AS BYTE
DIM zyroONOFF AS BYTE
DIM tilt_F_B AS INTEGER
DIM tilt_L_R AS INTEGER
DIM DELAY_TIME AS BYTE
DIM DELAY_TIME2 AS BYTE
DIM TEMP AS BYTE
DIM grip_pose AS BYTE
DIM tilt_CNT AS BYTE

DIM S6 AS BYTE
DIM S7 AS BYTE
DIM S8 AS BYTE

DIM S12 AS BYTE
DIM S13 AS BYTE
DIM S14 AS BYTE

'****Action_mode ******************
CONST ID = 0     ' 1:0, 2:32, 3:64, 4:96,

'**** TILT SENSOR CHECK

CONST F_B_tilt_AD_port = 2
CONST L_R_tilt_AD_port = 3
CONST tilt_check_CNT = 10  'ms


CONST min = 60	'95		
CONST max = 100	'160		
CONST COUNT_MAX = 30
CONST low_volt = 106	

PTP SETON 				
PTP ALLON				

DIR G6A,1,0,0,1,0,0		'motor0~5
DIR G6B,1,1,1,1,1,1		'motor6~11
DIR G6C,0,0,0,0,0,0		'motor12~17
DIR G6D,0,1,1,0,1,0		'motor18~23

'***** First Program *********************************
motorONOFF = 0
walk_order = 0
mirror_check = 0
tilt_CNT = 0		
grip_pose = 0

'**** Feedback *****************************

GOSUB MOTOR_ON

SPEED 5
GOSUB power_first_pose
GOSUB stand_pose


GOTO MAIN
'************************************************
'************************************************
'************************************************

start_music:
    TEMPO 220
    MUSIC "O23EAB7EA>3#C"
    RETURN
    '************************************************
end_music:
    TEMPO 220
    MUSIC "O38GD<BGD<BG"
    RETURN
    '************************************************
entertainment_music:
    TEMPO 220
    MUSIC "O28B>4D8C4E<8B>4D<8B>4G<8E>4C"
    RETURN
    '************************************************
game_music:
    TEMPO 210
    MUSIC "O23C5G3#F5G3A5G"
    RETURN
    '************************************************
fight_music:
    TEMPO 210
    MUSIC "O15A>C<A>3D"
    RETURN
    '************************************************
warning_music:
    TEMPO 180
    MUSIC "O13A"
    DELAY 300

    RETURN
    '************************************************
siren_music:
    TEMPO 180
    MUSIC "O22FC"
    DELAY 10
    RETURN
    '************************************************
mode1:
    TEMPO 200
    MUSIC "O18A>#CE#G#F#D#FB"
    RETURN
footballgame_music:
    TEMPO 180
    MUSIC "O28A#GABA"
    RETURN


mode3:
    TEMPO 190
    MUSIC "O28#C#D#F#G#A#G#F"
    RETURN

mode4:
    TEMPO 220
    MUSIC "O33C6D3C<6$B3A"
    RETURN


hurdle_game_music:
    TEMPO 200
    MUSIC "O37C<C#BCA"
    RETURN

    '************************************************
MOTOR_FAST_ON:

    MOTOR G6B
    MOTOR G6C
    MOTOR G6A
    MOTOR G6D

    motorONOFF = 0

    RETURN

    '************************************************
    '************************************************
MOTOR_ON:

    GOSUB MOTOR_GET

    MOTOR G6B
    DELAY 50
    MOTOR G6C
    DELAY 50
    MOTOR G6A
    DELAY 50
    MOTOR G6D

    motorONOFF = 0
    GOSUB start_music			
    RETURN

    '************************************************
MOTOR_OFF:

    MOTOROFF G6B
    MOTOROFF G6C
    MOTOROFF G6A
    MOTOROFF G6D
    motorONOFF = 1	
    GOSUB MOTOR_GET	
    GOSUB end_music	
    RETURN
    '************************************************
MOTOR_GET:
    GETMOTORSET G6A,1,1,1,1,1,0
    GETMOTORSET G6B,1,1,1,0,0,0
    GETMOTORSET G6C,1,1,1,0,0,0
    GETMOTORSET G6D,1,1,1,1,1,0
    RETURN

    '************************************************
All_motor_Reset:

    MOTORMODE G6A,1,1,1,1,1
    MOTORMODE G6D,1,1,1,1,1
    MOTORMODE G6B,1,1,1
    MOTORMODE G6C,1,1,1

    RETURN
    '************************************************
All_motor_mode2:

    MOTORMODE G6A,2,2,2,2,2
    MOTORMODE G6D,2,2,2,2,2
    MOTORMODE G6B,2,2,2
    MOTORMODE G6C,2,2,2

    RETURN
    '************************************************
All_motor_mode3:

    MOTORMODE G6A,3,3,3,3,3
    MOTORMODE G6D,3,3,3,3,3
    MOTORMODE G6B,3,3,3
    MOTORMODE G6C,3,3,3

    RETURN
    '************************************************
Leg_motor_mode1:
    MOTORMODE G6A,1,1,1,1,1
    MOTORMODE G6D,1,1,1,1,1
    RETURN
    '************************************************
Leg_motor_mode2:
    MOTORMODE G6A,2,2,2,2,2
    MOTORMODE G6D,2,2,2,2,2
    RETURN

    '************************************************
Leg_motor_mode3:
    MOTORMODE G6A,3,3,3,3,3
    MOTORMODE G6D,3,3,3,3,3
    RETURN
    '************************************************
Leg_motor_mode4:
    MOTORMODE G6A,3,2,2,1,3
    MOTORMODE G6D,3,2,2,1,3
    RETURN
    '************************************************
Leg_motor_mode5:
    MOTORMODE G6A,3,2,2,1,2
    MOTORMODE G6D,3,2,2,1,2
    RETURN
    '************************************************
    '************************************************
Arm_motor_mode1:
    MOTORMODE G6B,1,1,1
    MOTORMODE G6C,1,1,1
    RETURN
    '************************************************
Arm_motor_mode2:
    MOTORMODE G6B,2,2,2
    MOTORMODE G6C,2,2,2
    RETURN
    '************************************************
Arm_motor_mode3:
    MOTORMODE G6B,3,3,3
    MOTORMODE G6C,3,3,3
    RETURN
    '************************************************
power_first_pose:
    MOVE G6A,95,  76, 145,  93, 105, 100
    MOVE G6D,95,  76, 145,  93, 105, 100
    MOVE G6B,100,  45,  90, 100, 100, 100
    MOVE G6C,100,  45,  90, 100, 100, 100
    WAIT
    pose = 0
    RETURN
    '************************************************
stabilized_pose:
    MOVE G6A,98,  76, 145,  93, 101, 100
    MOVE G6D,98,  76, 145,  93, 101, 100
    MOVE G6B,100,  30,  80, 100, 100, 100
    MOVE G6C,100,  30,  80, 100, 100, 100
    WAIT
    pose = 0

    RETURN
    '******************************************	
    '************************************************
stand_pose:
    MOVE G6A,100,  76, 145,  93, 100, 100
    MOVE G6D,100,  76, 145,  93, 100, 100
    MOVE G6B,100,  30,  80, 100, 100, 100
    MOVE G6C,100,  30,  80, 100, 100, 100
    WAIT
    pose = 0
    grip_pose = 0
    RETURN
    '******************************************	
stand_up_pose:
    MOVE G6A,100, 56, 182, 76, 100, 100
    MOVE G6D,100, 56, 182, 76, 100, 100
    MOVE G6B,100, 20, 90, 100, 100, 100
    MOVE G6C,100, 20, 90, 100, 100, 100
    WAIT
    pose = 2
    RETURN
    '******************************************
sit_pose:

    MOVE G6A,100, 143,  28, 142, 100, 100
    MOVE G6D,100, 143,  28, 142, 100, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT
    pose = 1

    RETURN

    '**********************************************
RX_EXIT:

    ERX 9600, A, MAIN

    GOTO RX_EXIT
    '**********************************************
GOSUB_RX_EXIT:

    ERX 9600, A, GOSUB_RX_EXIT2

    GOTO GOSUB_RX_EXIT

GOSUB_RX_EXIT2:
    RETURN
    '**********************************************

front_walk_50:

    walk_speed = 13'5
    L_R_speed = 6'8'3
    L_R_speed2 = 4'5'2

    GOSUB Leg_motor_mode3

    IF walk_order = 0 THEN
        walk_order = 1

        SPEED 4
        MOVE G6A, 88,  71, 152,  91, 110
        MOVE G6D,108,  76, 146,  93,  94
        MOVE G6B,100,35
        MOVE G6C,100,35
        WAIT

        SPEED 10'walk_speed
        MOVE G6A, 90, 100, 115, 105, 114
        MOVE G6D,113,  78, 146,  93,  94
        MOVE G6B,90
        MOVE G6C,110
        WAIT


        GOTO front_walk_50_1	
    ELSE
        walk_order = 0

        SPEED 4
        MOVE G6D,  88,  71, 152,  91, 110
        MOVE G6A, 108,  76, 146,  93,  94
        MOVE G6C, 100,35
        MOVE G6B, 100,35
        WAIT

        SPEED 10'walk_speed
        MOVE G6D, 90, 100, 115, 105, 114
        MOVE G6A,113,  78, 146,  93,  94
        MOVE G6C,90
        MOVE G6B,110
        WAIT


        GOTO front_walk_50_2	

    ENDIF


    '*******************************


front_walk_50_1:

    SPEED walk_speed
    MOVE G6A, 90,  44, 163, 113, 114
    MOVE G6D,110,  77, 146,  93,  94
    WAIT

    SPEED L_R_speed
    GOSUB Leg_motor_mode3
    MOVE G6A,110,  76, 144, 100,  93
    MOVE G6D,90, 93, 155,  71, 112
    WAIT


    SPEED walk_speed
    GOSUB Leg_motor_mode2
    MOVE G6A,111,  77, 146,  93, 94
    MOVE G6D,90, 100, 105, 110, 114
    MOVE G6B,110
    MOVE G6C,90
    WAIT

    ERX 9600,A, front_walk_50_2
    IF A <> A_old THEN
        HIGHSPEED SETOFF
        GOSUB Leg_motor_mode3
        SPEED 5

        MOVE G6A, 106,  76, 146,  93,  96		
        MOVE G6D,  88,  71, 152,  91, 106
        MOVE G6B, 100,35
        MOVE G6C, 100,35
        WAIT	


        SPEED 3
        GOSUB stand_pose
        GOSUB Leg_motor_mode1
        GOTO RX_EXIT
    ENDIF
    '**********

front_walk_50_2:

    SPEED walk_speed
    MOVE G6D,90,  44, 163, 113, 114
    MOVE G6A,110,  77, 146,  93,  94
    WAIT

    SPEED L_R_speed
    GOSUB Leg_motor_mode3
    MOVE G6D,110,  76, 144, 100,  93
    MOVE G6A, 90, 93, 155,  71, 112
    WAIT

    SPEED walk_speed
    GOSUB Leg_motor_mode2
    MOVE G6A, 90, 100, 105, 110, 114
    MOVE G6D,111,  77, 146,  93,  94
    MOVE G6B, 90
    MOVE G6C,110
    WAIT

    ERX 9600,A, front_walk_50_1
    IF A <> A_old THEN
        HIGHSPEED SETOFF
        GOSUB Leg_motor_mode3
        SPEED 5

        MOVE G6D, 106,  76, 146,  93,  96		
        MOVE G6A,  88,  71, 152,  91, 106
        MOVE G6C, 100,35
        MOVE G6B, 100,35
        WAIT

        SPEED 3
        GOSUB stand_pose
        GOSUB Leg_motor_mode1
        GOTO RX_EXIT
    ENDIF


    GOTO front_walk_50_1
    '************************************************
    '************************************************
    '************************************************
back_walk_50:

    walk_speed = 14'6
    L_R_speed = 6'3
    L_R_speed2 = 4'2
    GOSUB Leg_motor_mode3



    IF walk_order = 0 THEN
        walk_order = 1

        SPEED 4
        MOVE G6A, 88,  71, 152,  91, 110
        MOVE G6D,108,  76, 146,  93,  94
        MOVE G6B,100,35
        MOVE G6C,100,35
        WAIT

        SPEED 10'walk_speed
        MOVE G6A, 90, 95, 115, 105, 114
        MOVE G6D,113,  78, 146,  93,  94
        MOVE G6B,90
        MOVE G6C,110
        WAIT

        GOTO back_walk_50_1	
    ELSE
        walk_order = 0

        SPEED 4
        MOVE G6D,  88,  71, 152,  91, 110
        MOVE G6A, 108,  76, 146,  93,  94
        MOVE G6C, 100,35
        MOVE G6B, 100,35
        WAIT

        'HIGHSPEED SETON
        SPEED 10'walk_speed
        MOVE G6D, 90, 95, 115, 105, 114
        MOVE G6A,113,  78, 146,  93,  94
        MOVE G6C,90
        MOVE G6B,110
        WAIT

        GOTO back_walk_50_2

    ENDIF

back_walk_50_1:
    SPEED walk_speed
    GOSUB Leg_motor_mode2
    MOVE G6D,110,  76, 144, 100,  93
    MOVE G6A, 90, 93, 155,  71, 112
    WAIT

    SPEED L_R_speed2
    GOSUB Leg_motor_mode3
    MOVE G6D,90,  46, 163, 113, 114
    MOVE G6A,110,  77, 147,  93,  94
    WAIT

    SPEED walk_speed
    MOVE G6A,112,  77, 147,  93, 94
    MOVE G6D,90, 100, 105, 110, 114
    MOVE G6B,110
    MOVE G6C,90
    WAIT

    ERX 9600,A, back_walk_50_2
    IF A <> A_old THEN
        HIGHSPEED SETOFF
        GOSUB Leg_motor_mode3
        SPEED 5

        MOVE G6A, 106,  77, 148,  93,  96		
        MOVE G6D,  88,  72, 154,  91, 106
        MOVE G6B, 100,35
        MOVE G6C, 100,35
        WAIT	

        SPEED 3
        GOSUB stand_pose
        GOSUB Leg_motor_mode1
        ' GOSUB zyroOFF
        GOTO RX_EXIT
    ENDIF
    '**********

back_walk_50_2:
    SPEED walk_speed
    GOSUB Leg_motor_mode2
    MOVE G6A,110,  76, 144, 100,  93
    MOVE G6D,90, 93, 155,  71, 112
    WAIT

    SPEED L_R_speed2
    GOSUB Leg_motor_mode3
    MOVE G6A, 90,  46, 163, 113, 114
    MOVE G6D,110,  77, 147,  93,  94
    WAIT

    SPEED walk_speed
    MOVE G6A, 90, 100, 105, 110, 114
    MOVE G6D,112,  76, 147,  93,  94
    MOVE G6B, 90
    MOVE G6C,110
    WAIT

    ERX 9600,A, back_walk_50_1
    IF A <> A_old THEN
        HIGHSPEED SETOFF
        GOSUB Leg_motor_mode3
        SPEED 5

        MOVE G6D, 106,  77, 148,  93,  96		
        MOVE G6A,  88,  72, 154,  91, 106
        MOVE G6B, 100,35
        MOVE G6C, 100,35
        WAIT	

        SPEED 3
        GOSUB stand_pose
        GOSUB Leg_motor_mode1
        GOTO RX_EXIT
    ENDIF  	

    GOTO back_walk_50_1
    '**********************************************
    '**********************************************
    '**********************************************

stay_walk:
    GOSUB Arm_motor_mode3
    GOSUB Leg_motor_mode2

    FOR i = 0 TO 4

        SPEED 4
        MOVE G6A,105,  76, 146,  93, 98, 100
        MOVE G6D,85,  73, 151,  90, 108, 100
        WAIT

        SPEED 12
        MOVE G6A,113,  76, 146,  93, 95, 100
        MOVE G6D,90,  100, 95,  120, 110, 100
        MOVE G6B,120
        MOVE G6C,80
        WAIT

        SPEED 10
        MOVE G6A,109,  76, 146,  93, 97, 100
        MOVE G6D,90,  76, 148,  92, 107, 100
        WAIT

        SPEED 4	
        MOVE G6A,98,  76, 146,  93, 100, 100
        MOVE G6D,98,  76, 146,  93, 100, 100	
        WAIT

        '***********************************
        SPEED 4
        MOVE G6D,105,  76, 146,  93, 98, 100
        MOVE G6A,85,  73, 151,  90, 108, 100
        WAIT	

        SPEED 12
        MOVE G6D,113,  76, 146,  93, 95, 100
        MOVE G6A,90,  100, 95,  120, 110, 100
        MOVE G6C,120
        MOVE G6B,80
        WAIT	

        SPEED 10
        MOVE G6D,109,  76, 146,  93, 97, 100
        MOVE G6A,90,  76, 148,  92, 107, 100
        WAIT	

        SPEED 4		
        MOVE G6D,98,  76, 146,  93, 100, 100
        MOVE G6A,98,  76, 146,  93, 100, 100	
        WAIT	


    NEXT i

    SPEED 5
    GOSUB stand_pose
    GOSUB Leg_motor_mode1
    RETURN
    '**********************************************	
    '**********************************************	
    '************************************************
right_walk20:


    SPEED 12
    MOVE G6D, 93,  90, 120, 105, 104, 100
    MOVE G6A,103,  76, 145,  93, 104, 100
    WAIT

    SPEED 12
    MOVE G6D, 102,  77, 145, 93, 100, 100
    MOVE G6A,90,  80, 140,  95, 107, 100
    WAIT

    SPEED 15
    MOVE G6D,98,  76, 145,  93, 100, 100
    MOVE G6A,98,  76, 145,  93, 100, 100
    WAIT

    SPEED 8

    GOSUB stand_pose

    GOTO RX_EXIT
    '**********************************************

left_walk20:


    SPEED 12
    MOVE G6A, 93,  90, 120, 105, 104, 100
    MOVE G6D,103,  76, 145,  93, 104, 100
    WAIT

    SPEED 12
    MOVE G6A, 102,  77, 145, 93, 100, 100
    MOVE G6D,90,  80, 140,  95, 107, 100
    WAIT

    SPEED 15
    MOVE G6A,98,  76, 145,  93, 100, 100
    MOVE G6D,98,  76, 145,  93, 100, 100
    WAIT

    SPEED 8

    GOSUB stand_pose
    GOTO RX_EXIT

    '**********************************************

right_walk70:

    SPEED 10
    MOVE G6D, 90,  90, 120, 105, 110, 100
    MOVE G6A,100,  76, 146,  93, 107, 100
    MOVE G6B,100,  40
    MOVE G6C,100,  40
    WAIT

    SPEED 12
    MOVE G6D, 102,  76, 147, 93, 100, 100
    MOVE G6A,83,  78, 140,  96, 115, 100
    WAIT

    SPEED 10
    MOVE G6D,98,  76, 146,  93, 100, 100
    MOVE G6A,98,  76, 146,  93, 100, 100
    WAIT

    SPEED 15
    GOSUB stand_pose

    GOTO RX_EXIT
    '*************

left_walk70:


    SPEED 10
    MOVE G6A, 90,  90, 120, 105, 110, 100	
    MOVE G6D,100,  76, 146,  93, 107, 100	
    MOVE G6B,100,  40
    MOVE G6C,100,  40
    WAIT

    SPEED 12
    MOVE G6A, 102,  76, 147, 93, 100, 100
    MOVE G6D,83,  78, 140,  96, 115, 100
    WAIT

    SPEED 10
    MOVE G6A,98,  76, 146,  93, 100, 100
    MOVE G6D,98,  76, 146,  93, 100, 100
    WAIT

    SPEED 15	
    GOSUB stand_pose

    GOTO RX_EXIT
    '************************************************

catch_right_walk20:


    SPEED 12
    MOVE G6D, 93,  90, 120, 97, 104, 100
    MOVE G6A,103,  76, 145,  85, 104, 100
    WAIT

    SPEED 12
    MOVE G6D, 102,  77, 145, 85, 100, 100
    MOVE G6A,90,  80, 140,  87, 107, 100
    WAIT

    SPEED 15
    MOVE G6D,98,  76, 145,  85, 100, 100
    MOVE G6A,98,  76, 145,  85, 100, 100
    WAIT

    SPEED 8
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT

    GOTO RX_EXIT
    '**********************************************

catch_left_walk20:


    SPEED 12
    MOVE G6A, 93,  90, 120, 97, 104, 100
    MOVE G6D,103,  76, 145,  85, 104, 100
    WAIT

    SPEED 12
    MOVE G6A, 102,  77, 145, 85, 100, 100
    MOVE G6D,90,  80, 140,  87, 107, 100
    WAIT

    SPEED 15
    MOVE G6A,98,  76, 145,  85, 100, 100
    MOVE G6D,98,  76, 145,  85, 100, 100
    WAIT

    SPEED 8
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT
    GOTO RX_EXIT

    '**********************************************

catch_right_walk70:

    SPEED 10
    MOVE G6D, 90,  90, 120, 92, 110, 100
    MOVE G6A,100,  76, 146,  85, 107, 100
    WAIT

    SPEED 12
    MOVE G6D, 102,  76, 147, 85, 100, 100
    MOVE G6A,83,  76, 140,  92, 115, 100
    WAIT

    SPEED 10
    MOVE G6D,98,  76, 146,  85, 100, 100
    MOVE G6A,98,  76, 146,  85, 100, 100
    WAIT

    SPEED 15
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT

    GOTO RX_EXIT
    '*************

catch_left_walk70:


    SPEED 10
    MOVE G6A, 90,  90, 120, 97, 110, 100	
    MOVE G6D,100,  76, 146,  85, 107, 100	
    WAIT

    SPEED 12
    MOVE G6A, 102,  76, 147, 85, 100, 100
    MOVE G6D,83,  76, 140,  92, 115, 100
    WAIT

    SPEED 10
    MOVE G6A,98,  76, 146,  85, 100, 100
    MOVE G6D,98,  76, 146,  85, 100, 100
    WAIT

    SPEED 15	
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT

    GOTO RX_EXIT
    '************************************************
slow_left_walk50:

    GOSUB Leg_motor_mode3
    GOSUB Arm_motor_mode3

    SPEED 3
    MOVE G6A, 88,  71, 152,  91, 110, '60
    MOVE G6D,108,  76, 146,  93,  92, '60
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT

    SPEED 5
    MOVE G6A, 85,  80, 140, 95, 114, 100
    MOVE G6D,112,  76, 146,  93, 98, 100
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT


    SPEED 5
    MOVE G6D,110,  92, 124,  97,  93,  100
    MOVE G6A, 76,  72, 160,  82, 128,  100
    MOVE G6B,100,  40,  80, , , ,
    MOVE G6C,100,  40,  80, , , ,
    WAIT

    SPEED 5
    MOVE G6A,94,  76, 145,  93, 106, 100
    MOVE G6D,94,  76, 145,  93, 106, 100
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT	
    '***********************
    SPEED 5
    MOVE G6A,110,  92, 124,  97,  93,  100
    MOVE G6D, 76,  72, 160,  82, 120,  100
    MOVE G6B,100,  40,  80, , , ,
    MOVE G6C,100,  40,  80, , , ,
    WAIT

    SPEED 6
    MOVE G6D, 85,  80, 140, 95, 110, 100
    MOVE G6A,112,  76, 146,  93, 98, 100
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT

    SPEED 4
    MOVE G6D, 88,  71, 152,  91, 110, '60
    MOVE G6A,108,  76, 146,  93,  92, '60
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT

    SPEED 2
    GOSUB stand_pose
    GOSUB All_motor_Reset	
    RETURN

    '**********************************************
    '************************************************
slow_right_walk50:

    GOSUB Leg_motor_mode3
    GOSUB Arm_motor_mode3

    SPEED 3
    MOVE G6D, 88,  71, 152,  91, 110, '60
    MOVE G6A,108,  76, 146,  93,  92, '60
    MOVE G6C,100,  40,  80
    MOVE G6B,100,  40,  80
    WAIT

    SPEED 5
    MOVE G6D, 85,  80, 140, 95, 114, 100
    MOVE G6A,112,  76, 146,  93, 98, 100
    MOVE G6C,100,  40,  80
    MOVE G6B,100,  40,  80
    WAIT


    SPEED 5
    MOVE G6A,110,  92, 124,  97,  93,  100
    MOVE G6D, 76,  72, 160,  82, 128,  100
    MOVE G6C,100,  40,  80, , , ,
    MOVE G6B,100,  40,  80, , , ,
    WAIT

    SPEED 5
    MOVE G6D,94,  76, 145,  93, 106, 100
    MOVE G6A,94,  76, 145,  93, 106, 100
    MOVE G6C,100,  40,  80
    MOVE G6B,100,  40,  80
    WAIT	
    '***********************
    SPEED 5
    MOVE G6D,110,  92, 124,  97,  93,  100
    MOVE G6A, 76,  72, 160,  82, 120,  100
    MOVE G6C,100,  40,  80, , , ,
    MOVE G6B,100,  40,  80, , , ,
    WAIT

    SPEED 6
    MOVE G6A, 85,  80, 140, 95, 110, 100
    MOVE G6D,112,  76, 146,  93, 98, 100
    MOVE G6C,100,  40,  80
    MOVE G6B,100,  40,  80
    WAIT

    SPEED 4
    MOVE G6A, 88,  71, 152,  91, 110, '60
    MOVE G6D,108,  76, 146,  93,  92, '60
    MOVE G6C,100,  40,  80
    MOVE G6B,100,  40,  80
    WAIT

    SPEED 2
    GOSUB stand_pose
    GOSUB All_motor_Reset	
    RETURN

    '**********************************************

    '**********************************************
left_turn_10:

    SPEED 5
    MOVE G6A,97,  86, 145,  83, 103, 100
    MOVE G6D,97,  66, 145,  103, 103, 100
    WAIT

    SPEED 12
    MOVE G6A,94,  86, 145,  83, 101, 100
    MOVE G6D,94,  66, 145,  103, 101, 100
    WAIT

    SPEED 6
    MOVE G6A,101,  76, 146,  93, 98, 100
    MOVE G6D,101,  76, 146,  93, 98, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB stand_pose
    GOTO RX_EXIT
    '**********************************************
right_turn_10:

    SPEED 5
    MOVE G6A,97,  66, 145,  103, 103, 100
    MOVE G6D,97,  86, 145,  83, 103, 100
    WAIT

    SPEED 12
    MOVE G6A,94,  66, 145,  103, 101, 100
    MOVE G6D,94,  86, 145,  83, 101, 100
    WAIT
    SPEED 6
    MOVE G6A,101,  76, 146,  93, 98, 100
    MOVE G6D,101,  76, 146,  93, 98, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB stand_pose

    GOTO RX_EXIT
    '**********************************************
    '**********************************************
left_turn_20:
    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A,95,  96, 145,  73, 105, 100
    MOVE G6D,95,  56, 145,  113, 105, 100
    MOVE G6B,110
    MOVE G6C,90
    WAIT

    SPEED 12
    MOVE G6A,93,  96, 145,  73, 105, 100
    MOVE G6D,93,  56, 145,  113, 105, 100
    WAIT
    SPEED 6
    MOVE G6A,101,  76, 146,  93, 98, 100
    MOVE G6D,101,  76, 146,  93, 98, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB stand_pose
    GOSUB Leg_motor_mode1
    GOTO RX_EXIT
    '**********************************************
right_turn_20:
    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A,95,  56, 145,  113, 105, 100
    MOVE G6D,95,  96, 145,  73, 105, 100
    MOVE G6B,90
    MOVE G6C,110
    WAIT

    SPEED 12
    MOVE G6A,93,  56, 145,  113, 105, 100
    MOVE G6D,93,  96, 145,  73, 105, 100
    WAIT

    SPEED 6
    MOVE G6A,101,  76, 146,  93, 98, 100
    MOVE G6D,101,  76, 146,  93, 98, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB stand_pose
    GOSUB Leg_motor_mode1
    GOTO RX_EXIT
    '**********************************************
left_turn_45:

    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A,95,  106, 145,  63, 105, 100
    MOVE G6D,95,  46, 145,  123, 105, 100
    MOVE G6B,115
    MOVE G6C,85
    WAIT

    SPEED 10
    MOVE G6A,93,  106, 145,  63, 105, 100
    MOVE G6D,93,  46, 145,  123, 105, 100
    WAIT

    SPEED 8
    GOSUB stand_pose
    GOSUB Leg_motor_mode1
    GOTO RX_EXIT

    '**********************************************
right_turn_45:

    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A,95,  46, 145,  123, 105, 100
    MOVE G6D,95,  106, 145,  63, 105, 100
    MOVE G6C,115
    MOVE G6B,85
    WAIT

    SPEED 10
    MOVE G6A,93,  46, 145,  123, 105, 100
    MOVE G6D,93,  106, 145,  63, 105, 100
    WAIT

    SPEED 8
    GOSUB stand_pose
    GOSUB Leg_motor_mode1
    GOTO RX_EXIT
    '**********************************************
left_turn_60:

    SPEED 15
    MOVE G6A,95,  116, 145,  53, 105, 100
    MOVE G6D,95,  36, 145,  133, 105, 100
    WAIT

    SPEED 15
    MOVE G6A,90,  116, 145,  53, 105, 100
    MOVE G6D,90,  36, 145,  133, 105, 100
    WAIT

    SPEED 10
    GOSUB stand_pose
    GOTO RX_EXIT

    '**********************************************
right_turn_60:

    SPEED 15
    MOVE G6A,95,  36, 145,  133, 105, 100
    MOVE G6D,95,  116, 145,  53, 105, 100

    WAIT

    SPEED 15
    MOVE G6A,90,  36, 145,  133, 105, 100
    MOVE G6D,90,  116, 145,  53, 105, 100

    WAIT

    SPEED 10
    GOSUB stand_pose
    GOTO RX_EXIT

    '************************************************
    '**********************************************
catch_left_turn_10:

    SPEED 5
    MOVE G6A,97,  86, 145,  75, 103, 100
    MOVE G6D,97,  66, 145,  95, 103, 100
    WAIT

    SPEED 12
    MOVE G6A,94,  86, 145,  75, 101, 100
    MOVE G6D,94,  66, 145,  95, 101, 100
    WAIT

    SPEED 6
    MOVE G6A,101,  76, 146,  85, 98, 100
    MOVE G6D,101,  76, 146,  85, 98, 100
    WAIT

    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT
    GOTO RX_EXIT
    '**********************************************
catch_right_turn_10:

    SPEED 5
    MOVE G6A,97,  66, 145,  95, 103, 100
    MOVE G6D,97,  86, 145,  75, 103, 100
    WAIT

    SPEED 12
    MOVE G6A,94,  66, 145,  95, 101, 100
    MOVE G6D,94,  86, 145,  75, 101, 100
    WAIT
    SPEED 6
    MOVE G6A,101,  76, 146,  85, 98, 100
    MOVE G6D,101,  76, 146,  85, 98, 100
    WAIT

    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT
    GOTO RX_EXIT
    '**********************************************
    '**********************************************
catch_left_turn_20:
    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A,95,  96, 145,  65, 105, 100
    MOVE G6D,95,  56, 145,  105, 105, 100
    WAIT

    SPEED 12
    MOVE G6A,93,  96, 145,  65, 105, 100
    MOVE G6D,93,  56, 145,  105, 105, 100
    WAIT
    SPEED 6
    MOVE G6A,101,  76, 146,  85, 98, 100
    MOVE G6D,101,  76, 146,  85, 98, 100
    WAIT

    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT
    GOSUB Leg_motor_mode1
    GOTO RX_EXIT
    '**********************************************
catch_right_turn_20:
    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A,95,  56, 145,  105, 105, 100
    MOVE G6D,95,  96, 145,  65, 105, 100
    WAIT

    SPEED 12
    MOVE G6A,93,  56, 145,  105, 105, 100
    MOVE G6D,93,  96, 145,  65, 105, 100
    WAIT

    SPEED 6
    MOVE G6A,101,  76, 146,  85, 98, 100
    MOVE G6D,101,  76, 146,  85, 98, 100
    WAIT

    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT

    GOSUB Leg_motor_mode1
    GOTO RX_EXIT
    '**********************************************
catch_left_turn_45:

    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A,95,  106, 145,  55, 105, 100
    MOVE G6D,95,  46, 145,  115, 105, 100
    WAIT

    SPEED 10
    MOVE G6A,93,  106, 145,  55, 105, 100
    MOVE G6D,93,  46, 145,  115, 105, 100
    WAIT

    SPEED 8
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT
    GOSUB Leg_motor_mode1
    GOTO RX_EXIT

    '**********************************************
catch_right_turn_45:

    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A,95,  46, 145,  115, 105, 100
    MOVE G6D,95,  106, 145,  55, 105, 100
    WAIT

    SPEED 10
    MOVE G6A,93,  46, 145,  115, 105, 100
    MOVE G6D,93,  106, 145,  55, 105, 100
    WAIT

    SPEED 8
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT
    GOSUB Leg_motor_mode1
    GOTO RX_EXIT
    '**********************************************
catch_left_turn_60:

    SPEED 15
    MOVE G6A,95,  116, 145,  45, 105, 100
    MOVE G6D,95,  36, 145,  125, 105, 100
    WAIT

    SPEED 15
    MOVE G6A,90,  116, 145,  45, 105, 100
    MOVE G6D,90,  36, 145,  125, 105, 100
    WAIT

    SPEED 10
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT
    GOTO RX_EXIT

    '**********************************************
catch_right_turn_60:

    SPEED 15
    MOVE G6A,95,  36, 145,  125, 105, 100
    MOVE G6D,95,  116, 145,  45, 105, 100
    WAIT

    SPEED 15
    MOVE G6A,90,  36, 145,  125, 105, 100
    MOVE G6D,90,  116, 145,  45, 105, 100
    WAIT

    SPEED 10
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT
    GOTO RX_EXIT

    '************************************************
bow_1:
    GOSUB All_motor_mode3
    SPEED 6
    MOVE G6A,100,  70, 125, 150, 100
    MOVE G6D,100,  70, 125, 150, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    DELAY 1000
    SPEED 6
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    '************************************************
bow_2:
    GOSUB All_motor_mode3
    SPEED 4
    MOVE G6A, 108,  76, 146,  93,  94
    MOVE G6D,  88,  71, 152,  91, 110
    MOVE G6B,110,  30,  80
    MOVE G6C,90,  30,  80
    WAIT

    SPEED 8
    MOVE G6D, 90, 95, 115, 105, 112
    MOVE G6A,113,  76, 146,  93,  94
    MOVE G6B,130,  30,  80
    MOVE G6C,75,  30,  80
    WAIT

    SPEED 8
    MOVE G6A,112,  86, 120, 120,  94
    MOVE G6D,90, 100, 155,  71, 112
    MOVE G6B,140,  30,  80
    MOVE G6C,70,  30,  80
    WAIT


    SPEED 10
    MOVE G6A,108,  85, 110, 140,  94
    MOVE G6D,85, 97, 145,  91, 112
    MOVE G6B,150,  20,  40
    MOVE G6C,60,  30,  80
    WAIT

    DELAY 1000
    '*******************
    GOSUB leg_motor_mode2
    SPEED 6
    MOVE G6D, 90, 95, 115, 105, 110
    MOVE G6A,114,  76, 146,  93,  96
    MOVE G6B,130,  30,  80
    MOVE G6C,75,  30,  80
    WAIT

    SPEED 8
    MOVE G6A, 108,  76, 146,  93,  94
    MOVE G6D,  88,  71, 152,  91, 110
    MOVE G6B,110,  30,  80
    MOVE G6C,90,  30,  80
    WAIT

    SPEED 3
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    '************************************************


bow_3:
    GOSUB All_motor_mode3

    SPEED 8
    MOVE G6A,100,  76, 145,  93, 100, 100
    MOVE G6D,100,  76, 145,  93, 100, 100
    MOVE G6C,100,  70,  80
    MOVE G6B,160,  35,  80
    WAIT

    SPEED 10
    MOVE G6A,100,  76, 145,  93, 100, 100
    MOVE G6D,100,  76, 145,  93, 100, 100
    MOVE G6C,100,  80,  80
    MOVE G6B,175,  15,  15
    WAIT

    SPEED 8
    MOVE G6A,100,  55, 145,  145, 100, 100
    MOVE G6D,100,  55, 145,  145, 100, 100
    MOVE G6C,100,  80,  80
    MOVE G6B,175,  15,  15
    WAIT

    DELAY 1000
    SPEED 6
    MOVE G6A,100,  76, 145,  93, 100, 100
    MOVE G6D,100,  76, 145,  93, 100, 100
    MOVE G6C,100,  80,  80
    MOVE G6B,175,  15,  15
    WAIT

    SPEED 8
    MOVE G6A,100,  76, 145,  93, 100, 100
    MOVE G6D,100,  76, 145,  93, 100, 100
    MOVE G6C,100,  70,  80
    MOVE G6B,160,  40,  80
    WAIT

    SPEED 10
    GOSUB stand_pose
    GOSUB All_motor_Reset

    RETURN
    '************************************************

cheer:
    GOSUB Arm_motor_mode3
    GOSUB Leg_motor_mode2
    SPEED 15
    MOVE G6A,100,  80, 145,  75, 100
    MOVE G6D,100,  80, 145,  75, 100
    MOVE G6B,100,  180,  120
    MOVE G6C,100,  180,  120
    WAIT	

    SPEED 10
    FOR i = 1 TO 2

        MOVE G6B,100,  145,  100
        MOVE G6C,100,  145,  100
        WAIT

        MOVE G6B,100,  180,  130
        MOVE G6C,100,  180,  130
        WAIT	
    NEXT i
    DELAY 200
    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    ''************************************************
win_ceremony_1:
    GOSUB Arm_motor_mode3
    GOSUB Leg_motor_mode2
    SPEED 15
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  180,  120
    MOVE G6C,100,  180,  120
    WAIT	

    SPEED 10
    FOR i = 1 TO 3

        MOVE G6B,100,  145,  100
        MOVE G6C,100,  145,  100
        WAIT

        MOVE G6B,100,  180,  130
        MOVE G6C,100,  180,  130
        WAIT	
    NEXT i
    DELAY 200
    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    ''************************************************
win_ceremony_2:
    SPEED 10
    GOSUB stand_pose
    GOSUB All_motor_mode3

    SPEED 8

    MOVE G6A, 100, 163,  75,  15, 100
    MOVE G6D, 100, 163,  75,  15, 100
    MOVE G6B,185, 100, 90
    MOVE G6C,185, 100, 90
    WAIT

    SPEED 2

    MOVE G6A, 100, 165,  70,  10, 100, 100
    MOVE G6D, 100, 165,  70,  10, 100, 100
    MOVE G6B,185, 100, 90
    MOVE G6C,185, 100, 90
    WAIT

    DELAY 400
    SPEED 15
    FOR I = 1 TO 5

        MOVE G6B,185, 20, 50
        MOVE G6C,185, 20, 50
        WAIT

        MOVE G6B,185, 70, 80
        MOVE G6C,185, 70, 80
        WAIT

    NEXT I

    MOVE G6B,100, 70, 80
    MOVE G6C,100, 70, 80
    WAIT

    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A, 100, 145,  70,  80, 100, 100
    MOVE G6D, 100, 145,  70,  80, 100, 100
    MOVE G6B,100, 40, 90
    MOVE G6C,100, 40, 90
    WAIT

    SPEED 8
    MOVE G6A,100, 121,  80, 110, 101, 100
    MOVE G6D,100, 121,  80, 110, 101, 100
    MOVE G6B,100,  40,  80, , ,
    MOVE G6C,100,  40,  80, , ,
    WAIT

    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    ''************************************************
    ''************************************************
win_ceremony_3:
    GOSUB All_motor_mode3
    SPEED 15
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  180,  120
    WAIT	

    SPEED 10
    FOR i = 1 TO 4

        MOVE G6C,100,  145,  100
        WAIT

        MOVE G6C,100,  180,  130
        WAIT	
    NEXT i
    DELAY 200
    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset

    RETURN
    ''************************************************

lose_action_1:
    GOSUB Leg_motor_mode3

    SPEED 10
    MOVE G6A,100, 140,  37, 140, 100, 100
    MOVE G6D,100, 140,  37, 140, 100, 100
    WAIT

    SPEED 3
    MOVE G6A,100, 143,  28, 142, 100, 100
    MOVE G6D,100, 143,  28, 142, 100, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    SPEED 8
    MOVE G6B,140
    MOVE G6C,140
    WAIT
    SPEED 4
    MOVE G6A,95, 163,  28, 160, 105
    MOVE G6D,95, 163,  28, 160, 105
    MOVE G6B,160,  15,  90
    MOVE G6C,185,  20,  85
    WAIT

    DELAY 400
    SPEED 10
    FOR i = 1 TO 8
        MOVE G6C,165,  20,  85
        WAIT
        MOVE G6C,188,  20,  85
        WAIT  	
    NEXT i
    DELAY 500

    GOSUB Leg_motor_mode3

    SPEED 10	
    MOVE G6A,  100, 165,  28, 162, 100
    MOVE G6D,  100, 165,  28, 162, 100
    MOVE G6B,  155, 15, 90
    MOVE G6C,  155, 15, 90
    WAIT

    SPEED 10	
    MOVE G6A,  100, 150,  27, 162, 100
    MOVE G6D,  100, 150,  27, 162, 100
    MOVE G6B,  140, 15, 90
    MOVE G6C,  140, 15, 90
    WAIT

    SPEED 6
    MOVE G6A,  100, 138,  28, 155, 100
    MOVE G6D,  100, 138,  28, 155, 100
    MOVE G6B, 113,  30, 80
    MOVE G6C, 113,  30, 80
    WAIT

    GOSUB Leg_motor_mode2
    SPEED 8
    GOSUB stand_pose

    RETURN

    ''************************************************
lose_action_2:
    GOSUB Arm_motor_mode3
    SPEED 15
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  185,  170
    WAIT	

    SPEED 4
    FOR i = 1 TO 8
        MOVE G6C,100,  170,  185
        WAIT

        MOVE G6C,100,  185,  170
        WAIT	
    NEXT i
    DELAY 200
    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset

    RETURN
    ''************************************************
    ''************************************************
hug:
    GOSUB All_motor_mode3

    SPEED 12
    MOVE G6A, 88,  71, 152,  91, 110
    MOVE G6D,108,  76, 146,  93,  92
    MOVE G6B,100,  60,  80
    MOVE G6C,160,  50,  80
    WAIT

    SPEED 15
    MOVE G6A, 85,  76, 145, 94, 110
    MOVE G6D,108,  81, 135,  98, 98
    MOVE G6B,100,  60,  80
    MOVE G6C,160,  50,  80
    WAIT

    SPEED 6
    MOVE G6A, 90,  92, 115, 109, 125, 100
    MOVE G6D,103,  76, 141,  98,  82, 100
    MOVE G6B,160,  50,  80
    MOVE G6C,188,  50,  80
    WAIT	

    SPEED 5
    FOR i = 1 TO 6

        MOVE G6B,160,  50,  50
        MOVE G6C,188,  50,  50
        WAIT

        MOVE G6B,160,  55,  80
        MOVE G6C,188,  55,  80
        WAIT
    NEXT i


    SPEED 10
    MOVE G6A, 85,  76, 145, 94, 110
    MOVE G6D,108,  81, 135,  98, 98
    MOVE G6B,100,  40,  80
    MOVE G6C,160,  60,  80
    WAIT

    SPEED 10
    MOVE G6A, 88,  71, 152,  91, 110
    MOVE G6D,108,  76, 146,  93,  92
    MOVE G6B,100,  40,  80
    MOVE G6C,140,  60,  90
    WAIT

    SPEED 6
    MOVE G6A, 95,  75, 146,  93, 105
    MOVE G6D,109,  76, 146,  93,  92
    WAIT

    SPEED 3
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN


    ''************************************************
breath_action:
    GOSUB All_motor_mode3
    SPEED 5
    MOVE G6B, 150,  35, 70,
    MOVE G6C, 150,  35, 70,
    MOVE G6A,100,  63, 135, 140, 100,
    MOVE G6D,100,  63, 135, 140, 100,
    WAIT

    FOR I = 1 TO 4
        SPEED 7
        MOVE G6B, 155,  40, 90,
        MOVE G6C, 155,  40, 90,
        WAIT

        SPEED 5
        MOVE G6A,100,  58, 135, 155, 100,
        MOVE G6D,100,  58, 135, 155, 100,
        WAIT

        SPEED 7
        MOVE G6B, 155,  20, 70,
        MOVE G6C, 155,  20, 70,
        WAIT

        SPEED 5
        MOVE G6A,100,  48, 160, 135, 100,
        MOVE G6D,100,  48, 160, 135, 100,
        WAIT
    NEXT I


    SPEED 6
    MOVE G6A,100,  65, 135, 140, 100,
    MOVE G6D,100,  65, 135, 140, 100,
    MOVE G6B, 155,  40, 80,
    MOVE G6C, 155,  40, 80,
    WAIT

    SPEED 6
    GOSUB stand_pose
    GOSUB All_motor_Reset 	
    RETURN
    ''************************************************

back_standup:

    GOSUB Arm_motor_mode1
    GOSUB Leg_motor_mode1


    SPEED 15
    MOVE G6A,100, 150, 170,  40, 100
    MOVE G6D,100, 150, 170,  40, 100
    MOVE G6B, 150, 150,  45
    MOVE G6C, 150, 150,  45
    WAIT

    SPEED 15
    MOVE G6A,  100, 155,  110, 120, 100
    MOVE G6D,  100, 155,  110, 120, 100
    MOVE G6B, 190, 80,  15
    MOVE G6C, 190, 80,  15
    WAIT

    GOSUB Leg_motor_mode2
    SPEED 15	
    MOVE G6A,  100, 165,  27, 162, 100
    MOVE G6D,  100, 165,  27, 162, 100
    MOVE G6B,  155, 15, 90
    MOVE G6C,  155, 15, 90
    WAIT


    SPEED 10	
    MOVE G6A,  100, 150,  27, 162, 100
    MOVE G6D,  100, 150,  27, 162, 100
    MOVE G6B,  140, 15, 90
    MOVE G6C,  140, 15, 90
    WAIT


    SPEED 6
    MOVE G6A,  100, 138,  27, 155, 100
    MOVE G6D,  100, 138,  27, 155, 100
    MOVE G6B, 113,  30, 80
    MOVE G6C, 113,  30, 80
    WAIT

    DELAY 100
    SPEED 10
    GOSUB stand_pose
    GOSUB Leg_motor_mode1

    RETURN


    '**********************************************
front_standup:
    GOSUB Arm_motor_mode1
    GOSUB Leg_motor_mode1


    SPEED 15
    MOVE G6A,100, 15,  70, 140, 100,
    MOVE G6D,100, 15,  70, 140, 100,
    MOVE G6B,20,  140,  15
    MOVE G6C,20,  140,  15
    WAIT

    SPEED 12
    MOVE G6A,100, 136,  35, 80, 100,
    MOVE G6D,100, 136,  35, 80, 100,
    MOVE G6B,20,  30,  80
    MOVE G6C,20,  30,  80
    WAIT

    SPEED 12
    MOVE G6A,100, 165,  70, 30, 100,
    MOVE G6D,100, 165,  70, 30, 100,
    MOVE G6B,30,  20,  95
    MOVE G6C,30,  20,  95
    WAIT

    SPEED 10
    MOVE G6A,100, 165,  45, 90, 100,
    MOVE G6D,100, 165,  45, 90, 100,
    MOVE G6B,130,  50,  60
    MOVE G6C,130,  50,  60
    WAIT

    SPEED 10
    GOSUB stand_pose

    RETURN

    '******************************************
    '******************************************
    '************************************************
    '************************************************
crawl:

    GOSUB Leg_motor_mode3
    SPEED 15
    MOVE G6A,100, 155,  28, 140, 100, 100
    MOVE G6D,100, 155,  28, 140, 100, 100
    MOVE G6B,180,  40,  85
    MOVE G6C,180,  40,  85
    WAIT

    SPEED 5	
    MOVE G6A, 100, 155,  53, 160, 100, 100
    MOVE G6D, 100, 155,  53, 160, 100, 100
    MOVE G6B,190,  30, 80
    MOVE G6C,190,  30, 80
    WAIT	

    GOSUB All_motor_mode2

    DELAY 300

    SPEED 8
    PTP SETOFF
    PTP ALLOFF
    HIGHSPEED SETON

    'GOTO crawlleft_turn__LOOP

crawl_LOOP:


    MOVE G6A, 100, 160,  55, 160, 100
    MOVE G6D, 100, 145,  75, 160, 100
    MOVE G6B, 175,  25,  70
    MOVE G6C, 190,  50,  40
    WAIT
    ERX 9600, A, crawl_1
    IF A = 8 THEN GOTO crawl_1
    IF A = 9 THEN GOTO crawlright_turn__LOOP
    IF A = 7 THEN GOTO crawlleft_turn__LOOP

    GOTO crawl_after_standup

crawl_1:
    MOVE G6A, 100, 150,  70, 160, 100
    MOVE G6D, 100, 140, 120, 120, 100
    MOVE G6B, 160,  25,  70
    MOVE G6C, 190,  25,  70
    WAIT

    MOVE G6D, 100, 160,  55, 160, 100
    MOVE G6A, 100, 145,  75, 160, 100
    MOVE G6C, 175,  25,  70
    MOVE G6B, 190,  50,  40
    WAIT

    ERX 9600, A, crawl_2
    IF A = 8 THEN GOTO crawl_2
    IF A = 9 THEN GOTO crawlright_turn__LOOP
    IF A = 7 THEN GOTO crawlleft_turn__LOOP

    GOTO crawl_after_standup

crawl_2:
    MOVE G6D, 100, 140,  80, 160, 100
    MOVE G6A, 100, 140, 120, 120, 100
    MOVE G6C, 160,  25,  70
    MOVE G6B, 190,  25,  70
    WAIT

    GOTO crawl_LOOP


    '**********************************
crawlleft_turn_:

    GOSUB Leg_motor_mode3
    SPEED 15
    MOVE G6A,100, 155,  28, 140, 100, 100
    MOVE G6D,100, 155,  28, 140, 100, 100
    MOVE G6B,180,  40,  85
    MOVE G6C,180,  40,  85
    WAIT

    SPEED 5	
    MOVE G6A, 100, 155,  53, 160, 100, 100
    MOVE G6D, 100, 155,  53, 160, 100, 100
    MOVE G6B,190,  30, 80
    MOVE G6C,190,  30, 80
    WAIT	

    GOSUB All_motor_mode2

    SPEED 10
    MOVE G6A, 100, 160,  110, 100, 110
    MOVE G6D, 100, 160,  110, 100, 110
    MOVE G6C,175,  20, 80
    MOVE G6B,175,  20, 80
    WAIT

    SPEED 8
    PTP SETOFF
    PTP ALLOFF
    HIGHSPEED SETON

crawlleft_turn__LOOP:

    MOVE G6A, 100, 160,  110, 100, 110
    MOVE G6D, 100, 160,  110, 100, 110
    MOVE G6B,175,  70, 20
    MOVE G6C,175,  10, 75
    WAIT	


    ERX 9600, A, crawlleft_turn__1
    IF A = 8 THEN GOTO crawl_LOOP
    IF A = 9 THEN GOTO crawlright_turn__LOOP
    IF A = 7 THEN GOTO crawlleft_turn__1
    GOTO crawl_after_standup

crawlleft_turn__1:
    MOVE G6A, 100, 155,  110, 100, 110
    MOVE G6D, 100, 155,  110, 100, 110
    MOVE G6B,175,  80, 30
    MOVE G6C,175,  30, 95
    WAIT		


    MOVE G6A, 100, 160,  110, 100, 110
    MOVE G6D, 100, 160,  110, 100, 110
    MOVE G6B,175,  15, 75
    MOVE G6C,175,  60, 20
    WAIT		

    ERX 9600, A, crawlleft_turn__2
    IF A = 8 THEN GOTO crawl_LOOP
    IF A = 9 THEN GOTO crawlright_turn__LOOP
    IF A = 7 THEN GOTO crawlleft_turn__2
    GOTO crawl_after_standup

crawlleft_turn__2:
    MOVE G6A, 100, 155,  110, 100, 110
    MOVE G6D, 100, 155,  110, 100, 110
    MOVE G6B,175,  10, 75
    MOVE G6C,175,  10, 75
    WAIT	

    GOTO crawlleft_turn__LOOP

    '**********************************
    '**********************************
crawlright_turn_:

    GOSUB Leg_motor_mode3
    SPEED 15
    MOVE G6A,100, 155,  28, 140, 100, 100
    MOVE G6D,100, 155,  28, 140, 100, 100
    MOVE G6B,180,  40,  85
    MOVE G6C,180,  40,  85
    WAIT

    SPEED 5	
    MOVE G6A, 100, 155,  53, 160, 100, 100
    MOVE G6D, 100, 155,  53, 160, 100, 100
    MOVE G6B,190,  30, 80
    MOVE G6C,190,  30, 80
    WAIT	

    GOSUB All_motor_mode2

    SPEED 10
    MOVE G6A, 100, 160,  110, 100, 110
    MOVE G6D, 100, 160,  110, 100, 110
    MOVE G6C,175,  20, 80
    MOVE G6B,175,  20, 80
    WAIT

    SPEED 8
    PTP SETOFF
    PTP ALLOFF
    HIGHSPEED SETON

crawlright_turn__LOOP:

    MOVE G6A, 100, 160,  110, 100, 110
    MOVE G6D, 100, 160,  110, 100, 110
    MOVE G6C,175,  70, 20
    MOVE G6B,175,  10, 75
    WAIT	


    ERX 9600, A, crawlright_turn__1
    IF A = 8 THEN GOTO crawl_LOOP
    IF A = 9 THEN GOTO crawlright_turn__1
    IF A = 7 THEN GOTO crawlleft_turn__LOOP
    GOTO crawl_after_standup

crawlright_turn__1:
    MOVE G6A, 100, 155,  110, 100, 110
    MOVE G6D, 100, 155,  110, 100, 110
    MOVE G6C,175,  80, 30
    MOVE G6B,175,  30, 95
    WAIT		


    MOVE G6A, 100, 160,  110, 100, 110
    MOVE G6D, 100, 160,  110, 100, 110
    MOVE G6C,175,  15, 75
    MOVE G6B,175,  60, 20
    WAIT		

    ERX 9600, A, crawlright_turn__2
    IF A = 8 THEN GOTO crawl_LOOP
    IF A = 9 THEN GOTO crawlright_turn__2
    IF A = 7 THEN GOTO crawlleft_turn__LOOP
    GOTO crawl_after_standup

crawlright_turn__2:
    MOVE G6A, 100, 155,  110, 100, 110
    MOVE G6D, 100, 155,  110, 100, 110
    MOVE G6C,175,  10, 75
    MOVE G6B,175,  10, 75
    WAIT	

    GOTO crawlright_turn__LOOP

    '**********************************
    GOTO RX_EXIT
    '**********************************	
crawl_after_standup:
    PTP SETON		
    PTP ALLON
    SPEED 15
    HIGHSPEED SETOFF


    MOVE G6A, 100, 150,  80, 150, 100
    MOVE G6D, 100, 150,  80, 150, 100
    MOVE G6B,185,  40, 60
    MOVE G6C,185,  40, 60
    WAIT

    GOSUB Leg_motor_mode3
    DELAY 300

    SPEED 10	
    MOVE G6A,  100, 165,  25, 162, 100
    MOVE G6D,  100, 165,  25, 162, 100
    MOVE G6B,  155, 15, 90
    MOVE G6C,  155, 15, 90
    WAIT

    SPEED 10	
    MOVE G6A,  100, 150,  25, 162, 100
    MOVE G6D,  100, 150,  25, 162, 100
    MOVE G6B,  140, 15, 90
    MOVE G6C,  140, 15, 90
    WAIT

    SPEED 6
    MOVE G6A,  100, 138,  25, 155, 100
    MOVE G6D,  100, 138,  25, 155, 100
    MOVE G6B, 113,  30, 80
    MOVE G6C, 113,  30, 80
    WAIT

    DELAY 100
    SPEED 8
    GOSUB Leg_motor_mode2

    SPEED 6
    MOVE G6A,100, 140,  37, 140, 100, 100
    MOVE G6D,100, 140,  37, 140, 100, 100
    WAIT

    GOSUB stand_pose

    GOTO RX_EXIT

    '******************************************************
front_tumbling:

    SPEED 15
    MOVE G6A,100, 155,  27, 140, 100, 100
    MOVE G6D,100, 155,  27, 140, 100, 100
    MOVE G6B,130,  30,  85
    MOVE G6C,130,  30,  85
    WAIT

    SPEED 10	
    MOVE G6A, 100, 165,  55, 165, 100, 100
    MOVE G6D, 100, 165,  55, 165, 100, 100
    MOVE G6B,185,  10, 100
    MOVE G6C,185,  10, 100
    WAIT

    SPEED 15
    MOVE G6A,100, 160, 110, 140, 100, 100
    MOVE G6D,100, 160, 110, 140, 100, 100
    MOVE G6B,140,  70,  20
    MOVE G6C,140,  70,  20
    WAIT

    SPEED 15
    MOVE G6A,100,  56, 110,  26, 100, 100
    MOVE G6D,100,  71, 177, 162, 100, 100
    MOVE G6B,170,  40,  70
    MOVE G6C,170,  40,  70
    WAIT

    SPEED 15
    MOVE G6A,100,  60, 110,  15, 100, 100
    MOVE G6D,100,  70, 120, 30, 100, 100
    MOVE G6B,170,  40,  70
    MOVE G6C,170,  40,  70
    WAIT

    SPEED 15
    MOVE G6A,100,  60, 110,  15, 100, 100
    MOVE G6D,100,  60, 110,  15, 100, 100
    MOVE G6B,190,  40,  70
    MOVE G6C,190,  40,  70
    WAIT
    DELAY 50

    SPEED 15
    MOVE G6A,100, 110, 70,  65, 100, 100
    MOVE G6D,100, 110, 70,  65, 100, 100
    MOVE G6B,190, 160, 115
    MOVE G6C,190, 160, 115
    WAIT

    SPEED 10
    MOVE G6A,100, 170,  70,  15, 100, 100
    MOVE G6D,100, 170,  70,  15, 100, 100
    MOVE G6B,190, 155, 120
    MOVE G6C,190, 155, 120
    WAIT

    SPEED 10
    MOVE G6A,100, 170,  30,  110, 100, 100
    MOVE G6D,100, 170,  30,  110, 100, 100
    MOVE G6B,190,  40,  60
    MOVE G6C,190,  40,  60
    WAIT

    SPEED 13
    GOSUB sit_pose

    SPEED 10
    GOSUB stand_pose

    RETURN

    '**********************************************
    '**********************************************
back_tumbling:

    SPEED 15
    MOVE G6A,100, 170,  71,  23, 100, 100
    MOVE G6D,100, 170,  71,  23, 100, 100
    MOVE G6B, 80,  50,  70
    MOVE G6C, 80,  50,  70
    WAIT	

    MOVE G6A,100, 133,  49,  23, 100, 100
    MOVE G6D,100, 133,  49,  23, 100, 100
    MOVE G6B, 45, 116,  15
    MOVE G6C, 45, 116,  15
    WAIT

    SPEED 15
    MOVE G6A,100, 133,  49,  23, 100, 100
    MOVE G6D,100,  70, 180, 160, 100, 100
    MOVE G6B, 45,  50,  70
    MOVE G6C, 45,  50,  70
    WAIT


    SPEED 15
    MOVE G6A,100, 133, 180, 160, 100, 100
    MOVE G6D,100,  133, 180, 160, 100, 100
    MOVE G6B, 10,  50,  70
    MOVE G6C, 10,  50,  70
    WAIT

    HIGHSPEED SETON

    MOVE G6A,100, 95, 180, 160, 100, 100
    MOVE G6D,100, 95, 180, 160, 100, 100
    MOVE G6B,160,  50,  70
    MOVE G6C,160,  50,  70
    WAIT

    HIGHSPEED SETOFF '

    SPEED 15
    MOVE G6A,100, 130, 120,  80, 100, 100
    MOVE G6D, 100, 130, 120,  80, 100, 100
    MOVE G6B,160, 160,  10
    MOVE G6C,160, 160,  10
    WAIT
    '
    SPEED 15
    MOVE G6A,100, 145, 150,  90, 100, 100
    MOVE G6D, 100, 145, 150,  90, 100, 100
    MOVE G6B,180, 90,  10
    MOVE G6C,180, 90,  10
    WAIT

    SPEED 10
    MOVE G6A, 100, 145,  85, 150, 100, 100
    MOVE G6D, 100, 145,  85, 150, 100, 100
    MOVE G6B,185,  40, 60
    MOVE G6C,185,  40, 60
    WAIT

    SPEED 15
    MOVE G6A, 100, 165,  55, 155, 100, 100
    MOVE G6D, 100, 165,  55, 155, 100, 100
    MOVE G6B,185,  10, 100
    MOVE G6C,185,  10, 100
    WAIT	

    GOSUB Leg_motor_mode2
    SPEED 15	
    MOVE G6A,  100, 165,  27, 162, 100
    MOVE G6D,  100, 165,  27, 162, 100
    MOVE G6B,  155, 15, 90
    MOVE G6C,  155, 15, 90
    WAIT

    SPEED 10	
    MOVE G6A,  100, 150,  27, 162, 100
    MOVE G6D,  100, 150,  27, 162, 100
    MOVE G6B,  140, 15, 90
    MOVE G6C,  140, 15, 90
    WAIT

    SPEED 6
    MOVE G6A,  100, 138,  27, 155, 100
    MOVE G6D,  100, 138,  27, 155, 100
    MOVE G6B, 113,  30, 80
    MOVE G6C, 113,  30, 80
    WAIT

    DELAY 100
    SPEED 10
    GOSUB stand_pose
    RETURN
    '************************************************
front_lie_down:

    SPEED 10
    MOVE G6A,100, 155,  25, 140, 100,
    MOVE G6D,100, 155,  25, 140, 100,
    MOVE G6B,130,  50,  85
    MOVE G6C,130,  50,  85
    WAIT

    SPEED 3
    MOVE G6A, 100, 165,  50, 160, 100,
    MOVE G6D, 100, 165,  50, 160, 100,
    MOVE G6B,185,  10, 100
    MOVE G6C,185,  10, 100
    WAIT

    SPEED 10
    MOVE G6A,100, 130, 120,  80, 100,
    MOVE G6D,100, 130, 120,  80, 100,
    MOVE G6B,125, 160,  10
    MOVE G6C,125, 160,  10
    WAIT	

    SPEED 12
    GOSUB stand_pose

    RETURN
    '**********************************************
    '******************************************
back_lie_down:

    SPEED 10
    MOVE G6A,100, 165,  40, 100, 100,
    MOVE G6D,100, 165,  40, 100, 100,
    MOVE G6B,110,  70,  50
    MOVE G6C,110,  70,  50
    WAIT

    SPEED 10
    MOVE G6A,100, 165,  70, 15, 100,
    MOVE G6D,100, 165,  70, 15, 100,
    MOVE G6B,30,  20,  95
    MOVE G6C,30,  20,  95
    WAIT

    SPEED 15
    MOVE G6A,100, 126,  60, 50, 100,
    MOVE G6D,100, 126,  60, 50, 100,
    MOVE G6B,20,  30,  90
    MOVE G6C,20,  30,  90
    WAIT

    SPEED 10
    MOVE G6A,100, 10,  83, 140, 100,
    MOVE G6D,100, 10,  83, 140, 100,
    MOVE G6B,20,  130,  15
    MOVE G6C,20,  130,  15
    WAIT

    SPEED 10
    MOVE G6A,100, 10,  100, 115, 100,
    MOVE G6D,100, 10,  100, 115, 100,
    MOVE G6B,100,  130,  15
    MOVE G6C,100,  130,  15
    WAIT

    SPEED 10
    GOSUB stand_pose

    RETURN
    '******************************************
    '**********************************************
    '**********************************************
left_tumbling:
    GOSUB Leg_motor_mode1
    SPEED 15
    GOSUB stand_pose


    SPEED 15
    MOVE G6D,100, 125,  60, 132, 100, 100
    MOVE G6A,100, 125,  60, 132, 100, 100
    MOVE G6B,105, 150, 140
    MOVE G6C,105, 150, 140
    WAIT

    SPEED 7
    MOVE G6D,83 , 108,  85, 125, 105, 100
    MOVE G6A,108, 125,  60, 132, 110, 100
    MOVE G6B,105, 155, 145
    MOVE G6C,105, 155, 145
    WAIT


    SPEED 10
    MOVE G6D,89,  125,  60, 132, 110, 100
    MOVE G6A,110, 125,  60, 132, 122, 100
    WAIT
    SPEED 8
    MOVE G6D, 89, 125,  60, 132, 150, 100
    MOVE G6A,106, 125,  60, 132, 150, 100
    MOVE G6B,105, 167, 190
    MOVE G6C,105, 167, 190
    WAIT

    SPEED 6
    MOVE G6D,120, 125,  60, 132, 170, 100
    MOVE G6A,104, 125,  60, 132, 170, 100
    WAIT

    SPEED 12
    MOVE G6D,120, 125,  60, 132, 183, 100
    MOVE G6A,110, 125,  60, 132, 185, 100
    WAIT

    DELAY 400

    SPEED 14
    MOVE G6D,120, 125,  60, 130, 168, 100
    MOVE G6A,120, 125  60, 130, 185, 100
    MOVE G6B,105, 120, 145
    MOVE G6C,105, 120, 145
    WAIT

    SPEED 12
    MOVE G6D,105, 125,  60, 130, 183, 100
    MOVE G6A, 86, 112,  73, 127, 100, 100
    MOVE G6B,105, 120, 135
    MOVE G6C,105, 120, 135
    WAIT

    SPEED 8
    MOVE G6D,107, 125,  62, 132, 113, 100
    MOVE G6A, 82, 110,  90, 120,  100, 100
    MOVE G6B,105, 50, 80
    MOVE G6C,105, 50, 80
    WAIT

    SPEED 6
    MOVE G6A,97, 125,  60, 132, 102, 100
    MOVE G6D,97, 125,  60, 132, 102, 100
    MOVE G6B,100, 50, 80
    MOVE G6C,100, 50, 80
    WAIT

    SPEED 10
    GOSUB stand_pose
    RETURN
    '**********************************************
    '**********************************************
right_tumbling:
    GOSUB Leg_motor_mode1
    SPEED 15
    GOSUB stand_pose


    SPEED 15
    MOVE G6A,100, 125,  60, 132, 100, 100
    MOVE G6D,100, 125,  60, 132, 100, 100
    MOVE G6B,105, 150, 140
    MOVE G6C,105, 150, 140
    WAIT

    SPEED 7
    MOVE G6A,83 , 108,  85, 125, 105, 100
    MOVE G6D,108, 125,  60, 132, 110, 100
    MOVE G6B,105, 155, 145
    MOVE G6C,105, 155, 145
    WAIT


    SPEED 10
    MOVE G6A,89,  125,  60, 132, 110, 100
    MOVE G6D,110, 125,  60, 132, 122, 100
    WAIT
    SPEED 8
    MOVE G6A, 89, 125,  60, 132, 150, 100
    MOVE G6D,106, 125,  60, 132, 150, 100
    MOVE G6B,105, 167, 190
    MOVE G6C,105, 167, 190
    WAIT

    SPEED 6
    MOVE G6A,120, 125,  60, 132, 170, 100
    MOVE G6D,104, 125,  60, 132, 170, 100
    WAIT

    SPEED 12
    MOVE G6A,120, 125,  60, 132, 183, 100
    MOVE G6D,110, 125,  60, 132, 185, 100
    WAIT

    DELAY 400

    SPEED 14
    MOVE G6A,120, 125,  60, 130, 168, 100
    MOVE G6D,120, 125  60, 130, 185, 100
    MOVE G6B,105, 120, 145
    MOVE G6C,105, 120, 145
    WAIT

    SPEED 12
    MOVE G6A,105, 125,  60, 130, 183, 100
    MOVE G6D, 86, 112,  73, 127, 100, 100
    MOVE G6B,105, 120, 135
    MOVE G6C,105, 120, 135
    WAIT

    SPEED 8
    MOVE G6A,107, 125,  62, 132, 113, 100
    MOVE G6D, 82, 110,  90, 120,  100, 100
    MOVE G6B,105, 50, 80
    MOVE G6C,105, 50, 80
    WAIT

    SPEED 6
    MOVE G6A,97, 125,  60, 132, 102, 100
    MOVE G6D,97, 125,  60, 132, 102, 100
    MOVE G6B,100, 50, 80
    MOVE G6C,100, 50, 80
    WAIT

    SPEED 10
    GOSUB stand_pose
    RETURN
    '**********************************************
left_foot_sit_standup:

    GOSUB Leg_motor_mode2

    SPEED 8
    MOVE G6A,112,  77, 146,  93,  92' 60	
    MOVE G6D, 80,  71, 152,  91, 112', 60
    MOVE G6C,100,  100,  100, , , ,
    MOVE G6B,100,  100,  100, , , ,	
    WAIT


    SPEED 8
    MOVE G6A,113,  77, 146,  93,  92, 100	
    MOVE G6D,80,  150,  27, 143, 114, 100
    MOVE G6C,100,  100,  100, , , ,
    MOVE G6B,100,  100,  100, , , ,	
    WAIT

    DELAY 500


    SPEED 8
    MOVE G6A,113, 152,  27, 140, 92, 100
    MOVE G6D,85, 154,  27, 143, 114, 100,
    MOVE G6C,100,  100,  100
    MOVE G6B,100,  100,  100
    WAIT

    GOSUB Leg_motor_mode1
    DELAY 1000

    SPEED 3
    MOVE G6A,115, 152,  35, 140, 92, 100
    WAIT

    SPEED 8
    MOVE G6A,113,  77, 146,  93, 92, 100
    WAIT

    GOSUB Leg_motor_mode2
    DELAY 500

    MOVE G6A,112,  77, 146,  93,  92, 100		
    MOVE G6D, 80, 88, 125, 100, 115, 100
    MOVE G6B,100,  100,  100, , , ,
    MOVE G6C,100,  100,  100, , , ,
    WAIT


    SPEED 4
    GOSUB stand_pose
    GOSUB Leg_motor_mode1

    RETURN
    '******************************************	
right_foot_sit_standup:

    GOSUB Leg_motor_mode2

    SPEED 8
    MOVE G6D,112,  77, 146,  93,  92, '60	
    MOVE G6A, 80,  71, 152,  91, 112,' 60
    MOVE G6C,100,  100,  100, , , ,
    MOVE G6B,100,  100,  100, , , ,	
    WAIT


    SPEED 8
    MOVE G6D,113,  77, 146,  93,  92, 100	
    MOVE G6A,80,  150,  27, 140, 114, 100
    MOVE G6C,100,  100,  100, , , ,
    MOVE G6B,100,  100,  100, , , ,	
    WAIT

    DELAY 500

    SPEED 8
    MOVE G6D,113, 152,  27, 140, 92, 100
    MOVE G6A,85, 154,  27, 140, 114, 100,
    MOVE G6C,100,  100,  100
    MOVE G6B,100,  100,  100
    WAIT

    GOSUB Leg_motor_mode1
    DELAY 1000

    SPEED 3
    MOVE G6D,115, 152,  35, 140, 92, 100
    WAIT

    SPEED 8
    MOVE G6D,113,  77, 146,  93, 92, 100
    WAIT

    GOSUB Leg_motor_mode2
    DELAY 500


    MOVE G6D,112,  77, 146,  93,  92, 100		
    MOVE G6A, 80, 88, 125, 100, 115, 100
    MOVE G6B,100,  100,  100, , , ,
    MOVE G6C,100,  100,  100, , , ,
    WAIT


    SPEED 4
    GOSUB stand_pose
    GOSUB Leg_motor_mode1

    RETURN
    '**********************************************
    '********************************************	
hand_standing:
    GOSUB front_lie_down
    GOSUB Arm_motor_mode1
    GOSUB Leg_motor_mode3

    SPEED 10
    MOVE G6B, 115,  45,  70,  ,  ,  ,
    MOVE G6C,115,  45,  70,  ,  ,  ,
    WAIT

    MOVE G6A,100, 125,  65,  10, 100
    MOVE G6D,100, 125,  65,  10, 100
    MOVE G6B, 130,  45,  70,  ,  ,  ,
    MOVE G6C,130,  45,  70,  ,  ,  ,
    WAIT

    SPEED 6
    MOVE G6A,100,  89, 129,  57, 100,
    MOVE G6D, 100,  89, 129,  57, 100
    MOVE G6B, 180,  45,  70,  ,  ,  ,
    MOVE G6C, 180,  45,  70,  ,  ,  ,
    WAIT

    MOVE G6A,100,  64, 169,  60, 100,
    MOVE G6D, 100,  64, 169,  60, 100
    MOVE G6B, 190,  45,  70,  ,  ,  ,
    MOVE G6C, 190,  45,  70,  ,  ,  ,
    WAIT
    DELAY 500

    SPEED 12

    FOR i = 1 TO 4

        MOVE G6A,100, 141,  30, 120, 100
        MOVE G6D, 100,  64, 169,  60, 100
        WAIT

        MOVE G6D,100, 141,  30, 120, 100
        MOVE G6A, 100,  64, 169,  60, 100
        WAIT

    NEXT i

    MOVE G6A,100,  64, 169,  60, 100,
    MOVE G6D, 100,  64, 169,  60, 100
    MOVE G6B, 190,  45,  70,  ,  ,  ,
    MOVE G6C, 190,  45,  70,  ,  ,  ,
    WAIT

    DELAY 300

    SPEED 4
    FOR i = 1 TO 3


        MOVE G6A,70,  64, 169,  60, 130,
        MOVE G6D, 70,  64, 169,  60, 130
        WAIT

        MOVE G6A,100,  64, 169,  60, 100,
        MOVE G6D, 100,  64, 169,  60, 100
        WAIT
    NEXT i

    DELAY 300	
    SPEED 10
    MOVE G6A,100,  89, 129,  65, 100,
    MOVE G6D,100,  89, 129,  65, 100
    MOVE G6B, 180,  45,  70,  ,  ,  ,
    MOVE G6C, 180,  45,  70,  ,  ,  ,
    WAIT

    SPEED 10
    MOVE G6A,100, 125,  65,  10, 100,
    MOVE G6D, 100, 125,  65,  10, 100
    MOVE G6B, 160,  45,  70,  ,  ,  ,
    MOVE G6C, 160,  45,  70,  ,  ,  ,
    WAIT

    SPEED 10
    MOVE G6A,100, 125,  65,  10, 100,
    MOVE G6D, 100, 125,  65,  10, 100
    MOVE G6B, 110,  45,  70,  ,  ,  ,
    MOVE G6C, 110,  45,  70,  ,  ,  ,
    WAIT
    SPEED 10
    GOSUB stand_pose

    GOSUB back_standup

    RETURN
    '**********************************************	
old_dance:

    DIM w1 AS BYTE
    GOSUB Leg_motor_mode2
    GOSUB Arm_motor_mode3

    SPEED 3
    MOVE G6A, 88,  71, 152,  91, 110, '60
    MOVE G6D,108,  76, 146,  93,  92, '60
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT

    SPEED 5
    MOVE G6A, 85,  80, 140, 95, 114, 100
    MOVE G6D,112,  76, 146,  93, 98, 100
    MOVE G6B,100,  40,  90
    MOVE G6C,100,  40,  90
    WAIT


    SPEED 5
    MOVE G6A, 80,  74, 146, 94, 116, 100
    MOVE G6D,108,  81, 137,  98, 98, 100
    MOVE G6B,100,  70,  90
    MOVE G6C,100,  70,  90	
    WAIT

    SPEED 5
    MOVE G6A,94,  76, 145,  93, 106, 100
    MOVE G6D,94,  76, 145,  93, 106, 100
    MOVE G6B,100,  100,  100
    MOVE G6C,100,  100,  100
    WAIT	



    SPEED 4
    MOVE G6A,94,  94, 145,  45, 106
    MOVE G6D,94,  94, 145,  45, 106
    WAIT	

    HIGHSPEED SETON


    FOR I= 1 TO 3
        SPEED 6
        FOR w1= 0 TO 2

            MOVE G6B,100,  150,  140,
            MOVE G6C,100,  100,  190,
            MOVE G6A, 95,  94, 145,  45, 107,
            MOVE G6D, 89,  94, 145,  45, 113,
            WAIT

            MOVE G6C,100,  150,  140,
            MOVE G6B,100,  100,  190,
            MOVE G6D, 95,  94, 145,  45, 107,
            MOVE G6A, 89,  94, 145,  45, 113,
            WAIT

        NEXT w1

        SPEED 12
        MOVE G6C,100,  100,  190,
        MOVE G6B,100,  75,  100,
        MOVE G6A, 95,  94, 145,  45, 107,
        MOVE G6D, 89,  94, 145,  45, 113,
        WAIT

        SPEED 12
        MOVE G6C,100,  150,  140,
        MOVE G6B,100,  100,  100,
        MOVE G6D, 95,  94, 145,  45, 107,
        MOVE G6A, 89,  94, 145,  45, 113,
        WAIT

        DELAY 200
        SPEED 6
        FOR w1= 0 TO 2


            MOVE G6B,100,  150,  140,
            MOVE G6C,100,  100,  190,
            MOVE G6A, 95,  94, 145,  45, 107,
            MOVE G6D, 89,  94, 145,  45, 113,
            WAIT

            MOVE G6C,100,  150,  140,
            MOVE G6B,100,  100,  190,
            MOVE G6D, 95,  94, 145,  45, 107,
            MOVE G6A, 89,  94, 145,  45, 113,
            WAIT

        NEXT w1

        SPEED 15
        MOVE G6B,100,  100,  190,
        MOVE G6C,100,  75,  100,
        MOVE G6D, 89,  94, 145,  45, 113,
        MOVE G6A, 95,  94, 145,  45, 107,
        WAIT

        SPEED 12
        MOVE G6B,100,  150,  140,
        MOVE G6C,100,  100,  100,
        MOVE G6D, 95,  94, 145,  45, 107,
        MOVE G6A, 89,  94, 145,  45, 113,
        WAIT

        DELAY 100
    NEXT I
    HIGHSPEED SETOFF

    GOSUB Arm_motor_mode3	
    GOSUB Leg_motor_mode1
    SPEED 15
    MOVE G6A,100,  76, 145,  93, 98, 100
    MOVE G6D,100,  76, 145,  93, 98, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT
    SPEED 5
    DELAY 50
    GOSUB stand_pose


    RETURN

    '************************************************
    '******************************************	
    '******************************************
front_run_50:

    SPEED 12
    HIGHSPEED SETON
    GOSUB Leg_motor_mode4

    IF walk_order = 0 THEN
        walk_order = 1
        MOVE G6A,95,  76, 145,  93, 101
        MOVE G6D,101,  78, 145,  93, 98
        WAIT

        GOTO front_run_50_1
    ELSE
        walk_order = 0
        MOVE G6D,95,  76, 145,  93, 101
        MOVE G6A,101,  78, 145,  93, 98
        WAIT

        GOTO front_run_50_4
    ENDIF


    '**********************

front_run_50_1:
    MOVE G6A,95,  95, 100, 120, 104
    MOVE G6D,104,  78, 146,  91,  102
    MOVE G6B, 80
    MOVE G6C,120
    WAIT


front_run_50_2:
    MOVE G6A,95,  75, 122, 120, 104
    MOVE G6D,104,  80, 146,  89,  100
    WAIT

front_run_50_3:
    MOVE G6A,103,  70, 145, 103,  100
    MOVE G6D, 95, 88, 160,  68, 102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, front_run_50_4
    IF A <> A_old THEN  GOTO front_run_50_stop

    '*********************************

front_run_50_4:
    MOVE G6D,95,  95, 100, 120, 104
    MOVE G6A,104,  78, 146,  91,  102
    MOVE G6C, 80
    MOVE G6B,120
    WAIT


front_run_50_5:
    MOVE G6D,95,  75, 122, 120, 104
    MOVE G6A,104,  80, 146,  89,  100
    WAIT

front_run_50_6:
    MOVE G6D,103,  70, 145, 103,  100
    MOVE G6A, 95, 88, 160,  68, 102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, front_run_50_1
    IF A <> A_old THEN  GOTO front_run_50_stop


    GOTO front_run_50_1


front_run_50_stop:
    HIGHSPEED SETOFF
    SPEED 15
    GOSUB stabilized_pose
    SPEED 5
    GOSUB stand_pose

    DELAY 400

    GOSUB Leg_motor_mode1
    GOTO RX_EXIT

    '******************************************

    '******************************************
back_run_40:
    SPEED 10

    HIGHSPEED SETON
    GOSUB Leg_motor_mode5

    IF walk_order = 0 THEN
        walk_order = 1
        MOVE G6A,88,  73, 145,  96, 102
        MOVE G6D,104,  73, 145,  96, 100
        WAIT

        GOTO back_run_40_1
    ELSE
        walk_order = 0
        MOVE G6D,88,  73, 145,  96, 102
        MOVE G6A,104,  73, 145,  96, 100
        WAIT

        GOTO back_run_40_4
    ENDIF


    '**********************

back_run_40_1:
    'SPEED 10
    MOVE G6A,92,  92, 100, 115, 104
    MOVE G6D,103,  74, 145,  96,  100
    MOVE G6B, 120
    MOVE G6C,80
    'WAIT



back_run_40_2:
    'SPEED 10
    MOVE G6A,95,  100, 122, 95, 104
    MOVE G6D,103,  70, 145,  102,  100
    'WAIT

back_run_40_3:
    'SPEED 10
    MOVE G6A,103,  90, 145, 80, 100
    MOVE G6D,92,  64, 145,  108,  102
    'WAIT

    ERX 9600,A, back_run_40_4
    IF A <> A_old THEN  GOTO back_run_40_stop
    '*********************************

back_run_40_4:
    'SPEED 10
    MOVE G6D,92,  92, 100, 115, 104
    MOVE G6A,103,  74, 145,  96,  100
    MOVE G6C, 120
    MOVE G6B,80
    'WAIT


back_run_40_5:
    'SPEED 10
    MOVE G6D,95,  100, 122, 95, 104
    MOVE G6A,103,  70, 145,  102,  100
    'WAIT

back_run_40_6:
    'SPEED 10
    MOVE G6D,103,  90, 145, 80, 100
    MOVE G6A,92,  64, 145,  108,  102
    ' WAIT

    ERX 9600,A, back_run_40_1
    IF A <> A_old THEN  GOTO back_run_40_stop

    GOTO  back_run_40_1 	

back_run_40_stop:
    HIGHSPEED SETOFF
    SPEED 15
    GOSUB stabilized_pose
    SPEED 5
    GOSUB stand_pose

    DELAY 400

    GOSUB Leg_motor_mode1
    GOTO RX_EXIT

    '******************************************

    '******************************************
stabilized_stop:
    HIGHSPEED SETOFF
    SPEED 10
    GOSUB stabilized_pose
    SPEED 15
    GOSUB stand_pose
    GOSUB Leg_motor_mode1
    RETURN

    '******************************************
    '******************************************
    '************************************************
F_B_tilt_check:
    FOR i = 0 TO COUNT_MAX
        A = AD(F_B_tilt_AD_port)	
        IF A > 250 OR A < 5 THEN RETURN
        IF A > MIN AND A < MAX THEN RETURN
        DELAY tilt_check_CNT
    NEXT i

    IF A < MIN THEN GOSUB tilt_front
    IF A > MAX THEN GOSUB tilt_back

    RETURN
    '**************************************************
tilt_front:
    A = AD(F_B_tilt_AD_port)
    'IF A < MIN THEN GOSUB front_standup
    IF A < MIN THEN  GOSUB back_standup
    RETURN

tilt_back:
    A = AD(F_B_tilt_AD_port)
    'IF A > MAX THEN GOSUB back_standup
    IF A > MAX THEN GOSUB front_standup
    RETURN
    '**************************************************
L_R_tilt_check:
    FOR i = 0 TO COUNT_MAX
        B = AD(L_R_tilt_AD_port)	
        IF B > 250 OR B < 5 THEN RETURN
        IF B > MIN AND B < MAX THEN RETURN
        DELAY tilt_check_CNT
    NEXT i

    IF B < MIN OR B > MAX THEN
        SPEED 8
        MOVE G6B,140,  40,  80
        MOVE G6C,140,  40,  80
        WAIT
        GOSUB stand_pose	
        RETURN

    ENDIF
    RETURN
    '**************************************************
right_ball_kick:
    GOSUB Leg_motor_mode2
    SPEED 4
	'MOVE G6A,110,  77, 145,  85,  92, 100	
    'MOVE G6D, 85,  71, 152,  91, 114, 100
    'MOVE G6B,100,  40,  80, , , ,
    'MOVE G6C,100,  40,  80, , , ,
    
    MOVE G6A,110,  77, 145,  93,  92, 100	
    MOVE G6D, 85,  71, 152,  91, 114, 100
    MOVE G6C,100,  40,  80, , , ,
    MOVE G6B,100,  40,  80, , , ,	
    WAIT

    SPEED 10
    MOVE G6A,113,  70, 145,  113,  95	
    MOVE G6D, 83,  85, 122,  115, 114
    WAIT


    HIGHSPEED SETON

    SPEED 15
    'MOVE G6D,  83,  20, 172, 120, 114,
    'MOVE G6A, 115 ,  71, 145,  83,  95,
 	'MOVE G6B, 150,  40,  80,  ,  ,
	'MOVE G6C,  50,  40,  80,  ,  ,
	 
    MOVE G6A,113,  75, 145,  80,  95	
    MOVE G6D, 83,  20, 172,  155, 114
    MOVE G6C,50
    MOVE G6B,150
    WAIT


    DELAY 400
    HIGHSPEED SETOFF


    SPEED 10
    'MOVE G6D,  83,  55, 142,  90, 114,
    'MOVE G6A, 115,  71, 153,  80,  95,
    'MOVE G6C,  85,  40,  80,  ,  ,
    'MOVE G6B,  95,  40,  80,  ,  ,
    
    MOVE G6A,113,  72, 145,  97,  95
    MOVE G6D, 83,  58, 122,  130, 114
    MOVE G6C,100,  40,  80, , , ,
    MOVE G6B,100,  40,  80, , , ,	
    WAIT	

    SPEED 8
    
    MOVE G6A,113,  77, 145,  95,  95	
    MOVE G6D, 80,  80, 142,  95, 114
    MOVE G6C,100,  40,  80, , , ,
    MOVE G6B,100,  40,  80, , , ,
    WAIT	

    SPEED 8
    
    MOVE G6A,110,  77, 145,  93,  93, 100	
    MOVE G6D, 80,  71, 152,  91, 114, 100
    WAIT


    SPEED 3
    GOSUB stand_pose	
    GOSUB Leg_motor_mode1
    DELAY 400

    RETURN


    '******************************************
left_ball_kick:

    GOSUB Leg_motor_mode2
    SPEED 4

    MOVE G6D,110,  77, 145,  85,  92, 100	
    MOVE G6A, 85,  71, 152,  91, 114, 100
    MOVE G6C,100,  40,  80, , , ,
    MOVE G6B,100,  40,  80, , , ,	
    WAIT


    HIGHSPEED SETON

    SPEED 15
   	
    
    MOVE G6A,  83,  20, 172, 120, 114,
    MOVE G6D, 115 ,  71, 145,  83,  95,
 	MOVE G6C, 150,  40,  80,  ,  ,
	 MOVE G6B,  50,  40,  80,  ,  ,
   
    WAIT


    DELAY 200
    HIGHSPEED SETOFF


    SPEED 10
    MOVE G6A,  83,  55, 142,  90, 114,
    MOVE G6D, 115,  71, 153,  80,  95,
    MOVE G6B,  85,  40,  80,  ,  ,
    MOVE G6C,  95,  40,  80,  ,  ,

    WAIT	

    SPEED 8

    MOVE G6A,  80,  77, 142,  95, 114,
    MOVE G6D, 115,  75, 144,  95,  95,
    MOVE G6B, 100,  40,  80,  ,  ,
    MOVE G6C, 100,  40,  80,  ,  ,

    WAIT	

    SPEED 8
    MOVE G6D,110,  77, 145,  93,  93, 100	
    MOVE G6A, 80,  71, 152,  91, 114, 100
    WAIT


    SPEED 3
    GOSUB stand_pose	
    GOSUB Leg_motor_mode1
    DELAY 400

    RETURN

    '******************************************
    '**************************************
LED_ON_OFF2:

    OUT 52,1
    DELAY 150

    OUT 52,0
    DELAY 150
    RETURN
    '**************************************
LED_ON_OFF:

    OUT 52,1
    DELAY 150
    OUT 52,0
    DELAY 150

    OUT 52,1
    DELAY 150
    OUT 52,0
    DELAY 150
    RETURN
    '**************************************
    '**************************************
back_dancer_2:

    GOSUB All_motor_mode3

    GOSUB LED_ON_OFF2

    SPEED 3
    MOVE G6A, 88,  71, 152,  91, 110, '60
    MOVE G6D,108,  76, 146,  93,  92, '60
    MOVE G6B,100,  40,  80, 100, 100, 100
    MOVE G6C,100,  40,  80, 100, 100, 100
    WAIT

    SPEED 5
    MOVE G6A, 85,  80, 140, 95, 114, 100
    MOVE G6D,112,  76, 146,  93, 98, 100
    MOVE G6B,100,  40,  90, 100, 100, 100
    MOVE G6C,100,  40,  90, 100, 100, 100
    WAIT


    SPEED 5
    MOVE G6A, 80,  74, 146, 94, 116, 100
    MOVE G6D,108,  81, 137,  98, 98, 100
    MOVE G6B,100,  70,  90, 100, 100, 100
    MOVE G6C,100,  70,  90, 100, 100, 100 	
    WAIT

    SPEED 4
    MOVE G6A,98,  76, 145,  93, 103, 100
    MOVE G6D,98,  76, 145,  93, 103, 100
    MOVE G6B,100,  100,  100, 100, 100, 100
    MOVE G6C,100,  100,  100, 100, 100, 100
    WAIT	


    GOSUB LED_ON_OFF2
    '****
    SPEED 3
    MOVE G6A,98,  76, 145,  93, 103, 100
    MOVE G6D,98,  76, 145,  93, 103, 100
    WAIT

    '**********************************
    FOR i = 0 TO 3
        SPEED 4
        MOVE G6A,108,  92, 119,  106, 99
        MOVE G6D,86,  76, 145,  94, 107
        WAIT

        SPEED 4
        MOVE G6A,102,  78, 139,  98, 84
        MOVE G6D,92,  90, 115,  110, 122
        WAIT

        SPEED 4
        MOVE G6D,108,  92, 119,  106, 99
        MOVE G6A,86,  76, 145,  94, 107
        WAIT

        SPEED 4
        MOVE G6D,102,  78, 139,  98, 84
        MOVE G6A,92,  90, 115,  110, 122
        WAIT

    NEXT i

    SPEED 3
    MOVE G6A,108,  92, 119,  106, 99
    MOVE G6D,86,  76, 145,  94, 107
    MOVE G6B,, , , , ,80
    WAIT

    GOSUB Leg_motor_mode1
    SPEED 4
    MOVE G6A,98,  76, 145,  93, 98, 100
    MOVE G6D,98,  76, 145,  93, 98, 100
    MOVE G6B,100,  100,  100, 100, 100, 100
    MOVE G6C,100,  100,  100, 100, 100, 100
    WAIT

    SPEED 6
    GOSUB stand_pose

    GOSUB All_motor_Reset
    RETURN
    '******************************************
L_R_back_lift_dance:

    GOSUB All_motor_Reset

    SPEED 8
    MOVE G6A,88,  76, 145,  93, 110
    MOVE G6D,88,  76, 145,  93, 110
    MOVE G6B,100,  40,  90
    MOVE G6C,100,  40,  90
    WAIT

    DELAY 500
    HIGHSPEED SETON

    SPEED 8	
    MOVE G6B,100,  100,  120
    MOVE G6C,100,  50,  90
    WAIT

    SPEED 15
    MOVE G6B,100,  150,  180
    MOVE G6C,100,  60,  100
    WAIT
    DELAY 400

    HIGHSPEED SETOFF
    GOSUB Arm_motor_mode3
    SPEED 6
    MOVE G6B,100,  150,  150
    MOVE G6C,100,  150,  150
    WAIT

    HIGHSPEED SETON
    GOSUB Arm_motor_mode1
    SPEED 8
    MOVE G6C,100,  150,  100
    MOVE G6B,100,  100,  100
    WAIT
    DELAY 300

    SPEED 15
    MOVE G6C,100,  150,  150
    MOVE G6B,100,  50,  100
    WAIT
    DELAY 300
    HIGHSPEED SETOFF

    GOSUB Arm_motor_mode3
    GOSUB Leg_motor_mode4

    FOR I = 2 TO 5
        TEMP = I * 3
        SPEED TEMP
        MOVE G6A,  99,  96, 111, 107,  82,
        MOVE G6D,  72,  89, 120, 103, 142,
        MOVE G6B, 170,  70,  50,  ,  ,
        MOVE G6C,  100, 140, 150,  ,  ,
        WAIT

        MOVE G6D,  99,  96, 111, 107,  82,
        MOVE G6A,  72,  89, 120, 103, 142,
        MOVE G6C, 170,  70,  50,  ,  ,
        MOVE G6B,  100, 140, 150,  ,  ,
        WAIT
    NEXT I

    HIGHSPEED SETON
    FOR I = 2 TO 4
        TEMP = I * 3
        SPEED TEMP
        MOVE G6A,  99,  96, 111, 107,  82,
        MOVE G6D,  72,  89, 120, 103, 142,
        MOVE G6B, 170,  70,  50,  ,  ,
        MOVE G6C,  100, 140, 150,  ,  ,
        WAIT

        MOVE G6D,  99,  96, 111, 107,  82,
        MOVE G6A,  72,  89, 120, 103, 142,
        MOVE G6C, 170,  70,  50,  ,  ,
        MOVE G6B,  100, 140, 150,  ,  ,
        WAIT
    NEXT I

    HIGHSPEED SETOFF

    DELAY 300
    GOSUB Leg_motor_mode1
    SPEED 15
    MOVE G6A,98,  76, 145,  93, 98, 100
    MOVE G6D,98,  76, 145,  93, 98, 100
    MOVE G6B,100,  30,  80, 100, 100, 100
    MOVE G6C,100,  30,  80, 100, 100, 100
    WAIT
    GOSUB stand_pose


    RETURN
    '**********************************************
    '******************************************
Arirang_dance:

    GOSUB All_motor_Reset

    HIGHSPEED SETON
    SPEED 8
    MOVE G6A,88,  76, 145,  93, 110
    MOVE G6D,88,  76, 145,  93, 110
    MOVE G6B,100,  100, 100
    MOVE G6C,100,  100,  100
    WAIT

    DELAY 300



    HIGHSPEED SETOFF

    GOSUB Arm_motor_mode3
    GOSUB Leg_motor_mode4

    FOR I = 1 TO 8
        SPEED 10
        MOVE G6A,  99,  96, 111, 107,  82,
        MOVE G6D,  72,  89, 120, 103, 142,
        MOVE G6B,100,  170, 150
        MOVE G6C,100,  70,  70
        WAIT

        SPEED 10
        MOVE G6A,  87,  92, 115, 105,  112,
        MOVE G6D,  87,  92, 115, 105, 112,
        MOVE G6B,100,  80, 180
        MOVE G6C,100,  120,  20
        WAIT

        '***************************
        SPEED 10
        MOVE G6D,  99,  96, 111, 107,  82,
        MOVE G6A,  72,  89, 120, 103, 142,
        MOVE G6C,100,  170, 150
        MOVE G6B,100,  70,  70
        WAIT

        SPEED 10
        MOVE G6A,  87,  92, 115, 105,  112,
        MOVE G6D,  87,  92, 115, 105, 112,
        MOVE G6C,100,  80, 180
        MOVE G6B,100,  120,  20
        WAIT
    NEXT I

    SPEED 15
    MOVE G6A,  87,  92, 115, 105,  112,
    MOVE G6D,  87,  92, 115, 105, 112,
    MOVE G6C,100,  80, 90
    MOVE G6B,100,  80,  90
    WAIT

    DELAY 300
    GOSUB Leg_motor_mode1
    SPEED 15
    MOVE G6A,98,  76, 145,  93, 98, 100
    MOVE G6D,98,  76, 145,  93, 98, 100
    MOVE G6B,100,  30,  80, 100, 100, 100
    MOVE G6C,100,  30,  80, 100, 100, 100
    WAIT
    GOSUB stand_pose


    RETURN
    '**********************************************


    '**********************************************

chanson_dance:

    GOSUB All_motor_mode3

    SPEED 4
    MOVE G6A, 88,  71, 152,  91, 110, '60
    MOVE G6D,108,  76, 146,  93,  92, '60
    MOVE G6B,100,  40,  80, 100, 100, 100
    MOVE G6C,160,  30,  80, 100, 100, 100
    WAIT

    SPEED 5
    MOVE G6A, 85,  80, 140, 95, 114, 100
    MOVE G6D,110,  76, 145,  93, 98, 100
    MOVE G6B,100,  40,  90, , , 55
    MOVE G6C,185,  15,  15	
    WAIT


    SPEED 4
    MOVE G6A, 80,  74, 145, 94, 116, 100
    MOVE G6D,108,  81, 135,  98, 98, 100
    MOVE G6B,100,  40,  90, , , 55
    MOVE G6C,185,  15,  15	
    WAIT

    SPEED 6
    MOVE G6A,98,  76, 145,  93, 103, 100
    MOVE G6D,98,  76, 145,  93, 103, 100
    MOVE G6B,100,  40,  90, , , 100
    MOVE G6C,185,  30,  10	
    WAIT	

    '**** ******
    SPEED 5
    MOVE G6A,98,  79, 145,  83, 103, 100
    MOVE G6D,98,  79, 145,  83, 103, 100
    MOVE G6B,100, 50, 90, , , 100
    MOVE G6C,185,  30,  10	
    WAIT
    '**** *****

    SPEED 4
    FOR I = 0 TO 1

        MOVE G6B,100,  80,  90, , , 80
        MOVE G6C,180,  30,  10	
        WAIT	

        MOVE G6B,100,  60,  90, , , 120
        MOVE G6C,185,  35,  10	
        WAIT	

    NEXT I


    SPEED 4
    MOVE G6A,95, 100, 145,  53, 105, 100
    MOVE G6D,95,  60, 145,  93, 105, 100
    MOVE G6B,100, 40, 90, , , 145
    WAIT

    SPEED 5
    FOR I = 0 TO 1

        MOVE G6B,100,  120,  120, , , 80
        MOVE G6C,180,  30,  10	
        WAIT	

        MOVE G6B,100,  80,  90, , , 120
        MOVE G6C,185,  35,  10	
        WAIT	

    NEXT I

    SPEED 4
    MOVE G6A,98,  79, 145,  83, 103, 100
    MOVE G6D,98,  79, 145,  83, 103, 100
    MOVE G6B,100, 40, 90, , ,100
    WAIT


    SPEED 4
    MOVE G6D,95, 100, 145,  53, 105, 100
    MOVE G6A,95,  60, 145,  93, 105, 100
    MOVE G6B,, , , , , 55
    WAIT


    SPEED 4
    FOR I = 0 TO 1

        MOVE G6B,120, 60, 90, , ,80
        WAIT
        MOVE G6B,80, 40, 90, , , 120
        WAIT

    NEXT I

    SPEED 4
    MOVE G6A,98,  79, 145,  83, 103, 100
    MOVE G6D,98,  79, 145,  83, 103, 100
    MOVE G6B,100, 50, 70, , , 100
    MOVE G6C,185,  40,  10	
    WAIT

    '******************************************

    '********
    SPEED 3
    MOVE G6A,108,  95, 119,  96, 99
    MOVE G6D,86,  82, 145,  83, 107
    MOVE G6B,80, 50, 80, , , 70
    MOVE G6C,185,  20,  10	
    WAIT

    SPEED 4
    MOVE G6A,112,  83, 139,  86, 84
    MOVE G6D,80,  80, 145,  83, 122
    MOVE G6B,100, 120, 110, , , 60
    MOVE G6C,175,  40,  10	
    WAIT


    SPEED 3
    MOVE G6D,108,  95, 119,  96, 99
    MOVE G6A,86,  82, 145,  83, 107
    MOVE G6B,100, 130, 120, , , 70
    MOVE G6C,185,  40,  10	
    WAIT

    SPEED 4
    MOVE G6D,112,  82, 139,  86, 84
    MOVE G6A,80,  79, 145,  83, 122
    MOVE G6B,120, 50, 70, , , 100
    MOVE G6C,185,  30,  10	
    WAIT


    '********
    SPEED 4
    MOVE G6A,108,  78, 119,  136, 99
    MOVE G6D,86,  65, 145,  123, 107
    MOVE G6B,120, 40, 80, , , 110
    MOVE G6C,185,  30,  10	
    WAIT

    SPEED 4
    MOVE G6A,112,  83, 139,  86, 84
    MOVE G6D,80,  80, 145,  83, 122
    MOVE G6B,120, 20, 80, , , 120
    MOVE G6C,175,  40,  10	
    WAIT

    SPEED 4
    MOVE G6A,108,  78, 119,  136, 99
    MOVE G6D,86,  65, 145,  123, 107
    MOVE G6B,120, 40, 80, , , 90
    MOVE G6C,185,  30,  10	
    WAIT

    SPEED 4
    MOVE G6A,112,  83, 139,  86, 84
    MOVE G6D,80,  80, 145,  83, 122
    MOVE G6B,120, 20, 80, , , 70
    MOVE G6C,190,  20,  10	
    WAIT

    '********
    SPEED 4
    MOVE G6A,108,  95, 119,  96, 99
    MOVE G6D,86,  82, 145,  83, 107
    MOVE G6B,100, 40, 90, , , 80
    MOVE G6C,185,  20,  10	
    WAIT

    SPEED 4
    MOVE G6A,112,  83, 139,  86, 84
    MOVE G6D,80,  80, 145,  83, 122
    MOVE G6B,100, 80, 95, , , 70
    MOVE G6C,175,  40,  10	
    WAIT

    SPEED 3
    MOVE G6D,108,  95, 119,  96, 99
    MOVE G6A,86,  82, 145,  83, 107
    MOVE G6B,70, 130, 120, , , 110
    MOVE G6C,185,  30,  10	
    WAIT

    SPEED 4
    MOVE G6D,108,  86, 139,  76, 90
    MOVE G6A,85,  83, 145,  73, 122
    MOVE G6B,150, 40, 70, , , 120
    MOVE G6C,190,  30,  10	
    WAIT


    SPEED 5
    FOR I = 0 TO 2
        MOVE G6B,150, 40, 70, , , 110
        MOVE G6C,190,  20,  10	
        WAIT
        MOVE G6B,150, 40, 70, , , 90
        MOVE G6C,185,  30,  10	
        WAIT
    NEXT I

    '********
    SPEED 4
    MOVE G6A,108,  78, 119,  136, 99
    MOVE G6D,86,  65, 145,  123, 107
    MOVE G6B,120, 40, 80, , , 110
    MOVE G6C,185,  30,  10	
    WAIT

    SPEED 4
    MOVE G6A,112,  83, 139,  86, 84
    MOVE G6D,80,  80, 145,  83, 122
    MOVE G6B,190, 20, 80, , , 120
    MOVE G6C,175,  40,  10	
    WAIT

    SPEED 4
    MOVE G6D,105,  102, 119,  76, 99
    MOVE G6A,87,  89, 145,  63, 107
    MOVE G6B,80, 150, 120, , , 50
    MOVE G6C,185,  30,  10	
    WAIT

    SPEED 4
    MOVE G6D,100,  82, 139,  91, 84
    MOVE G6A,95,  104, 105,  102, 122
    MOVE G6B,150, 100, 90, , , 60
    MOVE G6C,185,  20,  10	
    WAIT

    SPEED 4
    FOR I = 0 TO 2
        MOVE G6B,150, 100, 90, , ,70
        MOVE G6C,190,  20,  10	
        WAIT
        MOVE G6B,190, 100, 90, , ,90
        MOVE G6C,185,  30,  10	
        WAIT
    NEXT I

    '************
    SPEED 3
    MOVE G6D,100,  72, 139,  101, 84
    MOVE G6A,95,  114, 105,  92, 122
    MOVE G6B,150, 100, 90, , , 120
    MOVE G6C,185,  30,  10	
    WAIT

    SPEED 4
    MOVE G6A,100,  72, 139,  101, 84
    MOVE G6D,95,  114, 105,  92, 122
    MOVE G6B,150, 100, 90, , , 60
    MOVE G6C,185,  30,  10	
    WAIT
    '****************
    SPEED 3
    MOVE G6D,100,  72, 139,  101, 84
    MOVE G6A,95,  114, 105,  92, 122
    MOVE G6B,150, 70, 80, , , 90
    MOVE G6C,185,  30,  10	
    WAIT

    SPEED 4
    MOVE G6A,100,  72, 139,  101, 84
    MOVE G6D,95,  114, 105,  92, 122
    MOVE G6B,120, 130, 120, , , 60
    MOVE G6C,185,  30,  10	
    WAIT
    '************
    SPEED 3
    MOVE G6A,98,  87, 145,  63, 103, 100
    MOVE G6D,98,  87, 145,  63, 103, 100
    MOVE G6B,60, 180, 110, , , 100
    MOVE G6C,190,  30,  10	
    WAIT
    GOSUB LED_ON_OFF2
    SPEED 5
    FOR I = 0 TO 3
        MOVE G6B,, , , , , 95
        MOVE G6C,190,  40,  10	
        WAIT
        MOVE G6B,, , , , , 105
        MOVE G6C,190,  30,  10	
        WAIT    	

    NEXT I	

    '*************************

    DELAY 1000

    SPEED 8
    MOVE G6A,98,  87, 145,  63, 103, 100
    MOVE G6D,98,  87, 145,  63, 103, 100
    MOVE G6B,120,  80,  80
    MOVE G6C,120,  80,  80
    WAIT

    GOSUB Leg_motor_mode1
    SPEED 7
    MOVE G6A,102,  76, 145,  93, 98, 100
    MOVE G6D,102,  76, 145,  93, 98, 100
    WAIT

    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset

    GOSUB bow_3

    RETURN

    '************************************************

poop_poop_dance:

    GOSUB All_motor_mode3


    '********

    SPEED 12
    MOVE G6A, 88,  71, 152,  91, 110
    MOVE G6D,108,  76, 146,  93,  92
    MOVE G6B,130,  60,  90
    MOVE G6C,160,  30,  80
    WAIT

    SPEED 12
    MOVE G6A, 85,  80, 140, 95, 114
    MOVE G6D,110,  76, 145,  93, 98
    MOVE G6B,150,  80,  90
    MOVE G6C,185,  15,  15	
    WAIT


    SPEED 12
    MOVE G6A, 80,  74, 145, 94, 116
    MOVE G6D,108,  81, 135,  98, 98
    MOVE G6B,180,  90, 90
    MOVE G6C,185,  15,  15	
    WAIT

    SPEED 6
    MOVE G6A,100,  96, 105,  113, 110
    MOVE G6D,85,  76, 145,  93, 105
    MOVE G6B,180,  90, 90
    MOVE G6C,185,  15,  15	
    WAIT	

    DELAY 300

    FOR i = 1 TO 2

        SPEED 4
        MOVE G6D,100,  96, 105,  113, 110
        MOVE G6A,85,  76, 145,  93, 105
        MOVE G6B,180,  90, 90
        MOVE G6C,185,  15,  15	
        WAIT	


        SPEED 4
        MOVE G6A,100,  96, 105,  113, 110
        MOVE G6D,85,  76, 145,  93, 105
        MOVE G6B,180,  90, 90
        MOVE G6C,185,  15,  15	
        WAIT

    NEXT i

    SPEED 6
    FOR i = 1 TO 4
        MOVE G6B,180,  90, 90
        MOVE G6C,175,  20,  13	
        WAIT

        MOVE G6B,180,  90, 90
        MOVE G6C,188,  20,  18	
        WAIT   	
    NEXT i
    DELAY 300

    SPEED 8
    MOVE G6D,100,  96, 105,  113, 110
    MOVE G6A,85,  76, 145,  93, 105
    MOVE G6C,180,  90, 90
    MOVE G6B,185,  15,  15	
    WAIT	

    FOR i = 1 TO 2

        SPEED 4
        MOVE G6A,100,  96, 105,  113, 110
        MOVE G6D,85,  76, 145,  93, 105
        MOVE G6C,180,  90, 90
        MOVE G6B,185,  15,  15	
        WAIT

        SPEED 4
        MOVE G6D,100,  96, 105,  113, 110
        MOVE G6A,85,  76, 145,  93, 105
        MOVE G6C,180,  90, 90
        MOVE G6B,185,  15,  15	
        WAIT	

    NEXT i

    SPEED 6
    FOR i = 1 TO 4
        MOVE G6C,180,  90, 90
        MOVE G6B,175,  20,  13	
        WAIT

        MOVE G6C,180,  90, 90
        MOVE G6B,188,  20,  18	
        WAIT   	
    NEXT i	 	
    DELAY 300

    SPEED 10
    MOVE G6D,95,  80, 130,  110, 105
    MOVE G6A,95,  80, 130,  110, 105
    MOVE G6C,100,  70, 80
    MOVE G6B,100,  70,  80	
    WAIT		

    GOSUB Leg_motor_mode1
    SPEED 7
    MOVE G6A,102,  76, 145,  93, 98, 100
    MOVE G6D,102,  76, 145,  93, 98, 100
    WAIT

    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset

    RETURN
    '************************************************
come_on_action:
    GOSUB All_motor_mode3

    SPEED 10
    MOVE G6B,100,  40,  80
    MOVE G6C,180,  30,  80
    WAIT


    SPEED 15
    FOR i = 1 TO 3
        MOVE G6B,80,  40,  90
        MOVE G6C,185,  15,  80	
        WAIT

        MOVE G6B,80,  40,  90
        MOVE G6C,185,  15,  20	
        WAIT
    NEXT i

    SPEED 10
    MOVE G6B,80,  40,  90
    MOVE G6C,185,  25,  90	
    WAIT
    DELAY 400

    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset

    RETURN

    '************************************************

    '************************************************
    '************************************************
    '************************************************
    '************************************************

arm_action_copy:
    GOSUB Arm_motor_mode3
    SPEED 10
    MOVE G6C,100,  70,  100
    WAIT

    MOTOROFF G6C	
    SPEED 15

    TEMPO 230
    MUSIC "cde"	

    DELAY 1000

arm_action_copy_1:

    S12 = MOTORIN(12)
    S13 = MOTORIN(13)
    S14 = MOTORIN(14)

    SERVO 6,S12
    SERVO 7,S13
    SERVO 8,S14

    ERX 9600,A,arm_action_copy_1
    IF A = 26 THEN
        TEMPO 230
        MUSIC "cdefgab"

        GOSUB MOTOR_ON
        SPEED 5
        GOSUB stand_pose

        GOTO RX_EXIT
    ENDIF	


    GOTO arm_action_copy_1
    '******************************************
matrix:
    SPEED 10
    GOSUB stand_pose
    GOSUB All_motor_mode3

    SPEED 8

    MOVE G6A, 100, 163,  75,  15, 100, 100
    MOVE G6D, 100, 163,  75,  15, 100, 100
    MOVE G6B,185, 120, 130, 100, 100, 100
    MOVE G6C,185, 120, 130, 100, 100, 100
    WAIT

    SPEED 2

    MOVE G6A, 100, 168,  70,  10, 100, 100
    MOVE G6D, 100, 168,  70,  10, 100, 100
    MOVE G6B,185, 120, 130
    MOVE G6C,185, 120, 130
    WAIT

    DELAY 1000
    SPEED 10
    FOR I = 1 TO 5

        MOVE G6B,185, 20, 50
        MOVE G6C,185, 20, 50
        WAIT

        MOVE G6B,185, 70, 80
        MOVE G6C,185, 70, 80
        WAIT

    NEXT I

    MOVE G6B,100, 70, 80
    MOVE G6C,100, 70, 80
    WAIT

    SPEED 10
    FOR I = 1 TO 5

        MOVE G6B,100, 90, 90
        MOVE G6C,100, 90, 90
        WAIT

        MOVE G6B,100, 40, 70
        MOVE G6C,100, 40, 70
        WAIT

    NEXT I

    DELAY 500
    GOSUB Leg_motor_mode2
    SPEED 8
    MOVE G6A, 100, 145,  70,  80, 100, 100
    MOVE G6D, 100, 145,  70,  80, 100, 100
    MOVE G6B,100, 40, 90, 100, 100, 100
    MOVE G6C,100, 40, 90, 100, 100, 100
    WAIT

    SPEED 8
    MOVE G6A,100, 121,  80, 110, 101, 100
    MOVE G6D,100, 121,  80, 110, 101, 100
    MOVE G6B,100,  40,  80, , ,
    MOVE G6C,100,  40,  80, , ,
    WAIT

    SPEED 8
    GOSUB stand_pose
    RETURN
    '******************************************


    '******************************************
    '******************************************

    '******************************************

    '******************************************

    '******************************************
paper_cup_chack:
    GOSUB All_motor_mode3
    SPEED 8
    MOVE G6A,100,  76, 145,  90, 100, 100
    MOVE G6D,100,  76, 145,  90, 100, 100
    MOVE G6B,190,  30,  80
    MOVE G6C,190,  30,  80
    WAIT

    SPEED 8
    MOVE G6A,100, 143,  28, 135, 190, 100
    MOVE G6D,100, 143,  28, 135, 190, 100
    MOVE G6B,190,  30,  80
    MOVE G6C,190,  30,  80
    WAIT


    DELAY 1000
    SPEED 8
    MOVE G6A,100, 143,  28, 90, 190, 100
    MOVE G6D,100, 143,  28, 90, 190, 100
    MOVE G6B,180,  30,  80
    MOVE G6C,180,  30,  80
    WAIT

    SPEED 8
    MOVE G6B,170,  20,  40
    MOVE G6C,170,  20,  40
    WAIT
    DELAY 500


    SPEED 8
    MOVE G6A,100, 143,  28, 135, 190, 100
    MOVE G6D,100, 143,  28, 135, 190, 100
    MOVE G6B,190,  20,  40
    MOVE G6C,190,  20,  40
    WAIT

    SPEED 6
    MOVE G6A,100, 143,  28, 142, 100, 100
    MOVE G6D,100, 143,  28, 142, 100, 100
    WAIT

    SPEED 8
    MOVE G6A,100,  76, 145,  90, 100, 100
    MOVE G6D,100,  76, 145,  90, 100, 100
    WAIT


    RETURN

    '************************************************
thing_chack:
    GOSUB All_motor_mode3
    SPEED 5
    MOVE G6A,100,  33, 188,  155, 100
    MOVE G6D,100,  33, 188,  155, 100
    MOVE G6B,185,  35,  80
    MOVE G6C,185,  35,  80
    WAIT

    '****************
    MOVE G6B,185,  15,  60
    MOVE G6C,185,  15,  60
    WAIT

    SPEED 4
    MOVE G6A,100,  33, 170,  155, 100
    MOVE G6D,100,  33, 170,  155, 100
    WAIT

    SPEED 5
    MOVE G6A,100,  60, 150,  115, 100
    MOVE G6D,100,  60, 150,  115, 100
    WAIT

    SPEED 6
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT

    GOSUB All_motor_Reset
    grip_pose = 1
    RETURN
    '************************************************
    '************************************************
thing_put:
    GOSUB All_motor_mode3
    SPEED 5
    MOVE G6A,100,  35, 170,  155, 100
    MOVE G6D,100,  35, 170,  155, 100
    WAIT

    DELAY 300

    MOVE G6B,185,  40,  80
    MOVE G6C,185,  40,  80
    WAIT

    SPEED 5
    MOVE G6A,100,  65, 150,  105, 100
    MOVE G6D,100,  65, 150,  105, 100
    MOVE G6B,140,  40,  80
    MOVE G6C,140,  40,  80
    WAIT


    SPEED 6
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB All_motor_Reset
    grip_pose = 0
    RETURN
    '************************************************
    '******************************************
front_both_hands_punch:
    HIGHSPEED SETON
    SPEED 7
    MOVE G6A, 92, 100, 110, 100, 107
    MOVE G6D, 92, 100, 110, 100, 107
    MOVE G6B,185, 130,  15
    MOVE G6C,185, 130,  15
    WAIT

    SPEED 10
    MOVE G6B,185, 50,  15
    MOVE G6C,185, 50,  15
    WAIT

    SPEED 13
    FOR I = 0 TO 1
        MOVE G6B,185,  10, 80
        MOVE G6C,185, 80,  10
        WAIT
        DELAY 200
        MOVE G6B,185, 80,  10
        MOVE G6C,185,  10, 80
        WAIT
        DELAY 200

    NEXT I

    MOVE G6A, 92, 100, 113, 100, 107
    MOVE G6D, 92, 100, 113, 100, 107
    MOVE G6B,185, 130,  10
    MOVE G6C,185, 130,  10
    WAIT

    HIGHSPEED SETOFF
    SPEED 12
    MOVE G6A, 102, 100, 113, 100, 98
    MOVE G6D, 102, 100, 113, 100, 98
    MOVE G6B,100,  80,  60
    MOVE G6C,100,  80,  60
    WAIT

    GOSUB stand_pose

    GOTO RX_EXIT
    ''**********************************************

    '******************************************
front_lift_attack:
    HIGHSPEED SETON
    SPEED 7
    MOVE G6A,98,  70, 146,  103, 100
    MOVE G6D,98,  70, 146,  103, 100
    MOVE G6B,133, 30,  80
    MOVE G6C,133, 30,  80
    WAIT

    SPEED 10
    MOVE G6B,133, 15,  70
    MOVE G6C,133, 15,  70
    WAIT

    SPEED 13
    MOVE G6A,98,  77, 146,  73, 100
    MOVE G6D,98,  77, 146,  73, 100
    MOVE G6B,185, 15,  70
    MOVE G6C,185, 15,  70
    WAIT

    HIGHSPEED SETOFF
    DELAY 1000

    SPEED 12
    MOVE G6A,98,  70, 146,  103, 100
    MOVE G6D,98,  70, 146,  103, 100
    MOVE G6B,133, 30,  80
    MOVE G6C,133, 30,  80
    WAIT

    SPEED 10
    GOSUB stand_pose

    GOTO RX_EXIT
    ''**********************************************
    '******************************************
back_lift_attack:
    HIGHSPEED SETON
    SPEED 7
    MOVE G6A,98,  79, 146,  83, 100
    MOVE G6D,98,  79, 146,  83, 100
    MOVE G6B,80, 30,  80
    MOVE G6C,80, 30,  80
    WAIT

    SPEED 10
    MOVE G6B,80, 20,  70
    MOVE G6C,80, 20,  70
    WAIT

    SPEED 13
    MOVE G6A,98,  68, 146,  118, 100
    MOVE G6D,98,  68, 146,  118, 100
    MOVE G6B,15, 10,  70
    MOVE G6C,15, 10,  70
    WAIT

    HIGHSPEED SETOFF
    DELAY 1000

    SPEED 12
    MOVE G6A,98,  68, 146,  103, 100
    MOVE G6D,98,  68, 146,  103, 100
    MOVE G6B,80, 30,  80
    MOVE G6C,80, 30,  80
    WAIT

    SPEED 8
    GOSUB stand_pose

    GOTO RX_EXIT
    ''**********************************************
left_front_attack:
    GOSUB Leg_motor_mode3
    SPEED 15
    MOVE G6A, 88,  71, 152,  91, 110, '60
    MOVE G6D,108,  76, 146,  93,  92, '60
    MOVE G6B,185,  80,  20
    MOVE G6C,50,  40,  80
    WAIT

    SPEED 10
    MOVE G6A, 85,  80, 140, 95, 114, 100
    MOVE G6D,112,  76, 146,  93, 98, 100
    MOVE G6B,185,  80,  20
    MOVE G6C,50,  40,  80
    WAIT
    GOSUB All_motor_Reset
    SPEED 12
    HIGHSPEED SETON

    MOVE G6A,95,  84, 105, 126,  105,
    MOVE G6D, 86, 110, 136,  69, 114,
    MOVE G6B, 189,  30,  80
    MOVE G6C,  50,  40,  80
    WAIT

    DELAY 1000
    HIGHSPEED SETOFF

    SPEED 12
    MOVE G6A,93,  80, 130, 110,  105,
    MOVE G6D, 93, 80, 130,  110, 105,
    MOVE G6B, 100,  40,  80
    MOVE G6C, 100,  40,  80
    WAIT

    SPEED 6
    MOVE G6A,101,  80, 130, 110,  98,
    MOVE G6D, 101, 80, 130,  110, 98,
    MOVE G6B, 100,  40,  80
    MOVE G6C, 100,  40,  80
    WAIT

    SPEED 10
    GOSUB stand_pose
    GOTO RX_EXIT
    ''**********************************************
right_front_attack:
    GOSUB Leg_motor_mode3
    SPEED 15
    MOVE G6A,108,  76, 146,  93,  92
    MOVE G6D, 88,  71, 152,  91, 110
    MOVE G6B,50,  40,  80
    MOVE G6C,185,  80,  20
    WAIT

    SPEED 10
    MOVE G6A,112,  76, 146,  93, 98
    MOVE G6D, 85,  80, 140, 95, 114
    MOVE G6B,50,  40,  80
    MOVE G6C,185,  80,  20
    WAIT
    GOSUB All_motor_Reset
    SPEED 12
    HIGHSPEED SETON

    MOVE G6A, 86, 110, 136,  69, 114,
    MOVE G6D,95,  84, 105, 126,  105,
    MOVE G6B,  50,  40,  80
    MOVE G6C, 189,  30,  80
    WAIT

    DELAY 1000
    HIGHSPEED SETOFF
    SPEED 12
    MOVE G6A, 93, 80, 130,  110, 105,
    MOVE G6D,93,  80, 130, 110,  105,
    MOVE G6B, 100,  40,  80
    MOVE G6C, 100,  40,  80
    WAIT

    SPEED 6
    MOVE G6A, 101, 80, 130,  110, 98,
    MOVE G6D,101,  80, 130, 110,  98,
    MOVE G6B, 100,  40,  80
    MOVE G6C, 100,  40,  80
    WAIT
    SPEED 10
    GOSUB stand_pose
    GOTO RX_EXIT
    ''**********************************************
left_side_attack:
    HIGHSPEED SETON
    GOSUB Leg_motor_mode3
    SPEED 8
    MOVE G6D,108,  76, 146,  93,  92
    MOVE G6A, 88,  71, 152,  91, 110
    MOVE G6C,100,  60,  90, , ,
    MOVE G6B,100,  170,  15, , ,
    WAIT

    GOSUB All_motor_Reset

    SPEED 8
    MOVE G6D,110,  76, 146,  93,  92
    MOVE G6A, 88,  85, 130,  100, 110
    MOVE G6C,100,  60,  90, , ,
    MOVE G6B,100,  170,  45, , ,
    WAIT

    SPEED 13
    MOVE G6D, 63, 76,  160, 85, 130	
    MOVE G6A, 88, 125,  70, 120, 115
    MOVE G6C,100,  70,  100
    MOVE G6B,100, 125, 108
    WAIT


    DELAY 1000
    HIGHSPEED SETOFF
    SPEED 15

    MOVE G6A,100, 143,  28, 142, 100, 100
    MOVE G6D,100, 143,  28, 142, 100, 100
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT

    SPEED 10
    GOSUB stand_pose
    GOTO RX_EXIT
    ''**********************************************
right_side_attack:
    HIGHSPEED SETON
    GOSUB Leg_motor_mode3
    SPEED 8
    MOVE G6A,108,  76, 146,  93,  92
    MOVE G6D, 88,  71, 152,  91, 110
    MOVE G6B,100,  60,  90, , ,
    MOVE G6C,100,  170,  15, , ,
    WAIT

    GOSUB All_motor_Reset

    SPEED 8
    MOVE G6A,110,  76, 146,  93,  92
    MOVE G6D, 88,  85, 130,  100, 110
    MOVE G6B,100,  60,  90, , ,
    MOVE G6C,100,  170,  45, , ,
    WAIT

    SPEED 13
    MOVE G6A, 63, 76,  160, 85, 130	
    MOVE G6D, 88, 125,  70, 120, 115
    MOVE G6B,100,  70,  100
    MOVE G6C,100, 125, 108
    WAIT


    DELAY 1000
    HIGHSPEED SETOFF
    SPEED 15

    MOVE G6A,100, 143,  28, 142, 100, 100
    MOVE G6D,100, 143,  28, 142, 100, 100
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT

    SPEED 10
    GOSUB stand_pose
    GOTO RX_EXIT
    ''**********************************************
    '**********************************************
left_side_back_attack:

    HIGHSPEED SETON
    SPEED 12
    MOVE G6A,95,  96, 145,  73, 105, 100
    MOVE G6D,95,  56, 145,  113, 105, 100
    MOVE G6B,15,  40,  80
    MOVE G6C,115,  40,  80
    WAIT


    DELAY 1000
    HIGHSPEED SETOFF

    SPEED 15
    MOVE G6A,93,  96, 145,  73, 105, 100
    MOVE G6D,93,  56, 145,  113, 105, 100
    WAIT

    SPEED 12
    GOSUB stand_pose

    GOTO RX_EXIT

    '**********************************************
right_side_back_attack:


    HIGHSPEED SETON
    SPEED 12
    MOVE G6A,95,  56, 145,  113, 105, 100
    MOVE G6D,95,  96, 145,  73, 105, 100
    MOVE G6B,115,  40,  80
    MOVE G6C,15,  40,  80
    WAIT


    DELAY 1000
    HIGHSPEED SETOFF

    SPEED 15
    MOVE G6A,93,  56, 145,  113, 105, 100
    MOVE G6D,93,  96, 145,  73, 105, 100
    WAIT

    SPEED 12
    GOSUB stand_pose
    GOTO RX_EXIT


    '************************************************
    '************************************************
scissors:
    GOSUB Leg_motor_mode2
    SPEED 12
    MOVE G6A,100,  96, 145,  73, 100, 100
    MOVE G6D,100,  56, 145,  113, 100, 100
    MOVE G6B,100,  40,  80
    MOVE G6C,100,  40,  80
    WAIT

    DELAY 1000

    SPEED 15
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB All_motor_Reset
    RETURN

    '************************************************
rock:
    GOSUB Leg_motor_mode2
    SPEED 15
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  60,  50
    MOVE G6C,100,  60,  50
    WAIT

    DELAY 1000

    SPEED 15
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB All_motor_Reset

    RETURN

    '************************************************
cloth:
    GOSUB Leg_motor_mode2
    SPEED 12
    MOVE G6A,91,  76, 145,  93, 108
    MOVE G6D,91,  76, 145,  93, 108
    MOVE G6B,100,  40,  90
    MOVE G6C,100,  40,  90
    WAIT

    DELAY 1000

    SPEED 5
    MOVE G6A,101,  76, 145,  93, 98
    MOVE G6D,101,  76, 145,  93, 98
    WAIT

    SPEED 6
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB All_motor_Reset
    RETURN

    '************************************************
blue_white_game_pose:
    GOSUB All_motor_mode3
    SPEED 6
    MOVE G6A,100,  76, 145,  88, 100
    MOVE G6D,100,  76, 145,  88, 100
    MOVE G6B,135,  35,  80
    MOVE G6C,135,  35,  80
    WAIT

    RETURN
    '************************************************

left_up:
    SPEED 15
    SERVO 6,165

    RETURN
    '************************************************
left_down:
    SPEED 15
    SERVO 6,105
    RETURN
    '************************************************
right_up:
    SPEED 15
    SERVO 12,165
    RETURN
    '************************************************
right_down:
    SPEED 15
    SERVO 12,105
    RETURN
    '************************************************
dice_game:
    GOSUB All_motor_mode3
    SPEED 5
    MOVE G6A,100,  30, 188,  155, 100
    MOVE G6D,100,  30, 188,  155, 100
    MOVE G6B,185,  30,  80
    MOVE G6C,185,  30,  80
    WAIT

    MOVE G6B,185,  15,  65
    MOVE G6C,185,  15,  65
    WAIT

    SPEED 4
    MOVE G6A,100,  30, 170,  155, 100
    MOVE G6D,100,  30, 170,  155, 100
    WAIT

    SPEED 5
    MOVE G6A,100,  60, 150,  115, 100
    MOVE G6D,100,  60, 150,  115, 100
    WAIT

    SPEED 6
    MOVE G6A,100,  74, 145,  85, 100
    MOVE G6D,100,  74, 145,  85, 100
    WAIT


    TEMP = RND
    TEMP = TEMP MOD 3

    IF TEMP = 0 THEN

        SPEED 4
        MOVE G6B,135,  13,  50
        MOVE G6C,188,  13,  50
        WAIT

        SPEED 15
        MOVE G6C,188,  60,  50
        MOVE G6B,165,  13,  50
        WAIT

    ELSEIF TEMP = 1 THEN

        SPEED 4
        MOVE G6C,135,  13,  50
        MOVE G6B,188,  13,  50
        WAIT

        SPEED 15
        MOVE G6B,188,  60,  50
        MOVE G6C,165,  13,  50
        WAIT

    ELSE

        SPEED 15
        MOVE G6B,188,  50,  65
        MOVE G6C,188,  50,  65
        WAIT

    ENDIF


    DELAY 1000

    SPEED 10
    MOVE G6B,140,  70,  80
    MOVE G6C,140,  70,  80
    WAIT


    SPEED 4
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80
    WAIT

    GOSUB All_motor_Reset
    RETURN
    '************************************************
    '************************************************
cup_throw_game:
    GOSUB All_motor_mode3
    SPEED 5
    MOVE G6A,100,  30, 188,  155, 100
    MOVE G6D,100,  30, 188,  155, 100
    MOVE G6B,185,  30,  80
    MOVE G6C,185,  30,  80
    WAIT

    MOVE G6B,185,  15,  65
    MOVE G6C,185,  15,  65
    WAIT

    SPEED 4
    MOVE G6A,100,  30, 170,  155, 100
    MOVE G6D,100,  30, 170,  155, 100
    WAIT

    SPEED 5
    MOVE G6A,100,  60, 150,  115, 100
    MOVE G6D,100,  60, 150,  115, 100
    WAIT

    SPEED 6
    MOVE G6A,100,  74, 145,  85, 100
    MOVE G6D,100,  74, 145,  85, 100
    WAIT

    DELAY 500

    SPEED 10
    MOVE G6B,188,  50,  65
    MOVE G6C,188,  50,  65
    WAIT

    DELAY 1000

    SPEED 8
    MOVE G6A,100,  76, 145,  93, 100, 100
    MOVE G6D,100,  76, 145,  93, 100, 100
    MOVE G6B,100,  30,  80, 100, 100, 100
    MOVE G6C,100,  30,  80, 100, 100, 100
    WAIT

    GOSUB All_motor_Reset
    RETURN
    '************************************************
    '**********************************************

front_run_30:

    SPEED 6'12
    HIGHSPEED SETON

    IF walk_order = 0 THEN
        MOVE G6A,100,  80, 119, 118, 103
        MOVE G6D,102,  75, 149,  93,  100
        MOVE G6B, 80,  30,  80
        MOVE G6C,120,  30,  80

        walk_order = 1
        GOTO front_run_30_2
    ELSE
        MOVE G6D,100,  80, 119, 118, 103
        MOVE G6A,102,  75, 149,  93,  100
        MOVE G6C, 80,  30,  80
        MOVE G6B,120,  30,  80

        walk_order = 0
        GOTO front_run_30_4

    ENDIF



    '********************************************
front_run_30_1:
    MOVE G6A,100,  80, 119, 118, 103
    MOVE G6D,102,  75, 147,  93,  100
    MOVE G6B, 80,  30,  80
    MOVE G6C,120,  30,  80

    ERX 9600, A, front_run_30_2
    GOSUB stand_pose
    HIGHSPEED SETOFF
    GOTO RX_EXIT

front_run_30_2:

    MOVE G6A,102,  74, 140, 106,  100
    MOVE G6D, 95, 105, 124,  93, 103
    MOVE G6B,100,  30,  80
    MOVE G6C,100,  30,  80

front_run_30_3:
    MOVE G6D,100,  80, 119, 118, 103
    MOVE G6A,102,  75, 147,  93,  100
    MOVE G6C, 80,  30,  80
    MOVE G6B,120,  30,  80

    DELAY 5

    ERX 9600, A, front_run_30_4
    GOSUB stand_pose
    HIGHSPEED SETOFF
    GOTO RX_EXIT

front_run_30_4:
    MOVE G6D,102,  74, 140, 106,  100
    MOVE G6A, 95, 105, 124,  93, 103
    MOVE G6C,100,  30,  80
    MOVE G6B,100,  30,  80

    '************************************************
    GOTO front_run_30_1

    GOTO RX_EXIT
    '*********************************************
    '************************************************
    '********************************************
    '********************************************

shoot_out_left:
    SPEED 10

    '*************
    MOVE G6A,90,  78, 155,  103, 100, 100
    MOVE G6D,90,  66, 140,  83, 100, 100
    WAIT

    DELAY 20	
    SPEED 15
    MOVE G6A,100,  76, 146,  93, 100, 100
    MOVE G6D,100,  76, 146,  93, 100, 100
    WAIT

    DELAY 200	
    '*************
    GOTO RX_EXIT

    '************************************************
    '************************************************

shoot_out_right:
    SPEED 10

    '*************
    MOVE G6D,90,  78, 155,  103, 100, 100
    MOVE G6A,90,  66, 140,  83, 100, 100
    WAIT

    DELAY 20	
    SPEED 15
    MOVE G6D,100,  76, 146,  93, 100, 100
    MOVE G6A,100,  76, 146,  93, 100, 100
    WAIT

    DELAY 200	
    '*************
    GOTO RX_EXIT
    '************************************************
    '************************************************
stair_left_up_3cm:
    GOSUB All_motor_mode3

    SPEED 4
    MOVE G6A, 88,  71, 152,  91, 110
    MOVE G6D,108,  76, 146,  93,  94
    MOVE G6B,100,40
    MOVE G6C,100,40
    WAIT

    SPEED 10
    MOVE G6A, 90, 100, 110, 100, 114
    MOVE G6D,113,  78, 146,  93,  94
    WAIT

    GOSUB Leg_motor_mode2

    SPEED 15
    MOVE G6A, 90, 140, 35, 130, 114
    MOVE G6D,113,  73, 155,  90,  94
    WAIT


    SPEED 12
    MOVE G6A,  80, 40, 115, 160, 114,
    MOVE G6D,113,  70, 155,  90,  94
    WAIT

    GOSUB Leg_motor_mode3

    SPEED 7
    MOVE G6A, 105, 70, 100, 160, 100,
    MOVE G6D,80,  90, 165,  70, 100
    MOVE G6B,160,50
    MOVE G6C,160,40
    WAIT

    SPEED 6
    MOVE G6A, 113, 90, 80, 160,95,
    MOVE G6D,70,  95, 165,  65, 105
    MOVE G6B,180,50
    MOVE G6C,180,30
    WAIT

    '****************************
    GOSUB Leg_motor_mode2	
    SPEED 8
    MOVE G6A, 114, 90, 100, 150,95,
    MOVE G6D,75,  90, 165,  70, 105
    WAIT

    SPEED 12
    MOVE G6A, 114, 90, 100, 150,95,
    MOVE G6D,90,  120, 40,  140, 108
    WAIT

    SPEED 10
    MOVE G6A, 114, 90, 110, 130,95,
    MOVE G6D,90,  95, 90,  145, 108
    MOVE G6B,140,50
    MOVE G6C,140,30
    WAIT

    SPEED 10
    MOVE G6A, 110, 90, 110, 130,95,
    MOVE G6D,80,  85, 110,  135, 108
    MOVE G6B,110,40
    MOVE G6C,110,40
    WAIT

    SPEED 5
    MOVE G6D, 98, 90, 110, 125,99,
    MOVE G6A,98,  90, 110,  125, 99
    MOVE G6B,110,40
    MOVE G6C,110,40
    WAIT

    SPEED 6
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    '****************************************

stair_right_up_3cm:
    GOSUB All_motor_mode3

    SPEED 4
    MOVE G6D, 88,  71, 152,  91, 110
    MOVE G6A,108,  76, 146,  93,  94
    MOVE G6B,100,40
    MOVE G6C,100,40
    WAIT

    SPEED 10
    MOVE G6D, 90, 100, 110, 100, 114
    MOVE G6A,113,  78, 146,  93,  94
    WAIT

    GOSUB Leg_motor_mode2

    SPEED 15
    MOVE G6D, 90, 140, 35, 130, 114
    MOVE G6A,113,  73, 155,  90,  94
    WAIT


    SPEED 12
    MOVE G6D,  80, 40, 115, 160, 114,
    MOVE G6A,113,  70, 155,  90,  94
    WAIT

    GOSUB Leg_motor_mode3

    SPEED 7
    MOVE G6D, 105, 70, 100, 160, 100,
    MOVE G6A,80,  90, 165,  70, 100
    MOVE G6C,160,50
    MOVE G6B,160,40
    WAIT

    SPEED 6
    MOVE G6D, 113, 90, 80, 160,95,
    MOVE G6A,70,  95, 165,  65, 105
    MOVE G6C,180,50
    MOVE G6B,180,30
    WAIT

    '****************************
    GOSUB Leg_motor_mode2	
    SPEED 8
    MOVE G6D, 114, 90, 100, 150,95,
    MOVE G6A,75,  90, 165,  70, 105
    WAIT

    SPEED 12
    MOVE G6D, 114, 90, 100, 150,95,
    MOVE G6A,90,  120, 40,  140, 108
    WAIT

    SPEED 10
    MOVE G6D, 114, 90, 110, 130,95,
    MOVE G6A,90,  95, 90,  145, 108
    MOVE G6C,140,50
    MOVE G6B,140,30
    WAIT

    SPEED 10
    MOVE G6D, 110, 90, 110, 130,95,
    MOVE G6A,80,  85, 110,  135, 108
    MOVE G6B,110,40
    MOVE G6C,110,40
    WAIT

    SPEED 5
    MOVE G6D, 98, 90, 110, 125,99,
    MOVE G6A,98,  90, 110,  125, 99
    MOVE G6B,110,40
    MOVE G6C,110,40
    WAIT

    SPEED 6
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    '********************************************	

one_step:
    SPEED 10
    HIGHSPEED SETON
    GOSUB All_motor_mode3

    FOR i = 0 TO 1
        'front_narrow_walk_1:
        MOVE G6A,95,  95, 120, 100, 104
        MOVE G6D,104,  77, 146,  91,  102
        MOVE G6B, 80
        MOVE G6C,120
        WAIT

        'front_narrow_walk_2:
        MOVE G6A,95,  85, 130, 103, 104
        MOVE G6D,104,  79, 146,  89,  100
        WAIT

        'front_narrow_walk_3:
        MOVE G6A,103,   85, 130, 103,  100
        MOVE G6D, 95,  79, 146,  89, 102
        WAIT

        GOSUB F_B_tilt_check

        'ERX 9600,A, front_narrow_walk_4
        'IF A <> A_old THEN  GOTO front_narrow_walk_stop

        '*********************************

        'front_narrow_walk_4:
        MOVE G6D,95,  95, 120, 100, 104
        MOVE G6A,104,  77, 146,  91,  102
        MOVE G6C, 80
        MOVE G6B,120
        WAIT


        'front_narrow_walk_5:
        MOVE G6D,95,  85, 130, 103, 104
        MOVE G6A,104,  79, 146,  89,  100
        WAIT

        'front_narrow_walk_6:
        MOVE G6D,103,   85, 130, 103,  100
        MOVE G6A, 95,  79, 146,  89, 102
        WAIT

        GOSUB F_B_tilt_check

        'ERX 9600,A, front_narrow_walk_1
        'IF A <> A_old THEN  GOTO front_narrow_walk_stop

    NEXT i
    GOTO front_narrow_walk_stop


    '******************************************
front_narrow_walk:

    SPEED 10
    HIGHSPEED SETON
    GOSUB All_motor_mode3

    FOR i = 0 TO 2
        'front_narrow_walk_1:
        MOVE G6A,95,  95, 120, 100, 104
        MOVE G6D,104,  77, 146,  91,  102
        MOVE G6B, 80
        MOVE G6C,120
        WAIT

        'front_narrow_walk_2:
        MOVE G6A,95,  85, 130, 103, 104
        MOVE G6D,104,  79, 146,  89,  100
        WAIT

        'front_narrow_walk_3:
        MOVE G6A,103,   85, 130, 103,  100
        MOVE G6D, 95,  79, 146,  89, 102
        WAIT

        'GOSUB F_B_tilt_check

        'ERX 9600,A, front_narrow_walk_4
        'IF A <> A_old THEN  GOTO front_narrow_walk_stop

        '*********************************

        'front_narrow_walk_4:
        MOVE G6D,95,  95, 120, 100, 104
        MOVE G6A,104,  77, 146,  91,  102
        MOVE G6C, 80
        MOVE G6B,120
        WAIT


        'front_narrow_walk_5:
        MOVE G6D,95,  85, 130, 103, 104
        MOVE G6A,104,  79, 146,  89,  100
        WAIT

        'front_narrow_walk_6:
        MOVE G6D,103,   85, 130, 103,  100
        MOVE G6A, 95,  79, 146,  89, 102
        WAIT

        'GOSUB F_B_tilt_check

        'ERX 9600,A, front_narrow_walk_1
        'IF A <> A_old THEN  GOTO front_narrow_walk_stop

    NEXT i
    GOTO front_narrow_walk_stop

    '******************************************
front_narrow_walk_continue:

    SPEED 10
    HIGHSPEED SETON
    GOSUB All_motor_mode3

    IF walk_order = 0 THEN
        walk_order = 1
        MOVE G6A,95,  76, 145,  93, 101
        MOVE G6D,101,  77, 145,  93, 98
        MOVE G6B,100,  35
        MOVE G6C,100,  35
        WAIT

        GOTO front_narrow_walk_1
    ELSE
        walk_order = 0
        MOVE G6D,95,  76, 145,  93, 101
        MOVE G6A,101,  77, 145,  93, 98
        MOVE G6B,100,  35
        MOVE G6C,100,  35
        WAIT

        GOTO front_narrow_walk_4
    ENDIF


    '**********************

front_narrow_walk_1:
    MOVE G6A,95,  95, 120, 100, 104
    MOVE G6D,104,  77, 146,  91,  102
    MOVE G6B, 80
    MOVE G6C,120
    WAIT


front_narrow_walk_2:
    MOVE G6A,95,  85, 130, 103, 104
    MOVE G6D,104,  79, 146,  89,  100
    WAIT

front_narrow_walk_3:
    MOVE G6A,103,   85, 130, 103,  100
    MOVE G6D, 95,  79, 146,  89, 102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, front_narrow_walk_4
    IF A <> A_old THEN  GOTO front_narrow_walk_stop

    '*********************************

front_narrow_walk_4:
    MOVE G6D,95,  95, 120, 100, 104
    MOVE G6A,104,  77, 146,  91,  102
    MOVE G6C, 80
    MOVE G6B,120
    WAIT


front_narrow_walk_5:
    MOVE G6D,95,  85, 130, 103, 104
    MOVE G6A,104,  79, 146,  89,  100
    WAIT

front_narrow_walk_6:
    MOVE G6D,103,   85, 130, 103,  100
    MOVE G6A, 95,  79, 146,  89, 102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, front_narrow_walk_1
    IF A <> A_old THEN  GOTO front_narrow_walk_stop


    GOTO front_narrow_walk_1

front_narrow_walk_stop:
    HIGHSPEED SETOFF
    SPEED 15
    GOSUB stabilized_pose
    SPEED 10
    GOSUB stand_pose

    DELAY 400

    GOSUB Leg_motor_mode1
    GOTO RX_EXIT

    '******************************************
    '******************************************
back_narrow_walk:

    SPEED 11
    HIGHSPEED SETON
    GOSUB All_motor_mode3

    IF walk_order = 0 THEN
        walk_order = 1
        MOVE G6A,95,  76, 145,  93, 101
        MOVE G6D,101,  77, 145,  93, 98
        MOVE G6B,100,  35
        MOVE G6C,100,  35
        WAIT

        GOTO back_narrow_walk_1
    ELSE
        walk_order = 0
        MOVE G6D,95,  76, 145,  93, 101
        MOVE G6A,101,  77, 145,  93, 98
        MOVE G6B,100,  35
        MOVE G6C,100,  35
        WAIT

        GOTO back_narrow_walk_4
    ENDIF


    '**********************

back_narrow_walk_1:
    MOVE G6D,104,  77, 146,  91,  102
    MOVE G6A,95,  95, 120, 100, 104
    MOVE G6B,115
    MOVE G6C,85
    WAIT


back_narrow_walk_2:
    MOVE G6A,95,  90, 135, 90, 104
    MOVE G6D,104,  77, 146,  91,  100
    WAIT

back_narrow_walk_3:
    MOVE G6A, 103,  79, 146,  89, 100
    MOVE G6D,95,   65, 146, 103,  102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, back_narrow_walk_4
    IF A <> A_old THEN  GOTO back_narrow_walk_stop

    '*********************************

back_narrow_walk_4:
    MOVE G6D,95,  95, 120, 100, 104
    MOVE G6A,104,  77, 146,  91,  102
    MOVE G6C,115
    MOVE G6B,85
    WAIT


back_narrow_walk_5:
    MOVE G6A,104,  77, 146,  91,  100
    MOVE G6D,95,  90, 135, 90, 104
    WAIT

back_narrow_walk_6:
    MOVE G6D, 103,  79, 146,  89, 100
    MOVE G6A,95,   65, 146, 103,  102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, back_narrow_walk_1
    IF A <> A_old THEN  GOTO back_narrow_walk_stop


    GOTO back_narrow_walk_1


back_narrow_walk_stop:
    HIGHSPEED SETOFF
    SPEED 15
    GOSUB stabilized_pose
    SPEED 10
    GOSUB stand_pose

    DELAY 400

    GOSUB Leg_motor_mode1
    GOTO RX_EXIT

    '******************************************
    '************************************************
stair_left_down_3cm:
    GOSUB All_motor_mode3

    SPEED 4
    MOVE G6A, 88,  71, 152,  91, 110
    MOVE G6D,108,  76, 145,  93,  94
    MOVE G6B,100,40
    MOVE G6C,100,40
    WAIT

    SPEED 10
    MOVE G6A, 90, 100, 115, 105, 114
    MOVE G6D,113,  76, 145,  93,  94
    WAIT

    GOSUB Leg_motor_mode2


    SPEED 12
    MOVE G6A,  80, 30, 155, 150, 114,
    MOVE G6D,113,  65, 155,  90,  94
    WAIT

    GOSUB Leg_motor_mode2

    SPEED 7
    MOVE G6A,  80, 30, 175, 150, 114,
    MOVE G6D,113,  115, 65,  140,  94
    MOVE G6B,70,50
    MOVE G6C,70,40
    WAIT

    GOSUB Leg_motor_mode3
    SPEED 5
    MOVE G6A,90, 20, 150, 150, 110
    MOVE G6D,110,  155, 35,  120,94
    MOVE G6B,100,50
    MOVE G6C,140,40
    WAIT

    '****************************

    SPEED 8
    MOVE G6A,100, 30, 150, 150, 100
    MOVE G6D,100,  155, 70,  100,100
    MOVE G6B,140,50
    MOVE G6C,100,40
    WAIT

    SPEED 10
    MOVE G6A,114, 70, 130, 150, 94
    MOVE G6D,80,  125, 140,  85,114
    MOVE G6B,170,50
    MOVE G6C,100,40
    WAIT

    GOSUB Leg_motor_mode2	
    SPEED 10
    MOVE G6A,114, 70, 130, 150, 94
    MOVE G6D,80,  125, 50,  150,114
    WAIT

    SPEED 9
    MOVE G6A,114, 75, 130, 120, 94
    MOVE G6D,80,  85, 90,  150,114
    WAIT

    SPEED 8
    MOVE G6A,112, 80, 130, 110, 94
    MOVE G6D,80,  75,130,  115,114
    MOVE G6B,130,50
    MOVE G6C,100,40
    WAIT

    SPEED 6
    MOVE G6D, 98, 80, 130, 105,99,
    MOVE G6A,98,  80, 130,  105, 99
    MOVE G6B,110,40
    MOVE G6C,110,40
    WAIT

    SPEED 4
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    '****************************************
    '************************************************
stair_right_down_3cm:
    GOSUB All_motor_mode3

    SPEED 4
    MOVE G6D, 88,  71, 152,  91, 110
    MOVE G6A,108,  76, 145,  93,  94
    MOVE G6B,100,40
    MOVE G6C,100,40
    WAIT

    SPEED 10
    MOVE G6D, 90, 100, 115, 105, 114
    MOVE G6A,113,  76, 145,  93,  94
    WAIT

    GOSUB Leg_motor_mode2


    SPEED 12
    MOVE G6D,  80, 30, 155, 150, 114,
    MOVE G6A,113,  65, 155,  90,  94
    WAIT

    GOSUB Leg_motor_mode2

    SPEED 7
    MOVE G6D,  80, 30, 175, 150, 114,
    MOVE G6A,113,  115, 65,  140,  94
    MOVE G6B,70,50
    MOVE G6C,70,40
    WAIT

    GOSUB Leg_motor_mode3
    SPEED 5
    MOVE G6D,90, 20, 150, 150, 110
    MOVE G6A,110,  155, 35,  120,94
    MOVE G6C,100,50
    MOVE G6B,140,40
    WAIT

    '****************************

    SPEED 8
    MOVE G6D,100, 30, 150, 150, 100
    MOVE G6A,100,  155, 70,  100,100
    MOVE G6C,140,50
    MOVE G6B,100,40
    WAIT

    SPEED 10
    MOVE G6D,114, 70, 130, 150, 94
    MOVE G6A,80,  125, 140,  85,114
    MOVE G6C,170,50
    MOVE G6B,100,40
    WAIT

    GOSUB Leg_motor_mode2	
    SPEED 10
    MOVE G6D,114, 70, 130, 150, 94
    MOVE G6A,80,  125, 50,  150,114
    WAIT

    SPEED 9
    MOVE G6D,114, 75, 130, 120, 94
    MOVE G6A,80,  85, 90,  150,114
    WAIT

    SPEED 8
    MOVE G6D,112, 80, 130, 110, 94
    MOVE G6A,80,  75,130,  115,114
    MOVE G6C,130,50
    MOVE G6B,100,40
    WAIT

    SPEED 6
    MOVE G6D, 98, 80, 130, 105,99,
    MOVE G6A,98,  80, 130,  105, 99
    MOVE G6B,110,40
    MOVE G6C,110,40
    WAIT

    SPEED 4
    GOSUB stand_pose
    GOSUB All_motor_Reset
    RETURN
    '************************************************

    '******************************************
catch_running:

    SPEED 15
    GOSUB All_motor_mode3

    IF walk_order = 0 THEN
        walk_order = 1
        MOVE G6A,95,  76, 145,  85, 101
        MOVE G6D,101,  77, 145,  85, 98
        WAIT

        GOTO catch_running_1
    ELSE
        walk_order = 0
        MOVE G6D,95,  76, 145,  85, 101
        MOVE G6A,101,  77, 145,  85, 98
        WAIT

        GOTO catch_running_4
    ENDIF


    '**********************

catch_running_1:
    MOVE G6A,95,  95, 120, 95, 104
    MOVE G6D,104,  77, 145,  87,  102
    WAIT
    DELAY 5

catch_running_2:
    MOVE G6D,104,  79, 145,  82,  100
    MOVE G6A,95,  85, 130, 95, 104
    WAIT
    DELAY 5
catch_running_3:
    MOVE G6A,103,   85, 130, 95,  100
    MOVE G6D, 97,  79, 145,  82, 102
    WAIT
    DELAY 5

    GOSUB F_B_tilt_check

    ERX 9600,A, catch_running_4
    IF A <> A_old THEN  GOTO catch_running_stop

    '*********************************

catch_running_4:
    MOVE G6D,95,  95, 120, 95, 104
    MOVE G6A,104,  77, 145,  87,  102
    WAIT


catch_running_5:
    MOVE G6D,95,  85, 130, 95, 104
    MOVE G6A,104,  79, 145,  82,  100
    WAIT

catch_running_6:
    MOVE G6D,103,   85, 130, 95,  100
    MOVE G6A, 97,  79, 145,  82, 102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, catch_running_1
    IF A <> A_old THEN  GOTO catch_running_stop


    GOTO catch_running_1


catch_running_stop:
    HIGHSPEED SETOFF
    SPEED 15
    MOVE G6A,98,  76, 145,  85, 101, 100
    MOVE G6D,98,  76, 145,  85, 101, 100
    SPEED 10
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT

    DELAY 400

    GOSUB Leg_motor_mode1
    GOTO RX_EXIT

    '******************************************
    '******************************************
catch_back_walk:

    SPEED 10
    HIGHSPEED SETON
    GOSUB All_motor_mode3

    IF walk_order = 0 THEN
        walk_order = 1
        MOVE G6A,95,  76, 145,  85, 101
        MOVE G6D,101,  77, 145,  85, 98
        WAIT

        GOTO catch_back_walk_1
    ELSE
        walk_order = 0
        MOVE G6D,95,  76, 145,  85, 101
        MOVE G6A,101,  77, 145,  85, 98
        WAIT

        GOTO catch_back_walk_4
    ENDIF


    '**********************

catch_back_walk_1:
    MOVE G6D,104,  77, 146,  83,  102
    MOVE G6A,95,  95, 120, 92, 104
    WAIT


catch_back_walk_2:
    MOVE G6A,95,  90, 135, 82, 104
    MOVE G6D,104,  77, 146,  83,  100
    WAIT

catch_back_walk_3:
    MOVE G6A, 103,  79, 146,  81, 100
    MOVE G6D,95,   65, 146, 95,  102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, catch_back_walk_4
    IF A <> A_old THEN  GOTO catch_back_walk_stop

    '*********************************

catch_back_walk_4:
    MOVE G6D,95,  95, 120, 92, 104
    MOVE G6A,104,  77, 146,  83,  102
    WAIT


catch_back_walk_5:
    MOVE G6A,104,  77, 146,  83,  100
    MOVE G6D,95,  90, 135, 82, 104
    WAIT

catch_back_walk_6:
    MOVE G6D, 103,  79, 146,  81, 100
    MOVE G6A,95,   65, 146, 95,  102
    WAIT

    GOSUB F_B_tilt_check

    ERX 9600,A, catch_back_walk_1
    IF A <> A_old THEN  GOTO catch_back_walk_stop


    GOTO catch_back_walk_1


catch_back_walk_stop:
    HIGHSPEED SETOFF
    SPEED 15
    MOVE G6A,98,  76, 145,  85, 101, 100
    MOVE G6D,98,  76, 145,  85, 101, 100
    SPEED 10
    MOVE G6A,100,  76, 145,  85, 100
    MOVE G6D,100,  76, 145,  85, 100
    WAIT

    DELAY 400


    GOSUB Leg_motor_mode1
    GOTO RX_EXIT

    '******************************************
    '*********************************************	
front_sit_walk:
    GOSUB All_motor_mode3
    SPEED 9

front_sit_walk_1:

    MOVE G6A,114, 143,  28, 142,  96, 100
    MOVE G6D, 87, 135,  28, 155, 110, 100
    WAIT


    MOVE G6D,98, 126,  28, 160, 102, 100
    MOVE G6A,98, 160,  28, 125, 102, 100
    WAIT

    ERX 9600, A, front_sit_walk_2
    SPEED 6
    IF  grip_pose = 0 THEN
        GOSUB sit_pose
    ELSE
        MOVE G6A,100, 140,  28, 142, 100, 100
        MOVE G6D,100, 140,  28, 142, 100, 100
        WAIT
        pose = 1
    ENDIF
    GOSUB All_motor_Reset
    GOTO RX_EXIT

front_sit_walk_2:
    MOVE G6D,113, 143,  28, 142,  96, 100
    MOVE G6A, 87, 135,  28, 155, 110, 100
    WAIT

    MOVE G6A,98, 126,  28, 160, 102, 100
    MOVE G6D,98, 160,  28, 125, 102, 100
    WAIT

    ERX 9600, A, front_sit_walk_1
    SPEED 6
    IF  grip_pose = 0 THEN
        GOSUB sit_pose
    ELSE
        MOVE G6A,100, 140,  28, 142, 100, 100
        MOVE G6D,100, 140,  28, 142, 100, 100
        WAIT
        pose = 1
    ENDIF
    GOSUB All_motor_Reset
    GOTO RX_EXIT


    GOTO front_sit_walk_1
    '*****************************
back_sit_walk:
    GOSUB All_motor_mode3
    SPEED 8

back_sit_walk_1:

    MOVE G6D,113, 140,  28, 142,  96, 100
    MOVE G6A, 87, 140,  28, 140, 110, 100
    WAIT

    MOVE G6A,98, 155,  28, 125, 102, 100
    MOVE G6D,98, 121,  28, 160, 102, 100
    WAIT

    ERX 9600, A, back_sit_walk_2
    SPEED 6
    IF  grip_pose = 0 THEN
        GOSUB sit_pose
    ELSE
        MOVE G6A,100, 140,  28, 142, 100, 100
        MOVE G6D,100, 140,  28, 142, 100, 100
        WAIT
        pose = 1
    ENDIF
    GOSUB All_motor_Reset
    GOTO RX_EXIT

back_sit_walk_2:
    MOVE G6A,113, 140,  28, 142,  96, 100
    MOVE G6D, 87, 140,  28, 140, 110, 100
    WAIT


    MOVE G6D,98, 155,  28, 125, 102, 100
    MOVE G6A,98, 121,  28, 160, 102, 100
    WAIT

    ERX 9600, A, back_sit_walk_1
    SPEED 6
    IF  grip_pose = 0 THEN
        GOSUB sit_pose
    ELSE
        MOVE G6A,100, 140,  28, 142, 100, 100
        MOVE G6D,100, 140,  28, 142, 100, 100
        WAIT
        pose = 1
    ENDIF
    GOSUB All_motor_Reset
    GOTO RX_EXIT


    GOTO back_sit_walk_1
    '*****************************		

sit_right_walk:
    GOSUB All_motor_mode3
    SPEED 6
    MOVE G6A,110, 145,  28, 140, 100, 100
    MOVE G6D, 86, 145,  28, 140, 105, 100
    WAIT

    SPEED 5
    MOVE G6A, 90, 145,  28, 140, 110, 100
    MOVE G6D, 90, 145,  28, 140, 110, 100
    WAIT

    SPEED 6 	
    MOVE G6A, 80, 135,  45, 135, 105, 100
    MOVE G6D,108, 145,  28, 140, 100, 100
    WAIT

    SPEED 4 		
    MOVE G6A, 90, 145,  28, 140, 100, 100
    MOVE G6D,106, 145,  28, 140, 100, 100
    WAIT

    SPEED 3
    IF  grip_pose = 0 THEN
        GOSUB sit_pose
    ELSE
        MOVE G6A,100, 140,  28, 142, 100, 100
        MOVE G6D,100, 140,  28, 142, 100, 100
        WAIT
        pose = 1
    ENDIF
    GOSUB All_motor_Reset
    GOTO RX_EXIT
    '*****************************	
sit_left_walk:
    GOSUB All_motor_mode3
    SPEED 6
    MOVE G6D,110, 145,  28, 140, 100, 100
    MOVE G6A, 86, 145,  28, 140, 105, 100
    WAIT

    SPEED 5
    MOVE G6D, 90, 145,  28, 140, 110, 100
    MOVE G6A, 90, 145,  28, 140, 110, 100
    WAIT

    SPEED 6 	
    MOVE G6D, 80, 135,  45, 135, 105, 100
    MOVE G6A,108, 145,  28, 140, 100, 100
    WAIT

    SPEED 4 		
    MOVE G6D, 90, 145,  28, 140, 100, 100
    MOVE G6A,106, 145,  28, 140, 100, 100
    WAIT

    SPEED 3
    IF  grip_pose = 0 THEN
        GOSUB sit_pose
    ELSE
        MOVE G6A,100, 140,  28, 142, 100, 100
        MOVE G6D,100, 140,  28, 142, 100, 100
        WAIT
        pose = 1
    ENDIF
    GOSUB All_motor_Reset
    GOTO RX_EXIT
    '**********************************************	
left_hand_up:

    SPEED 15
    MOVE G6B,,100
    WAIT

    GOSUB stand_pose
    RETURN
    '**********************************************
right_hand_up:
    SPEED 15
    MOVE G6C,,100
    WAIT

    GOSUB stand_pose
    RETURN
    '**********************************************
keeper_left:

    HIGHSPEED SETON
    SPEED 7
    MOVE G6D,65, 85,  120, 110, 90, 100
    MOVE G6A,110, 143,  28, 142, 130, 100
    MOVE G6B,185,  100,  80
    MOVE G6C,185,  100,  80
    WAIT

    DELAY 2000
    HIGHSPEED SETOFF
    SPEED 10
    GOSUB sit_pose
    SPEED 8
    GOSUB stand_pose


    RETURN
    '**********************************************
keeper_right:

    HIGHSPEED SETON
    SPEED 7
    MOVE G6A,65, 85,  120, 110, 90, 100
    MOVE G6D,110, 143,  28, 142, 130, 100
    MOVE G6B,185,  100,  80
    MOVE G6C,185,  100,  80
    WAIT

    DELAY 2000
    HIGHSPEED SETOFF
    SPEED 10
    GOSUB sit_pose
    SPEED 8
    GOSUB stand_pose

    RETURN
    '**********************************************
keeper_front:

    HIGHSPEED SETON
    SPEED 7
    MOVE G6A,80, 143,  28, 142, 118, 100
    MOVE G6D,80, 143,  28, 142, 118, 100
    MOVE G6B,100,  60,  80, 100, 100, 100
    MOVE G6C,100,  60,  80, 100, 100, 100
    WAIT

    DELAY 2000
    HIGHSPEED SETOFF
    SPEED 8
    GOSUB stand_pose

    RETURN
    '**********************************************

    '**********************************************

drawing_right:
    GOSUB All_motor_mode3
    SPEED 7
    MOVE G6A,100,  76, 145,  95, 100
    MOVE G6D,100,  76, 145,  95, 100
    MOVE G6B,100,  185,  110
    MOVE G6C,100,  185,  110
    WAIT

    GOSUB GOSUB_RX_EXIT
drawing_right_1:
    ERX 9600, A, drawing_right_1
    '****************
    SPEED 4
    MOVE G6B,,  ,  135
    MOVE G6C,,  ,  135
    WAIT

    DELAY 500
    GOSUB GOSUB_RX_EXIT	

drawing_right_2:
    ERX 9600, A, drawing_right_2
    IF A = 26 THEN
        SPEED 10
        GOSUB stand_pose
        GOSUB All_motor_Reset	
        GOTO RX_EXIT
    ELSEIF A = 4 THEN
        GOTO drawing_left_3
    ELSEIF A = 5 THEN
        GOTO drawing_front_3
    ELSEIF A = 6 THEN
        GOTO drawing_right_3
    ENDIF
drawing_right_3:
    GOSUB Leg_motor_mode2
    SPEED 6
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    WAIT

    SPEED 8
    MOVE G6A,95,  46, 145,  123, 105, 100
    MOVE G6D,95,  106, 145,  63, 105, 100
    WAIT

    SPEED 10
    MOVE G6A,93,  46, 145,  123, 105, 100
    MOVE G6D,93,  106, 145,  63, 105, 100
    WAIT

    SPEED 8
    MOVE G6A,100,  76, 145,  93, 100
    MOVE G6D,100,  76, 145,  93, 100
    WAIT


    SPEED 6
    MOVE G6A,100,  96, 125,  85, 100
    MOVE G6D,100,  96, 125,  85, 100
    MOVE G6B,135,  ,
    MOVE G6C,135,  ,
    WAIT

    GOSUB All_motor_Reset
    HIGHSPEED SETON

    SPEED 12
    MOVE G6A,100,  66, 145,  115, 100
    MOVE G6D,100,  66, 145,  115, 100
    MOVE G6B,75,  , 125
    MOVE G6C,75,  , 125
    WAIT	

    DELAY 800
    HIGHSPEED SETOFF

    GOSUB All_motor_mode3
    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset

    RETURN
    RETURN
    '**********************************************
drawing_left:
    GOSUB All_motor_mode3
    SPEED 7
    MOVE G6A,100,  76, 145,  95, 100
    MOVE G6D,100,  76, 145,  95, 100
    MOVE G6B,100,  185,  110
    MOVE G6C,100,  185,  110
    WAIT

    GOSUB GOSUB_RX_EXIT
drawing_left_1:
    ERX 9600, A, drawing_left_1
    '****************
    SPEED 4
    MOVE G6B,,  ,  135
    MOVE G6C,,  ,  135
    WAIT

    DELAY 500
    GOSUB GOSUB_RX_EXIT	

drawing_left_2:
    ERX 9600, A, drawing_left_2
    IF A = 26 THEN
        SPEED 10
        GOSUB stand_pose
        GOSUB All_motor_Reset	
        GOTO RX_EXIT
    ELSEIF A = 4 THEN
        GOTO drawing_left_3
    ELSEIF A = 5 THEN
        GOTO drawing_front_3
    ELSEIF A = 6 THEN
        GOTO drawing_right_3
    ENDIF
drawing_left_3:
    GOSUB Leg_motor_mode2
    SPEED 6
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6A,100,  76, 145,  93, 100
    WAIT

    SPEED 8
    MOVE G6D,95,  46, 145,  123, 105, 100
    MOVE G6A,95,  106, 145,  63, 105, 100
    WAIT

    SPEED 10
    MOVE G6D,93,  46, 145,  123, 105, 100
    MOVE G6A,93,  106, 145,  63, 105, 100
    WAIT

    SPEED 8
    MOVE G6D,100,  76, 145,  93, 100
    MOVE G6A,100,  76, 145,  93, 100
    WAIT


    SPEED 6
    MOVE G6A,100,  96, 125,  85, 100
    MOVE G6D,100,  96, 125,  85, 100
    MOVE G6B,135,  ,
    MOVE G6C,135,  ,
    WAIT	
    GOSUB All_motor_Reset
    HIGHSPEED SETON

    SPEED 12
    MOVE G6A,100,  66, 145,  115, 100
    MOVE G6D,100,  66, 145,  115, 100
    MOVE G6B,75,  , 125
    MOVE G6C,75,  , 125
    WAIT	

    DELAY 800
    HIGHSPEED SETOFF

    GOSUB All_motor_mode3
    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset

    RETURN
    RETURN
    '**********************************************
drawing_front:
    GOSUB All_motor_mode3
    SPEED 7
    MOVE G6A,100,  76, 145,  95, 100
    MOVE G6D,100,  76, 145,  95, 100
    MOVE G6B,100,  185,  110
    MOVE G6C,100,  185,  110
    WAIT

    GOSUB GOSUB_RX_EXIT
drawing_front_1:
    ERX 9600, A, drawing_front_1
    '****************
    SPEED 4
    MOVE G6B,,  ,  135
    MOVE G6C,,  ,  135
    WAIT

    DELAY 500
    GOSUB GOSUB_RX_EXIT	

drawing_front_2:
    ERX 9600, A, drawing_front_2
    IF A = 26 THEN
        SPEED 10
        GOSUB stand_pose
        GOSUB All_motor_Reset	
        GOTO RX_EXIT
    ELSEIF A = 4 THEN
        GOTO drawing_left_3
    ELSEIF A = 5 THEN
        GOTO drawing_front_3
    ELSEIF A = 6 THEN
        GOTO drawing_right_3
    ENDIF
drawing_front_3:
    SPEED 5
    MOVE G6A,100,  96, 125,  85, 100
    MOVE G6D,100,  96, 125,  85, 100
    MOVE G6B,135,  ,
    MOVE G6C,135,  ,
    WAIT

    GOSUB All_motor_Reset
    HIGHSPEED SETON

    SPEED 12
    MOVE G6A,100,  66, 145,  115, 100
    MOVE G6D,100,  66, 145,  115, 100
    MOVE G6B,75,  , 125
    MOVE G6C,75,  , 125
    WAIT	

    DELAY 800
    HIGHSPEED SETOFF

    GOSUB All_motor_mode3
    SPEED 8
    GOSUB stand_pose
    GOSUB All_motor_Reset

    RETURN
    '**********************************************
right_shoot:
    GOSUB Leg_motor_mode3
    SPEED 4
    MOVE G6D,  88,  71, 152,  91, 110
    MOVE G6A, 108,  76, 146,  93,  94
    MOVE G6C, 100,40
    MOVE G6B, 100,40
    WAIT

    GOSUB Leg_motor_mode1
    HIGHSPEED SETON
    SPEED 8
    MOVE G6D, 80, 95, 115, 105, 140
    MOVE G6A,113,  78, 146,  93,  94
    MOVE G6C, 100,60
    MOVE G6B, 100,60
    WAIT

    DELAY 100
    HIGHSPEED SETOFF
    GOSUB Leg_motor_mode3
    SPEED 8
    MOVE G6A, 106,  76, 146,  93,  96		
    MOVE G6D,  88,  71, 152,  91, 106
    MOVE G6B, 100,35
    MOVE G6C, 100,35
    WAIT	

    GOSUB Leg_motor_mode3
    SPEED 3
    GOSUB stand_pose
    GOSUB Leg_motor_mode1


    RETURN
    '**********************************************
left_shoot:
    GOSUB Leg_motor_mode3
    SPEED 4
    MOVE G6A,  88,  71, 152,  91, 110
    MOVE G6D, 108,  76, 146,  93,  94
    MOVE G6B, 100,40
    MOVE G6C, 100,40
    WAIT

    GOSUB Leg_motor_mode1
    HIGHSPEED SETON

    SPEED 8
    MOVE G6A, 80, 95, 115, 105, 140
    MOVE G6D,113,  78, 146,  93,  94
    MOVE G6B, 100,60
    MOVE G6C, 100,60
    WAIT

    DELAY 100
    HIGHSPEED SETOFF
    GOSUB Leg_motor_mode3

    SPEED 8
    MOVE G6D, 106,  76, 146,  93,  96		
    MOVE G6A,  88,  71, 152,  91, 106
    MOVE G6B, 100,35
    MOVE G6C, 100,35
    WAIT	

    GOSUB Leg_motor_mode3
    SPEED 3
    GOSUB stand_pose
    GOSUB Leg_motor_mode1

    RETURN

    '**********************************************
back_shoot:
    IF walk_order = 0 THEN
        GOSUB Leg_motor_mode2
        SPEED 4

        MOVE G6A,110,  77, 145,  93,  92, 100	
        MOVE G6D, 85,  71, 152,  91, 114, 100
        MOVE G6C,100,  40,  80, , , ,
        MOVE G6B,100,  40,  80, , , ,	
        WAIT

        SPEED 10
        MOVE G6A,114,  78, 147,  85,  95	
        MOVE G6D, 83,  85, 122,  100, 114
        WAIT


        HIGHSPEED SETON

        SPEED 15
        MOVE G6A,114,  75, 147,  110,  95	
        MOVE G6D, 83,  110, 122,  75, 114
        MOVE G6B,80
        MOVE G6C,120
        WAIT


        DELAY 100
        HIGHSPEED SETOFF
        GOSUB Leg_motor_mode3

        SPEED 10
        MOVE G6A,114,  74, 147,  97,  95
        MOVE G6D, 83,  85, 122,  105, 114
        MOVE G6C,100,  40,  80, , , ,
        MOVE G6B,100,  40,  80, , , ,	
        WAIT	

        SPEED 8
        MOVE G6A,113,  76, 147,  95,  95	
        MOVE G6D, 80,  76, 142,  95, 114
        MOVE G6C,100,  40,  80, , , ,
        MOVE G6B,100,  40,  80, , , ,
        WAIT	

        SPEED 8
        MOVE G6A,110,  77, 147,  93,  93, 100	
        MOVE G6D, 80,  71, 154,  91, 114, 100
        WAIT


        SPEED 3
        GOSUB stand_pose	
        GOSUB Leg_motor_mode1
        walk_order = 1
    ELSE
        GOSUB Leg_motor_mode2
        SPEED 4

        MOVE G6D,110,  77, 145,  93,  92, 100	
        MOVE G6A, 85,  71, 152,  91, 114, 100
        MOVE G6B,100,  40,  80, , , ,
        MOVE G6C,100,  40,  80, , , ,	
        WAIT

        SPEED 10
        MOVE G6D,114,  78, 147,  85,  95	
        MOVE G6A, 83,  85, 122,  100, 114
        WAIT


        HIGHSPEED SETON

        SPEED 15
        MOVE G6D,114,  75, 147,  110,  95	
        MOVE G6A, 83,  110, 122,  75, 114
        MOVE G6C,80
        MOVE G6B,120
        WAIT


        DELAY 100
        HIGHSPEED SETOFF
        GOSUB Leg_motor_mode3

        SPEED 10
        MOVE G6D,114,  74, 147,  97,  95
        MOVE G6A, 83,  85, 122,  105, 114
        MOVE G6B,100,  40,  80, , , ,
        MOVE G6C,100,  40,  80, , , ,	
        WAIT	

        SPEED 8
        MOVE G6D,113,  76, 147,  95,  95	
        MOVE G6A, 80,  76, 142,  95, 114
        MOVE G6B,100,  40,  80, , , ,
        MOVE G6C,100,  40,  80, , , ,
        WAIT	

        SPEED 8
        MOVE G6D,110,  77, 147,  93,  93, 100	
        MOVE G6A, 80,  71, 154,  91, 114, 100
        WAIT


        SPEED 3
        GOSUB stand_pose	
        GOSUB Leg_motor_mode1
        walk_order = 0
    ENDIF

    RETURN
    '**********************************************
    '**********************************************
    '**********************************************

motorONOFF_LED:
    IF motorONOFF = 1  THEN
        OUT 52,1
        DELAY 200
        OUT 52,0
        DELAY 200
    ENDIF
    RETURN
    '**********************************************
LOW_Voltage:

    B = AD(6)

    IF B < low_volt THEN
        GOSUB warning_music

    ENDIF

    RETURN
    '**********************************************
    '******************************************	
MAIN:


    'GOSUB LOW_Voltage
    GOSUB F_B_tilt_check
    GOSUB L_R_tilt_check


    'GOSUB motorONOFF_LED

    'A = REMOCON(1)


    ERX 9600,A,MAIN
    A = A - ID
    IF A > 32  THEN
        GOTO MAIN
    ENDIF

    ON A GOTO MAIN,KEY1,KEY2,KEY3,KEY4,KEY5,KEY6,KEY7,KEY8,KEY9,KEY10,KEY11,KEY12,KEY13,KEY14,KEY15,KEY16,KEY17,KEY18 ,KEY19,KEY20,KEY21,KEY22,KEY23,KEY24,KEY25,KEY26,KEY27,KEY28 ,KEY29,KEY30,KEY31,KEY32

    GOTO MAIN	
    '*******************************************
    '		MAIN
    '*******************************************

KEY1:

    IF pose = 0 THEN
        GOSUB front_walk_50 'keeper_left
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY2:
    IF pose = 0 THEN	
        GOSUB back_walk_50 'keeper_front
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY3:
    IF pose = 0 THEN
        GOSUB left_turn_10 'keeper_right
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY4:
    IF pose = 0 THEN
        GOSUB right_turn_10 'drawing_left
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY5:
    IF pose = 0 THEN
        GOSUB left_walk20 'drawing_front
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY6:
    IF pose = 0 THEN
        GOSUB right_walk20 'drawing_right
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY7:
    IF pose = 0 THEN
        GOTO left_ball_kick 'left_turn_10
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY8:
    IF pose = 0 THEN
        GOTO right_ball_kick 'front_walk_50
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY9:
    IF pose = 0 THEN
        GOSUB front_standup 'right_turn_10
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY10: '0
    IF pose = 0 THEN
        GOSUB  back_standup
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY11: ' ¡ã
    IF pose = 0 THEN
        GOTO front_narrow_walk
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY12: ' ¡å
    IF pose = 0 THEN
        GOTO slow_left_walk50'left_walk70
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY13: '¢º
    IF pose = 0 THEN
        GOTO slow_right_walk50'right_walk70
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY14: ' ¢¸
    IF pose = 0 THEN
        GOTO stand_pose
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY15: ' A
    IF pose = 0 THEN
        GOSUB one_step
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY16: ' POWER

    GOSUB MOTOR_OFF
    GOSUB GOSUB_RX_EXIT

    '*******************************************
KEY17: ' C
    IF pose = 0 THEN
        GOSUB shoot_out_left
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY18: ' E
    IF pose = 0 THEN
        GOTO shoot_out_right
    ENDIF

    GOTO RX_EXIT
    '*******************************************


KEY19: ' P2
    IF pose = 0 THEN
        GOTO left_shoot
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY20: ' B	
    IF pose = 0 THEN
        GOSUB right_shoot
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY21: ' ¡â
    IF pose = 0 THEN
        GOTO front_narrow_walk
    ENDIF
    
    GOTO RX_EXIT
    '*******************************************
KEY22: ' *	
    IF pose = 0 THEN
        GOTO left_turn_20
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY23: ' G
    IF pose = 0 THEN
        GOTO shoot_out_right	
    ENDIF
    
    GOTO RX_EXIT
    '*******************************************
KEY24: ' #
	IF pose = 0 THEN
        GOTO right_turn_20	
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY25: ' P1
    IF pose = 0 THEN
        GOTO left_turn_45
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY26: ' ¡á

    GOTO RX_EXIT
    '*******************************************
KEY27: ' D
    IF pose = 0 THEN
        GOSUB right_shoot
    ENDIF
        
    GOTO RX_EXIT
    '*******************************************
KEY28: ' ¢·
    IF pose = 0 THEN
        GOTO left_walk20
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY29: ' ¡à

    GOTO RX_EXIT
    '*******************************************
KEY30: ' ¢¹
    IF pose = 0 THEN
        GOTO left_shoot
    ENDIF

    GOTO RX_EXIT
    '*******************************************
KEY31: ' ¡ä
    IF pose = 0 THEN
        GOTO back_narrow_walk
    ENDIF

    GOTO RX_EXIT
    '*******************************************

KEY32: ' F

    IF pose = 0 THEN
        GOSUB back_shoot
    ENDIF

    GOTO RX_EXIT
    '*******************************************