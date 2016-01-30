#ifndef REG_MAP_H
#define REG_MAP_H

/////////////////////////////////////////
#define REG_STATE 	        0
#define REG_COMMAND 	    	1
/////////////////////////////////////////
#define REG_THRUST	        2
#define REG_PITCH	        3
#define REG_ROLL	        4
#define REG_YAW		        5
////////////////////////////////////////
#define REG_PID_X_PH		6//-
#define REG_PID_X_PL		7//-
#define REG_PID_X_IH		8//-- PID X
#define REG_PID_X_IL		9//-- PID X
#define REG_PID_X_DH		10//-
#define REG_PID_X_DL		11//-
#define REG_PID_X_UPDATE    	12
/////////////////////////////////////////
#define REG_PID_Y_PH		13//-
#define REG_PID_Y_PL		14//-
#define REG_PID_Y_IH		15//-- PID Y
#define REG_PID_Y_IL		16//-- PID Y
#define REG_PID_Y_DH		17//-
#define REG_PID_Y_DL		18//-
#define REG_PID_Y_UPDATE    	19
/////////////////////////////////////////
#define REG_PID_Z_PH		20//-
#define REG_PID_Z_PL		21//-
#define REG_PID_Z_IH		22//-- PID Z
#define REG_PID_Z_IL		23//-- PID Z
#define REG_PID_Z_DH		24//-
#define REG_PID_Z_DL		25//-
#define REG_PID_Z_UPDATE    	26
/////////////////////////////////////////
#define REG_GYRO_XH		27
#define REG_GYRO_XL		28
#define REG_GYRO_YH		29
#define REG_GYRO_YL		30
#define REG_GYRO_ZH		31
#define REG_GYRO_ZL		32

#define REG_PID_XH		33
#define REG_PID_XL		34
#define REG_PID_YH		35
#define REG_PID_YL		36
#define REG_PID_ZH		37
#define REG_PID_ZL		38

#define REG_LOG_ENABLE		39
/////////////////////////////////////////
#define REG_FL_PWM		40
#define REG_FR_PWM		41
#define REG_BL_PWM		42
#define REG_BR_PWM		43
#define REG_PWM_UPDATE		44
#define REG_MOTOR_ENABLE	45
////////////////////////////////////////
#define REG_GX_OFFSET		46
#define REG_GY_OFFSET		47
#define REG_GZ_OFFSET		48

#define __REG_END__         	49
#define REG_COUNT __REG_END__

#endif // REG_MAP_H

