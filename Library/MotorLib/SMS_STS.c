/*
 * SMS_STS.c
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2025.3.3
 * 作者: 
 */

#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SMS_STS.h"


/* 孩子们，HLS这个系列舵机的设计师家里得请高人了，0x2c这个目标扭矩，至今没有看到他具体的作用是什么。
   位置控制中，真实输出的扭矩压根没有跟踪这个值，但是按照飞特官方库函数（下面这个）
   Host2SCS(bBuf+3, bBuf+4, 0);这个代码直接将目标扭矩设为0。而我们用飞特上位机调试可以知道，目标扭矩虽然没啥用但是不能为0，为0则舵机没有扭矩输出！ */
/* 二补，现在飞特出了HLS.c的库修正了这个问题 */

/**
 * 功能：控制单个舵机以指定加速度和速度运动到指定位置（立即执行）
 * 参数：ID-舵机ID，Position-目标位置，Speed-运动速度，ACC-加速度
 * 说明：位置值支持负数表示反向，通过最高位标识方向
 */
int WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}



/**
 * 功能：将单个舵机的扭矩使能/失能
 * 参数：ID-舵机ID
 * 说明：启动该舵机的扭矩/让舵机卸力
 */
int EnableTorqueSTS(uint8_t ID, uint8_t ifEnable)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, ifEnable);
}


/**
 * 功能：设置单个舵机扭矩限制值（最大100.0%）
 * 参数：ID-舵机ID，torque_limit-最大扭矩的百分比(0-1000)
 * 说明：将舵机从速度控制模式切换到位置模式
 */
int SetTorqueSTS(uint8_t ID, uint16_t torque_limit)
{
	return writeWord(ID, SMS_STS_TORQUE_LIMIT_L, torque_limit);
}



/**
 * 功能：设置单个舵机为关节模式（有角度限制）
 * 参数：ID-舵机ID
 * 说明：将舵机从速度控制模式切换到位置模式
 */
int JointMode(uint8_t ID)
{
	return writeByte(ID, SMS_STS_MODE, 0);		
}


/**[核心函数]
 * 功能：设置单个舵机为轮式模式（连续旋转模式）
 * 参数：ID-舵机ID
 * 说明：将舵机从位置模式切换到速度控制模式
 */
int WheelMode(uint8_t ID)
{
	return writeByte(ID, SMS_STS_MODE, 1);		
}



/**
 * 功能：在轮式模式下控制舵机以指定加速度和速度连续旋转
 * 参数：ID-舵机ID，Speed-旋转速度，ACC-加速度
 * 说明：仅当舵机处于轮式模式时有效，速度值决定方向和转速
 */
int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, 0);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}







//====================================用不上===========================================

/**
 * 功能：控制单个舵机以指定加速度和速度运动到指定位置（寄存器写入，等待触发执行）
 * 参数：ID-舵机ID，Position-目标位置，Speed-运动速度，ACC-加速度
 * 说明：使用regWrite向寄存器写入指令，需要配合Action指令才能执行
 */
int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

/**[核心函数]
 * 功能：同步控制多个舵机以各自的加速度和速度运动到各自的位置
 * 参数：ID[]-舵机ID数组，IDN-舵机数量，Position[]-位置数组，Speed[]-速度数组，ACC[]-加速度数组
 * 说明：最多支持32个舵机同步运动，速度/加速度数组为NULL时使用默认值0
 */
void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
{
	uint8_t offbuf[32*7];
	uint8_t i;
	uint16_t V;
  for(i = 0; i<IDN; i++){
		if(Position[i]<0){
			Position[i] = -Position[i];
			Position[i] |= (1<<15);
		}

		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
		if(ACC){
			offbuf[i*7] = ACC[i];
		}else{
			offbuf[i*7] = 0;
		}
		Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
    Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
    Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
	}
  syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

/**[校准函数]
 * 功能：校准舵机偏移量
 * 参数：ID-舵机ID
 * 说明：将当前位置设置为想要的角度，上电后调用，需要先获取舵机实际位置
 */
int CalibrationOfs(uint8_t ID, uint16_t offset)
{
	return writeWord(ID, SMS_STS_OFS_L, offset);
}



/** 辅助函数
  * @brief  计算将当前实际位置显示为指定角度所需的偏移值
  * @param  current_value: 当前舵机实际位置值 (0-4095)
  * @param  target_angle: 想要显示的角度值 (0-359.99度)
  * @retval 偏移寄存器的值 (0-4095)
  * @note   根据公式：显示值 = (实际值 - 偏移) mod 4096
  *         推导：偏移 = 实际值 - 期望显示值 (mod 4096)

			当前角度-当前角度+1707=目标示数1707,又有当前角度-偏移=1707
  */
uint16_t Calculate_Offset_Angle(uint16_t current_value, uint16_t target_angle)
{
    // 确保目标显示值在0-4095范围内
    target_angle %= 4096;
    
    // 计算偏移值：偏移 = 实际值 - 目标显示值 (mod 4096)
    int32_t offset = (int32_t)current_value - (int32_t)target_angle;
    
    // 将偏移值归一化到0-4095范围内
    // 如果offset为负数，加4096使其为正
    offset = (offset % 4096 + 4096) % 4096;
    
    return (uint16_t)offset;
}



/**
 * 功能：设置舵机角度值为0-4095
 * 参数：ID-舵机ID
 * 说明：上电后调用，设置舵机角度最大为4095，使用多圈编码器会出现不正常控制情况
 */
int SetMaxAngleLimit(uint8_t ID)
{
	return writeWord(ID, SMS_STS_MAX_ANGLE_LIMIT_L, 4095);
}


/**
 * 功能：解锁舵机EEPROM，允许写入参数
 * 参数：ID-舵机ID
 * 说明：解锁后可以对EEPROM区域进行写操作
 */
int unLockEpromEx(uint8_t ID)
{
	return writeByte(ID, SMS_STS_LOCK, 0);
}

/**
 * 功能：锁定舵机EEPROM，保护参数不被修改
 * 参数：ID-舵机ID
 * 说明：锁定后EEPROM区域变为只读，防止意外修改参数
		 不能修改：舵机ID、波特率、角度限制、保护模式等配置参数
		 可以修改：目标位置、速度、加速度等实时控制参数
 */
int LockEpromEx(uint8_t ID)
{
	return writeByte(ID, SMS_STS_LOCK, 1);
}



