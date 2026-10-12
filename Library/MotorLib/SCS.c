/*
 * SCS.c
 * 飞特SCS系列串行舵机协议程序
 * 日期: 2025.6.30
 * 作者: txl
 */

#include <stdlib.h>
#include "INST.h"
#include "SCS.h"

// 全局变量定义
static uint8_t Level = 1;     // 舵机返回等级，1=开启应答（默认写指令开启应答）
static uint8_t End = 0;       // 处理器大小端结构，0=小端存储格式（默认）
static uint8_t u8Status;      // 舵机状态字节
static uint8_t u8Error;       // 通信状态错误码

// 同步读相关全局变量
uint8_t syncReadRxPacketIndex;  // 同步读接收包索引
uint8_t syncReadRxPacketLen;    // 同步读接收包长度
uint8_t *syncReadRxPacket;      // 同步读接收包指针
uint8_t *syncReadRxBuff;        // 同步读接收缓冲区
uint16_t syncReadRxBuffLen;     // 同步读接收缓冲区实际长度
uint16_t syncReadRxBuffMax;     // 同步读接收缓冲区最大长度

/**
 * 功能：设置处理器字节序（大小端模式）
 * 参数：_End - 0=小端模式，1=大端模式
 */
void setEnd(uint8_t _End)
{
	End = _End;
}

/**
 * 功能：获取当前处理器字节序设置
 * 返回值：当前字节序模式（0=小端，1=大端）
 */
uint8_t getEnd(void)
{
	return End;
}

/**
 * 功能：设置舵机返回等级
 * 参数：_Level - 0=关闭应答，1=开启应答（默认）
 */
void setLevel(uint8_t _Level)
{
	Level = _Level;
}

/**
 * 功能：获取最近一次通信的舵机状态
 * 返回值：舵机状态字节
 */
int getState(void)
{
	return u8Status;
}

/**
 * 功能：获取最近一次通信的错误码
 * 返回值：错误码（0=无错误）
 */
int getLastError(void)
{
	return u8Error;
}

/**[工具函数]
 * 功能：将一个16位整数拆分为两个8位字节（主机到舵机数据格式转换）
 * 参数：DataL - 指向低字节的指针，DataH - 指向高字节的指针，Data - 要拆分的16位数据
 */
void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data)
{
	if(End){
		// 大端模式：高字节在前
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		// 小端模式：低字节在前（默认）
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

/**[工具函数]
 * 功能：将两个8位字节组合为一个16位整数（舵机到主机数据格式转换）
 * 参数：DataL - 低字节，DataH - 高字节
 * 返回值：组合后的16位整数
 */
int SCS2Host(uint8_t DataL, uint8_t DataH)
{
	int Data;
	if(End){
		// 大端模式：第一个字节是高字节
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		// 小端模式：第一个字节是低字节（默认）
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

/**
 * 功能：构建并发送数据包到舵机（底层发送函数）
 * 参数：ID - 舵机ID，MemAddr - 内存地址，nDat - 要写入的数据，nLen - 数据长度，Fun - 指令类型
 */
void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
	uint8_t i;
	uint8_t msgLen = 2;  // 默认消息长度（指令+校验和）
	uint8_t bBuf[6];
	uint8_t CheckSum = 0;
	
	// 构建包头
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	
	if(nDat){
		// 有数据的情况
		msgLen += nLen + 1;  // +1 为内存地址
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		writeSCS(bBuf, 6);
	}else{
		// 无数据的情况（如PING、RESET指令）
		bBuf[3] = msgLen;
		writeSCS(bBuf, 5);
	}
	
	// 计算校验和
	CheckSum = ID + msgLen + Fun + MemAddr;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		writeSCS(nDat, nLen);
	}
	CheckSum = ~CheckSum;
	writeSCS(&CheckSum, 1);
}

/**
 * 功能：普通写指令（立即执行）
 * 参数：ID - 舵机ID，MemAddr - 内存地址，nDat - 要写入的数据，nLen - 数据长度
 * 返回值：成功=1，失败=0
 */
int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();  // 清空接收缓冲区
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	wFlushSCS();  // 清空发送缓冲区
	return Ack(ID);  // 等待并检查应答
}

/**
 * 功能：异步写指令（写入寄存器，等待触发执行）
 * 参数：ID - 舵机ID，MemAddr - 内存地址，nDat - 要写入的数据，nLen - 数据长度
 * 返回值：成功=1，失败=0
 */
int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * 功能：异步写执行指令（触发所有异步写入的舵机执行）
 * 参数：ID - 舵机ID
 * 返回值：成功=1，失败=0
 */
int regAction(uint8_t ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
	wFlushSCS();
	return Ack(ID);
}

/**
 * 功能：同步写指令（同时控制多个舵机）
 * 参数：ID[] - 舵机ID数组，IDN - 舵机数量，MemAddr - 内存地址，nDat - 数据数组，nLen - 每个舵机的数据长度
 */
void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	uint8_t mesLen = ((nLen+1)*IDN+4);  // 计算消息总长度
	uint8_t Sum = 0;
	uint8_t bBuf[7];
	uint8_t i, j;
	
	// 构建包头
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;  // 广播ID
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	
	rFlushSCS();
	writeSCS(bBuf, 7);

	// 计算校验和（初始值）
	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;

	// 发送所有舵机的ID和数据
	for(i=0; i<IDN; i++){
		writeSCS(&ID[i], 1);
		writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	Sum = ~Sum;
	writeSCS(&Sum, 1);
	wFlushSCS();
}

/**
 * 功能：写入单字节数据
 * 参数：ID - 舵机ID，MemAddr - 内存地址，bDat - 要写入的字节数据
 * 返回值：成功=1，失败=0
 */
int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * 功能：写入双字节数据（16位）
 * 参数：ID - 舵机ID，MemAddr - 内存地址，wDat - 要写入的16位数据
 * 返回值：成功=1，失败=0
 */
int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
	uint8_t buf[2];
	Host2SCS(buf+0, buf+1, wDat);  // 转换为舵机字节序
	rFlushSCS();
	writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * 功能：读取指令
 * 参数：ID - 舵机ID，MemAddr - 内存地址，nData - 读取数据存储缓冲区，nLen - 要读取的数据长度
 * 返回值：实际读取的数据长度，0=失败
 */
int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
	int Size;
	uint8_t bBuf[4];
	uint8_t calSum;
	uint8_t i;
	
	rFlushSCS();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);  // nLen作为参数发送
	wFlushSCS();
	
	u8Error = 0;
	
	// 检查数据包头（0xFF 0xFF）
	if(!checkHead()){
		u8Error = SCS_ERR_NO_REPLY;
		return 0;
	}
	
	// 读取ID、长度和状态
	if(readSCS(bBuf, 3)!=3){
		u8Error = SCS_ERR_NO_REPLY;
		return 0;
	}
	
	// 验证ID
	if(bBuf[0]!=ID && ID!=0xfe){
		u8Error = SCS_ERR_SLAVE_ID;
		return 0;
	}
	
	// 验证长度
	if(bBuf[1]!=(nLen+2)){  // +2 包括状态字节和校验和
		u8Error = SCS_ERR_BUFF_LEN;
		return 0;
	}
	
	// 读取数据
	Size = readSCS(nData, nLen);
	if(Size!=nLen){
		u8Error = SCS_ERR_NO_REPLY;
		return 0;
	}
	
	// 读取校验和
	if(readSCS(bBuf+3, 1)!=1){
		u8Error = SCS_ERR_NO_REPLY;
		return 0;
	}
	
	// 计算校验和验证
	calSum = bBuf[0]+bBuf[1]+bBuf[2];
	for(i=0; i<Size; i++){
		calSum += nData[i];
	}
	calSum = ~calSum;
	if(calSum!=bBuf[3]){
		u8Error = SCS_ERR_CRC_CMP;
		return 0;
	}
	
	u8Status = bBuf[2];  // 保存状态字节
	return Size;
}

/**
 * 功能：读取单字节数据
 * 参数：ID - 舵机ID，MemAddr - 内存地址
 * 返回值：读取的数据，-1=读取失败
 */
int readByte(uint8_t ID, uint8_t MemAddr)
{
	uint8_t bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

/**
 * 功能：读取双字节数据（16位）
 * 参数：ID - 舵机ID，MemAddr - 内存地址
 * 返回值：读取的16位数据，-1=读取失败
 */
int readWord(uint8_t ID, uint8_t MemAddr)
{	
	uint8_t nDat[2];
	int Size;
	uint16_t wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);  // 转换为主机字节序
	return wDat;
}

/**
 * 功能：Ping指令（检测舵机在线状态）
 * 参数：ID - 舵机ID（0xFE为广播）
 * 返回值：应答的舵机ID，-1=无应答或错误
 */
int	Ping(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	wFlushSCS();
	
	u8Status = 0;
	if(!checkHead()){
		u8Error = SCS_ERR_NO_REPLY;
		return -1;
	}
	
	u8Error = 0;
	if(readSCS(bBuf, 4)!=4){
		u8Error = SCS_ERR_NO_REPLY;
		return -1;
	}
	
	if(bBuf[0]!=ID && ID!=0xfe){
		u8Error = SCS_ERR_SLAVE_ID;
		return -1;
	}
	
	if(bBuf[1]!=2){
		u8Error = SCS_ERR_BUFF_LEN;
		return -1;
	}
	
	calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		u8Error = SCS_ERR_CRC_CMP;
		return -1;			
	}
	
	u8Status = bBuf[2];
	return bBuf[0];  // 返回应答舵机ID
}

/**
 * 功能：Reset指令（重置舵机）
 * 参数：ID - 舵机ID
 * 返回值：应答的舵机ID，-1=无应答或错误
 */
int	Reset(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_RESET);
	wFlushSCS();
	
	u8Status = 0;
	if(!checkHead()){
		u8Error = SCS_ERR_NO_REPLY;
		return -1;
	}
	
	u8Error = 0;
	if(readSCS(bBuf, 4)!=4){
		u8Error = SCS_ERR_NO_REPLY;
		return -1;
	}
	
	if(bBuf[0]!=ID && ID!=0xfe){
		u8Error = SCS_ERR_SLAVE_ID;
		return -1;
	}
	
	if(bBuf[1]!=2){
		u8Error = SCS_ERR_BUFF_LEN;
		return -1;
	}
	
	calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		u8Error = SCS_ERR_CRC_CMP;
		return -1;			
	}
	
	u8Status = bBuf[2];
	return bBuf[0];
}

/**
 * 功能：检查并跳过数据包头（0xFF 0xFF）
 * 返回值：1=找到包头，0=未找到或超时
 */
int checkHead(void)
{
	uint8_t bDat;
	uint8_t bBuf[2] = {0, 0};
	uint8_t Cnt = 0;
	
	while(1){
		if(!readSCS(&bDat, 1)){
			return 0;  // 读取失败
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if(bBuf[0]==0xff && bBuf[1]==0xff){
			break;  // 找到包头
		}
		Cnt++;
		if(Cnt>10){
			return 0;  // 超时
		}
	}
	return 1;
}

/**
 * 功能：等待并处理指令应答
 * 参数：ID - 期望的舵机ID
 * 返回值：1=成功应答，0=应答失败或无应答
 */
int	Ack(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	u8Error = 0;
	
	// 如果ID不是广播且开启了应答等级
	if(ID!=0xfe && Level){
		if(!checkHead()){
			u8Error = SCS_ERR_NO_REPLY;
			return 0;
		}
		
		u8Status = 0;
		if(readSCS(bBuf, 4)!=4){
			u8Error = SCS_ERR_NO_REPLY;
			return 0;
		}
		
		if(bBuf[0]!=ID){
			u8Error = SCS_ERR_SLAVE_ID;
			return 0;
		}
		
		if(bBuf[1]!=2){
			u8Error = SCS_ERR_BUFF_LEN;
			return 0;
		}
		
		calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
		if(calSum!=bBuf[3]){
			u8Error = SCS_ERR_CRC_CMP;
			return 0;			
		}
		
		u8Status = bBuf[2];
	}
	return 1;
}

/**
 * 功能：发送同步读请求包
 * 参数：ID[] - 舵机ID数组，IDN - 舵机数量，MemAddr - 内存地址，nLen - 要读取的数据长度
 * 返回值：接收缓冲区的数据长度
 */
int	syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen)
{
	uint8_t checkSum;
	uint8_t i;
	
	rFlushSCS();
	syncReadRxPacketLen = nLen;  // 设置要读取的数据长度
	
	// 计算校验和
	checkSum = (4+0xfe)+IDN+MemAddr+nLen+INST_SYNC_READ;
	
	// 构建并发送数据包
	writeByteSCS(0xff);
	writeByteSCS(0xff);
	writeByteSCS(0xfe);  // 广播ID
	writeByteSCS(IDN+4);
	writeByteSCS(INST_SYNC_READ);
	writeByteSCS(MemAddr);
	writeByteSCS(nLen);
	
	for(i=0; i<IDN; i++){
		writeByteSCS(ID[i]);
		checkSum += ID[i];
	}
	checkSum = ~checkSum;
	writeByteSCS(checkSum);
	wFlushSCS();
	
	// 读取所有应答到缓冲区
	syncReadRxBuffLen = readSCS(syncReadRxBuff, syncReadRxBuffMax);
	return syncReadRxBuffLen;
}

/**
 * 功能：同步读开始（分配缓冲区）
 * 参数：IDN - 舵机数量，rxLen - 每个舵机返回的数据长度
 */
void syncReadBegin(uint8_t IDN, uint8_t rxLen)
{
	syncReadRxBuffMax = IDN*(rxLen+6);  // 每个舵机：包头2+ID1+长度1+状态1+数据+校验1
	syncReadRxBuff = malloc(syncReadRxBuffMax);  // 动态分配缓冲区
}

/**
 * 功能：同步读结束（释放缓冲区）
 */
void syncReadEnd(void)
{
	if(syncReadRxBuff){
		free(syncReadRxBuff);
		syncReadRxBuff = NULL;
	}
}

/**
 * 功能：从同步读缓冲区中解析指定舵机的数据包
 * 参数：ID - 要解析的舵机ID，nDat - 数据存储缓冲区
 * 返回值：实际解析的数据长度，0=失败
 */
int syncReadPacketRx(uint8_t ID, uint8_t *nDat)
{
	uint16_t syncReadRxBuffIndex = 0;
	syncReadRxPacket = nDat;
	syncReadRxPacketIndex = 0;
	u8Status = 0;
	
	// 遍历接收缓冲区
	while((syncReadRxBuffIndex+6+syncReadRxPacketLen)<=syncReadRxBuffLen){
		uint8_t bBuf[] = {0, 0, 0};
		uint8_t calSum = 0;
		
		// 寻找数据包头
		while(syncReadRxBuffIndex<syncReadRxBuffLen){
			bBuf[0] = bBuf[1];
			bBuf[1] = bBuf[2];
			bBuf[2] = syncReadRxBuff[syncReadRxBuffIndex++];
			if(bBuf[0]==0xff && bBuf[1]==0xff && bBuf[2]!=0xff){
				u8Error = SCS_ERR_NO_REPLY;
				break;
			}
		}
		
		// 检查ID是否匹配
		if(bBuf[2]!=ID){
			u8Error = SCS_ERR_SLAVE_ID;
			continue;
		}
		
		// 检查数据长度
		if(syncReadRxBuff[syncReadRxBuffIndex++]!=(syncReadRxPacketLen+2)){
			continue;
		}
		
		// 读取状态字节
		u8Status = syncReadRxBuff[syncReadRxBuffIndex++];
		calSum = ID+(syncReadRxPacketLen+2)+u8Status;
		
		// 读取数据
		for(uint8_t i=0; i<syncReadRxPacketLen; i++){
			syncReadRxPacket[i] = syncReadRxBuff[syncReadRxBuffIndex++];
			calSum += syncReadRxPacket[i];
		}
		
		// 验证校验和
		calSum = ~calSum;
		if(calSum!=syncReadRxBuff[syncReadRxBuffIndex++]){
			u8Error = SCS_ERR_CRC_CMP;
			return 0;
		}
		
		return syncReadRxPacketLen;  // 成功解析
	}
	return 0;  // 未找到匹配的数据包
}

/**
 * 功能：从同步读包中读取一个字节
 * 返回值：读取的字节值，-1=无数据
 */
int syncReadRxPacketToByte(void)
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

/**
 * 功能：从同步读包中读取一个字（16位），可处理负数表示
 * 参数：negBit - 负数标志位（如15表示第15位为符号位）
 * 返回值：读取的16位数据（已转换为有符号整数），-1=无数据
 */
int syncReadRxPacketToWrod(uint8_t negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		return -1;
	}
	
	// 将两个字节组合为16位整数
	int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	
	// 处理负数表示（如果指定了符号位）
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));  // 取反并加上负号
		}
	}
	return Word;
}
