#include "cmd.h"
#include "I2CMain.h"

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define HEXA 0x100

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C (Master) Callback Function									                                   */
/*---------------------------------------------------------------------------------------------------------*/


int I2C_MasterSendData(struct i2c_client *client)
{
	int result;
	struct epack_dev_data *epack = i2c_get_clientdata(client);
	struct i2c_msg msgs = {

	.addr = client->addr,
	.flags = client->flags,
	.len = 64,
	.buf = (u8 *)(epack->sendbuf),
	};

	if(epack == NULL)
	{
		epack_log("%s: epack struct is null\n", __func__);
		return -1;
	}

	epack_log("%s sendbuf : \n", __func__);
	print_array(epack->sendbuf,PACKET_SIZE);

	result = i2c_transfer(client->adapter, &msgs, 1);

	return result;
}

int I2C_MasterRcvData(struct i2c_client *client)
{
	int result;
	struct epack_dev_data *epack = i2c_get_clientdata(client);
	struct i2c_msg msgs = {

	.addr = client->addr,
	.flags = client->flags | I2C_M_RD,
	.len = 64,
	.buf = epack->rcvbuf,
	};

	if(epack == NULL)
	{
		epack_log("%s: epack struct is null\n", __func__);
		return -1;
	}

	result = i2c_transfer(client->adapter, &msgs, 1);

	epack_log("%s rcvbuf : \n", __func__);
	print_array(epack->rcvbuf,PACKET_SIZE);

	return result;
}


bool send_data(struct i2c_client *client)
{
	int result;
	struct epack_dev_data *epack = i2c_get_clientdata(client);

	if(epack == NULL)
	{
		epack_log("%s: epack struct is null\n", __func__);
		return 0;
	}
	gcksum = check_sum(epack->sendbuf, PACKET_SIZE);

	result = I2C_MasterSendData(client);

	if(result <= 0)
	{
		epack_log("%s  result= %d exit.", __func__, result);
		return 0;
	}

	return result;
}

bool RcvData(struct i2c_client *client)
{
	int result;
	unsigned short lcksum;
	int rcv_g_packno;
	u8 *buf;
	struct epack_dev_data *epack = i2c_get_clientdata(client);
	if(epack == NULL)
	{
		epack_log("%s: epack struct is null\n", __func__);
		return 0;
	}
	msleep(50);//50ms

	result = I2C_MasterRcvData(client);

	if(result <= 0)
	{
		epack_log("%s  result= %d exit.",__func__,  result);
		return 0;
	}

	buf = epack->rcvbuf;
	memcpy(&lcksum, buf, 2);
	buf += 4;

	rcv_g_packno = buf[0]+ (HEXA)*buf[1] + (HEXA)*(HEXA)*buf[2] + (HEXA)*(HEXA)*(HEXA)*buf[3];
	epack_log("%s   buf[0]: %x buf[1]: %x buf[2]: %x buf[3]: %x\n", __func__,buf[0],buf[1],buf[2],buf[3]);

	//if((unsigned short)(buf[0]) != g_packno)
	if(rcv_g_packno != g_packno)
	{
		epack_log("%s g_packno=%d rcv %d\n", __func__, g_packno, rcv_g_packno );
		result = 0;
	}
	else
	{
		if(lcksum != gcksum)
		{
			epack_log("%s gcksum=%x lcksum=%x\n", __func__, gcksum, lcksum);
			result = 0;
		}
		g_packno++;
	}
	return result;
}

int epack_i2c_send(struct i2c_client *client,uint8_t* sendbuf)
{
	struct i2c_msg msgs = {
	.addr = client->addr,
	.flags = client->flags,
	.len = 64,
	.buf = sendbuf,
	};

	return i2c_transfer(client->adapter, &msgs, 1);
}

int epack_i2c_receive(struct i2c_client *client,uint8_t* rcvbuf)
{
	struct i2c_msg msgs = {
	.addr = client->addr,
	.flags = client->flags | I2C_M_RD,
	.len = 64,
	.buf = rcvbuf,
	};

	return i2c_transfer(client->adapter, &msgs, 1);
}


void epack_create_i2c_packet(unsigned long cmdData,int8_t *msg_buf){
	unsigned long cmd = cmdData;
	unsigned int g_packno = SEND_G_PACKNO;

	memset(msg_buf,0, PACKET_SIZE);
	memcpy(msg_buf+0, &cmd, 4);
	memcpy(msg_buf+4, &g_packno, 4);
}


bool epack_i2c_comm(bool is_writing,struct epack_dev_data *epack
				,unsigned long cmdData,void *value,int length)
{
	struct i2c_client *client = epack->client;
	u8 msg_buf[PACKET_SIZE];
	unsigned short gcksum,lcksum;
	int g_packno,rc;

	/*0. create a I2C msg to be sent */
	epack_create_i2c_packet(cmdData,msg_buf);
	if (is_writing)
		memcpy(msg_buf+8, value, length);

	/*1. send i2c msg to PlusPack*/
	mutex_lock(&epack->i2c_lock);
	rc = epack_i2c_send(client,msg_buf);
	if(rc <= 0) {
		epack_log("%s :Failed to write I2C for CMD %lu \n", __func__,cmdData);
		mutex_unlock(&epack->i2c_lock);
		return 0;
	}
	/*2. calculate checksum of sent msg*/
	gcksum = check_sum(msg_buf, PACKET_SIZE);
	msleep(50);

	/*3. recevie i2c ACK msg from PlusPack*/
	rc = epack_i2c_receive(client,msg_buf);
	mutex_unlock(&epack->i2c_lock);
	if(rc <= 0) {
		epack_log("%s :Failed to read I2C for CMD %lu \n", __func__,cmdData);
		return 0;
	}

	/*4. validate i2c comm. on comparing cksum */
	memcpy(&lcksum, msg_buf, 2);
	if(gcksum != lcksum) {
		epack_log("%s :gcksum=%x lcksum=%x\n", __func__, gcksum, lcksum);
		return 0;
	}

	/*5. validate i2c comm. on checking g_packno */
	g_packno = msg_buf[4]+ (HEXA)*msg_buf[5] + (HEXA)*(HEXA)*msg_buf[6] + (HEXA)*(HEXA)*(HEXA)*msg_buf[7];
	if(g_packno != RCV_G_PACKNO) {
		epack_log("%s :rcv_g_packno=%d\n", __func__, g_packno);
		return 0;
	}

	/*6-1 for writing case, func exits after 3rd validation step */
	if (is_writing) {
		if (*(u8*)value == msg_buf[VAILD_CHECK_BYTE_8])
			return 1;
		epack_log("%s : final validation fails (%d)\n"
				, __func__,(int)msg_buf[VAILD_CHECK_BYTE_8]);
		return 0;
	}

	/*6-2 for reading case, func extis after mem copy to the specific address */
	memcpy(value,msg_buf+VAILD_CHECK_BYTE_8,length);
	return rc;
}


bool epack_i2c_write_byte(struct epack_dev_data *epack,unsigned long cmdData,void *value)
{
	return epack_i2c_comm(1,epack,cmdData,value,1);
}

bool epack_i2c_read_bytes(struct epack_dev_data *epack,unsigned long cmdData,void *value,int length)
{
	return epack_i2c_comm(0,epack,cmdData,value,length);
}
