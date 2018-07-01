/* qmcX983.c - qmcX983 compass driver
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <cust_mag.h>
#include "qmcX983.h"
#include "mag.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*----------------------------------------------------------------------------*/
#define DEBUG 1
#define QMCX983_DEV_NAME         "qmcX983"
#define DRIVER_VERSION          "driver version 3.4"
/*----------------------------------------------------------------------------*/

#define	QMCX983_BUFSIZE		32

#define QMCX983_AXIS_X            0
#define QMCX983_AXIS_Y            1
#define QMCX983_AXIS_Z            2
#define QMCX983_AXES_NUM          3

#define QMCX983_DEFAULT_DELAY 100


#define QMC6983_A1_D1             0
#define QMC6983_E1		  		  1	
#define QMC6983_E1_Metal          2
#define QMC7983_Vertical          3
#define QMC7983_Slope             4




#define CALIBRATION_DATA_SIZE   28

#define MSE_TAG					"[QMC-Msensor] "
#define MSE_FUN(f)				pr_info(MSE_TAG"%s\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)	pr_err(MSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)	pr_info(MSE_TAG fmt, ##args)


static int chip_id = QMC6983_E1;
static struct i2c_client *this_client = NULL;
static short qmcd_delay = QMCX983_DEFAULT_DELAY;

//static int sensor_data[CALIBRATION_DATA_SIZE] = {0};
static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;

static unsigned char regbuf[2] = {0};
static int OTP_Kx;
static int OTP_Ky;



static atomic_t open_flag = ATOMIC_INIT(0);
static unsigned char v_open_flag = 0x00;

//static int g_rawMag[3] = {0};
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static const struct i2c_device_id qmcX983_i2c_id[] = {{QMCX983_DEV_NAME,0},{}};

static struct mag_hw mag_cust;
static struct mag_hw *qst_hw = &mag_cust;

/* For  driver get cust info */
struct mag_hw *get_cust_mag(void)
{
	return &mag_cust;
}

/*----------------------------------------------------------------------------*/
static int qmcX983_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int qmcX983_i2c_remove(struct i2c_client *client);
static int qmcX983_suspend(struct device *dev);
static int qmcX983_resume(struct device *dev);
static int qmcX983_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int qmcX983_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs);
static int qmcX983_flush(void);
static int qmcX983_m_get_data(int *x, int *y, int *z, int *status);
//static int qmcX983_remove(void);


#ifdef QST_Dummygyro
	static atomic_t g_flag = ATOMIC_INIT(0);
#ifdef QST_Dummygyro_VirtualSensors
	static atomic_t gr_flag = ATOMIC_INIT(0);
	static atomic_t la_flag = ATOMIC_INIT(0);
	static atomic_t rv_flag = ATOMIC_INIT(0);
#endif
#endif
DECLARE_COMPLETION(data_updated);

/*----------------------------------------------------------------------------*/
typedef enum {
    QMC_FUN_DEBUG  = 0x01,
	QMC_DATA_DEBUG = 0x02,
	QMC_HWM_DEBUG  = 0x04,
	QMC_CTR_DEBUG  = 0x08,
	QMC_I2C_DEBUG  = 0x10,
} QMC_TRC;

/*----------------------------------------------------------------------------*/
struct qmcX983_i2c_data {
    struct i2c_client *client;
    struct mag_hw hw;
    atomic_t layout;
    atomic_t trace;
	struct hwmsen_convert   cvt;
	short xy_sensitivity;
	short z_sensitivity;
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

#define DATA_AVG_DELAY 6
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
    { .compatible = "mediatek,msensor_qmcX983", },
    {},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops QMCX983_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(qmcX983_suspend, qmcX983_resume)
};
#endif

static struct i2c_driver qmcX983_i2c_driver = {
    .driver = {
        .name  = QMCX983_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm    = &QMCX983_pm_ops,
#endif
#ifdef CONFIG_OF
	.of_match_table = mag_of_match,
#endif
    },
	.probe      = qmcX983_i2c_probe,
	.remove     = qmcX983_i2c_remove,
	.detect 	= qmcX983_i2c_detect,
	.id_table   = qmcX983_i2c_id,
};


static int qmcX983_local_init(void);
static int qmcX983_local_remove(void);
static int qmcX983_init_flag =-1; // 0<==>OK -1 <==> fail


static struct mag_init_info qmcX983_init_info = {
        .name = QMCX983_DEV_NAME,
        .init = qmcX983_local_init,
        .uninit = qmcX983_local_remove,
};



static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&read_i2c_xyz);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&read_i2c_xyz);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		MSE_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&read_i2c_xyz);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{	
	/*because address also occupies one byte, the maximum length for write is 7 bytes */
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&read_i2c_xyz);
	if (!client) {
		mutex_unlock(&read_i2c_xyz);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&read_i2c_xyz);
	return err;
}


static int I2C_RxData(char *rxData, int length)
{
	

	struct i2c_client *client = this_client;
	int res = 0;
	char addr = rxData[0];

	if ((rxData == NULL) || (length < 1))
		return -EINVAL;
	res = mag_i2c_read_block(client, addr, rxData, length);
	if (res < 0)
		return -1;
	return 0;

}

static int I2C_TxData(char *txData, int length)
{	

	struct i2c_client *client = this_client;
	int res = 0;
	char addr = txData[0];
	u8 *buff = &txData[1];

	if ((txData == NULL) || (length < 2))
		return -EINVAL;
	res = mag_i2c_write_block(client, addr, buff, (length - 1));
	if (res < 0)
		return -1;
	return 0;

}


/* X,Y and Z-axis magnetometer data readout
 * param *mag pointer to \ref QMCX983_t structure for x,y,z data readout
 * note data will be read by multi-byte protocol into a 6 byte structure
 */

static int qmcX983_read_mag_xyz(int *data)
{
	int res;
	unsigned char mag_data[6];
	unsigned char databuf[6];
	int hw_d[3] = {0};

	int t1 = 0;
	unsigned char rdy = 0;
	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *clientdata = i2c_get_clientdata(client);

    MSE_FUN();

	/* Check status register for data availability */
	while(!(rdy & 0x07) && (t1<3)){
		databuf[0] = STA_REG_ONE;
		res = I2C_RxData(databuf,1);
		rdy = databuf[0];
		MSE_LOG("QMCX983 Status register is (%02X)\n", rdy);
		t1 ++;
	}

	//MSE_LOG("QMCX983 read mag_xyz begin\n");

	//mutex_lock(&read_i2c_xyz);

	databuf[0] = OUT_X_L;

	res = I2C_RxData(databuf, 6);
	if(res != 0)
    {
		//mutex_unlock(&read_i2c_xyz);
		return -EFAULT;
	}

	memcpy(mag_data,databuf,sizeof(databuf));
	
	/*MSE_LOG("QMCX983 mag_data[%02x, %02x, %02x, %02x, %02x, %02x]\n",
		mag_data[0], mag_data[1], mag_data[2],
		mag_data[3], mag_data[4], mag_data[5]);*/

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);

	hw_d[0] = hw_d[0] * 1000 / clientdata->xy_sensitivity;
	hw_d[1] = hw_d[1] * 1000 / clientdata->xy_sensitivity;
	hw_d[2] = hw_d[2] * 1000 / clientdata->z_sensitivity;

	//MSE_LOG("Hx=%d, Hy=%d, Hz=%d\n",hw_d[0],hw_d[1],hw_d[2]);

	data[QMCX983_AXIS_X] = clientdata->cvt.sign[QMCX983_AXIS_X]*hw_d[clientdata->cvt.map[QMCX983_AXIS_X]];
	data[QMCX983_AXIS_Y] = clientdata->cvt.sign[QMCX983_AXIS_Y]*hw_d[clientdata->cvt.map[QMCX983_AXIS_Y]];
	data[QMCX983_AXIS_Z] = clientdata->cvt.sign[QMCX983_AXIS_Z]*hw_d[clientdata->cvt.map[QMCX983_AXIS_Z]];

	//MSE_LOG("QMCX983 data [%d, %d, %d]\n", data[0], data[1], data[2]);
	
	return res;
}


/* Set the Gain range */
int qmcX983_set_range(short range)
{
	int err = 0;
	unsigned char data[2];
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(this_client);

	int ran ;
	switch (range) {
	case QMCX983_RNG_2G:
		ran = RNG_2G;
		break;
	case QMCX983_RNG_8G:
		ran = RNG_8G;
		break;
	case QMCX983_RNG_12G:
		ran = RNG_12G;
		break;
	case QMCX983_RNG_20G:
		ran = RNG_20G;
		break;
	default:
		return -EINVAL;
	}

	obj->xy_sensitivity = 20000/ran;
	obj->z_sensitivity = 20000/ran;

	data[0] = CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[0] &= 0xcf;
	data[0] |= (range << 4);
	data[1] = data[0];

	data[0] = CTL_REG_ONE;
	err = I2C_TxData(data, 2);
	return err;

}

/* Set the sensor mode */
int qmcX983_set_mode(char mode)
{
	int err = 0;

	unsigned char data[2];
	data[0] = CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[0] &= 0xfc;
	data[0] |= mode;
	data[1] = data[0];

	data[0] = CTL_REG_ONE;
	//MSE_LOG("QMCX983 in qmcX983_set_mode, data[1] = [%02x]", data[1]);
	err = I2C_TxData(data, 2);

	return err;
}

int qmcX983_set_ratio(char ratio)
{
	int err = 0;

	unsigned char data[2];
	data[0] = 0x0b;//RATIO_REG;
	data[1] = ratio;
	err = I2C_TxData(data, 2);
	return err;
}

static void qmcX983_start_measure(struct i2c_client *client)
{

	unsigned char data[2];
	int err;

	data[1] = 0x1d;
	data[0] = CTL_REG_ONE;
	err = I2C_TxData(data, 2);

}

static void qmcX983_stop_measure(struct i2c_client *client)
{

	unsigned char data[2];
	int err;

	data[1] = 0x1c;
	data[0] = CTL_REG_ONE;
	err = I2C_TxData(data, 2);
}

static int qmcX983_enable(struct i2c_client *client)
{

	unsigned char data[2];
	int err;

	data[1] = 0x1;
	data[0] = 0x21;
	err = I2C_TxData(data, 2);

	data[1] = 0x40;
	data[0] = 0x20;
	err = I2C_TxData(data, 2);

  	if(chip_id != QMC6983_A1_D1)
	{

		data[1] = 0x80;
		data[0] = 0x29;
		err = I2C_TxData(data, 2); 		

		data[1] = 0x0c;
		data[0] = 0x0a;
		err = I2C_TxData(data, 2);				
	}
	
	if(chip_id == QMC6983_E1_Metal || chip_id == QMC7983_Slope)
	{
		data[1] = 0x80;
		data[0] = 0x1b;
		err = I2C_TxData(data, 2); 			
	}
	
	MSE_LOG("start measure!\n");
	qmcX983_start_measure(client);

	qmcX983_set_range(QMCX983_RNG_8G);
	qmcX983_set_ratio(QMCX983_SETRESET_FREQ_FAST);				//the ratio must not be 0, different with qmc5983
	usleep_range(20000,30000); //fixit for amr ready
	qmcX983_start_measure(client);

	return 0;
}

static int qmcX983_disable(struct i2c_client *client)
{
	MSE_LOG("stop measure!\n");
	qmcX983_stop_measure(client);

	return 0;
}

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;

#ifdef	CONFIG_HAS_EARLYSUSPEND
static int qmcX983_SetPowerMode(struct i2c_client *client, bool enable)
{
	if(enable == true)
	{
		if(qmcX983_enable(client))
		{
			MSE_LOG("qmcX983: set power mode failed!\n");
			return -EINVAL;
		}
		else
		{
			MSE_LOG("qmcX983: set power mode enable ok!\n");
		}
	}
	else
	{
		if(qmcX983_disable(client))
		{
			MSE_LOG("qmcX983: set power mode failed!\n");
			return -EINVAL;
		}
		else
		{
			MSE_LOG("qmcX983: set power mode disable ok!\n");
		}
	}

	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static void qmcX983_power(struct mag_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;
	power_on = on;
}

/*----------------------------------------------------------------------------*/
static int qmcX983_ReadChipInfo(char *buf, int bufsize)
{
	if((!buf)||(bufsize <= QMCX983_BUFSIZE -1))
	{
		return -EINVAL;
	}
	if(!this_client)
	{
		*buf = 0;
		return -EINVAL;
	}
	if(chip_id == QMC7983_Vertical)
	{
		snprintf(buf, bufsize, "QMC7983_Vertical Chip");	
	}
	else if(chip_id == QMC7983_Slope)
	{
		snprintf(buf, bufsize, "QMC7983_Slope Chip");
	}
	else if(chip_id == QMC6983_E1)
	{
		snprintf(buf, bufsize, "QMC6983_E1 Chip");
	}
	else if(chip_id == QMC6983_E1_Metal)
	{
		snprintf(buf, bufsize, "QMC6983_E1_Metal Chip");
	}
	else if(chip_id == QMC6983_A1_D1)
	{
		snprintf(buf, bufsize, "QMC6983_A1_D1 Chip");	
	}		 
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[QMCX983_BUFSIZE];
	qmcX983_ReadChipInfo(strbuf, QMCX983_BUFSIZE);
	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int sensordata[3];

	qmcX983_read_mag_xyz(sensordata);

	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *data = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw.direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw.direction, &data->cvt))
		{
			MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw.direction);
		}
		else
		{
			MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw.direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(client);
	ssize_t len = 0;

	//if(obj->hw)
	//{
		len += scnprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw.i2c_num, obj->hw.direction, obj->hw.power_id, obj->hw.power_vol);
	//}


	len += scnprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	len += scnprintf(buf+len, PAGE_SIZE-len, "open_flag = 0x%x, v_open_flag=0x%x\n",
			atomic_read(&open_flag), v_open_flag);
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(this_client);
	
	if(NULL == obj)
	{
		MSE_ERR("qmcX983_i2c_data is null!!\n");
		return -EINVAL;
	}

	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj)
	{
		MSE_ERR("qmcX983_i2c_data is null!!\n");
		return -EINVAL;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		MSE_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}

	return count;
}

static ssize_t show_WRregisters_value(struct device_driver *ddri, char *buf)
{
	int res;

	unsigned char databuf[2];

//	struct i2c_client *client = this_client;
	//struct qmcX983_i2c_data *clientdata = i2c_get_clientdata(client);

	MSE_FUN();

	databuf[0] = regbuf[0];
	res = I2C_RxData(databuf, 1);
	if(res != 0){
		return -EFAULT;
	}
			
	MSE_LOG("QMCX983 hw_registers = 0x%02x\n",databuf[0]);  

	return scnprintf(buf, PAGE_SIZE, "hw_registers = 0x%02x\n", databuf[0]);
}

static ssize_t store_WRregisters_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(this_client);
	unsigned char tempbuf[1] = {0};
	unsigned char data[2] = {0};
	int err = 0;
	if(NULL == obj)
	{
		MSE_ERR("qmcX983_i2c_data is null!!\n");
		return -EINVAL;
	}
	tempbuf[0] = *buf;
	MSE_ERR("QMC6938:store_WRregisters_value: 0x%2x \n", tempbuf[0]);
	data[1] = tempbuf[0];
	data[0] = regbuf[0];
	err = I2C_TxData(data, 2);
	if(err != 0)
	   MSE_ERR("QMC6938: write registers 0x%2x  ---> 0x%2x success! \n", regbuf[0],tempbuf[0]);

	return count;
}

static ssize_t show_registers_value(struct device_driver *ddri, char *buf)
{
	MSE_LOG("QMCX983 hw_registers = 0x%02x\n",regbuf[0]);  
	   
	return scnprintf(buf, PAGE_SIZE, "hw_registers = 0x%02x\n", regbuf[0]);
}
static ssize_t store_registers_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(this_client);

	if(NULL == obj)
	{
		MSE_ERR("qmcX983_i2c_data is null!!\n");
		return -EINVAL;
	}
	regbuf[0] = *buf;
	MSE_ERR("QMC6938: REGISTERS = 0x%2x\n", regbuf[0]);
	return count;
}

static ssize_t show_regiter_map(struct device_driver *ddri, char *buf)
{
	int res;
	int i =0;
	char strbuf[300];
	char tempstrbuf[24];
	unsigned char databuf[2];
	int length=0;

	MSE_FUN();

	/* Check status register for data availability */	
	for(i =0;i<12;i++)
	{
		databuf[0] = i;
		res = I2C_RxData(databuf, 1);
		if(res < 0)
			MSE_LOG("QMCX983 dump registers 0x%02x failed !\n", i);

		length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "reg[0x%2x] =  0x%2x \n",i, databuf[0]);
		snprintf(strbuf+length*i, sizeof(strbuf)-length*i, "  %s \n",tempstrbuf);
	}

	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}

int FctShipmntTestProcess_Body(void)
{
	return 1;
}

static ssize_t store_shipment_test(struct device_driver * ddri,const char * buf, size_t count)
{
	return count;            
}

static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;
	res = FctShipmntTestProcess_Body();
	if(1 == res)
	{
	   MSE_LOG("shipment_test pass\n");
	   strcpy(result,"y");
	}
	else if(-1 == res)
	{
	   MSE_LOG("shipment_test fail\n");
	   strcpy(result,"n");
	}
	else
	{
	  MSE_LOG("shipment_test NaN\n");
	  strcpy(result,"NaN");
	}
	
	return sprintf(buf, "%s\n", result);        
}

static ssize_t show_OTP_value(struct device_driver *ddri, char *buf)
{
	return sprintf(buf,"%d,%d\n",OTP_Kx,OTP_Ky);	
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(shipmenttest,S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
static DRIVER_ATTR(regmap,  S_IRUGO , show_regiter_map, NULL);
static DRIVER_ATTR(WRregisters, S_IRUGO | S_IWUSR, show_WRregisters_value, store_WRregisters_value);
static DRIVER_ATTR(registers,   S_IRUGO | S_IWUSR, show_registers_value, store_registers_value);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
static DRIVER_ATTR(otp,			S_IRUGO, show_OTP_value, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *qmcX983_attr_list[] = {
	&driver_attr_shipmenttest,
	&driver_attr_regmap,
    &driver_attr_WRregisters,
	&driver_attr_registers,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
	&driver_attr_otp,
};
/*----------------------------------------------------------------------------*/
static int qmcX983_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)ARRAY_SIZE(qmcX983_attr_list);
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, qmcX983_attr_list[idx]);
		if(err < 0)
		{
			MSE_ERR("driver_create_file (%s) = %d\n", qmcX983_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int qmcX983_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)ARRAY_SIZE(qmcX983_attr_list);

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, qmcX983_attr_list[idx]);
	}

	return 0;
}


/*----------------------------------------------------------------------------*/
static int qmcX983_open(struct inode *inode, struct file *file)
{
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if(atomic_read(&obj->trace) & QMC_CTR_DEBUG)
	{
		MSE_LOG("Open device node:qmcX983\n");
	}
	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int qmcX983_release(struct inode *inode, struct file *file)
{
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(this_client);
	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) & QMC_CTR_DEBUG)
	{
		MSE_LOG("Release device node:qmcX983\n");
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static long qmcX983_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	//char buff[QMCX983_BUFSIZE];				/* for chip information */
	char rwbuf[16]; 		/* for READ/WRITE */
	int ret =-1;				

	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *clientdata = i2c_get_clientdata(client);

	if ((clientdata != NULL) && (atomic_read(&clientdata->trace) & QMC_FUN_DEBUG))
		MSE_LOG("qmcX983_unlocked_ioctl !cmd= 0x%x\n", cmd);
	
	switch (cmd){
	case QMC_IOCTL_WRITE:
		if(argp == NULL)
		{
			MSE_LOG("invalid argument.");
			return -EINVAL;
		}
		if(copy_from_user(rwbuf, argp, sizeof(rwbuf)))
		{
			MSE_LOG("copy_from_user failed.");
			return -EFAULT;
		}

		if((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE-1)))
		{
			MSE_LOG("invalid argument.");
			return -EINVAL;
		}
		ret = I2C_TxData(&rwbuf[1], rwbuf[0]);
		if(ret < 0)
		{
			return ret;
		}
		break;
			
	case QMC_IOCTL_READ:
		if(argp == NULL)
		{
			MSE_LOG("invalid argument.");
			return -EINVAL;
		}
		
		if(copy_from_user(rwbuf, argp, sizeof(rwbuf)))
		{
			MSE_LOG("copy_from_user failed.");
			return -EFAULT;
		}

		if((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE-1)))
		{
			MSE_LOG("invalid argument.");
			return -EINVAL;
		}
		ret = I2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
		{
			return ret;
		}
		if(copy_to_user(argp, rwbuf, rwbuf[0]+1))
		{
			MSE_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
#if 0		
	/*-------------------------for ftm------------------------**/

	case MSENSOR_IOCTL_READ_CHIPINFO:       //reserved?
		if(argp == NULL)
		{
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}

		qmcX983_ReadChipInfo(buff, QMCX983_BUFSIZE);
		if(copy_to_user(argp, buff, strlen(buff)+1))
		{
			return -EFAULT;
		}
		break;

	case MSENSOR_IOCTL_READ_SENSORDATA:	//for daemon
		if(argp == NULL)
		{
			MSE_LOG("IO parameter pointer is NULL!\r\n");
			break;
		}

		qmcX983_read_mag_xyz(vec);

		if ((clientdata != NULL) && (atomic_read(&clientdata->trace) & QMC_DATA_DEBUG))
			MSE_LOG("mag_data[%d, %d, %d]\n",vec[0],vec[1],vec[2]);
		
		snprintf(buff, sizeof(buff), "%x %x %x", vec[0], vec[1], vec[2]);
		if(copy_to_user(argp, buff, strlen(buff)+1))
		{
			return -EFAULT;
		}

			break;

	case MSENSOR_IOCTL_SENSOR_ENABLE:

		if(argp == NULL)
		{
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}
		if(copy_from_user(&enable, argp, sizeof(enable)))
		{
			MSE_LOG("copy_from_user failed.");
			return -EFAULT;
		}
		else
		{
			if(enable == 1)
			{
				v_open_flag |= 0x01;
				// we start measurement here.
				qmcX983_enable(this_client);
			}
			else
			{
				v_open_flag &= 0x3e;
				qmcX983_disable(this_client);
			}

			atomic_set(&open_flag, v_open_flag);

			MSE_ERR("qmcX983 v_open_flag = 0x%x,open_flag= 0x%x\n",v_open_flag, atomic_read(&open_flag));
		}
		break;

	case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		if(argp == NULL)
		{
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}


		mutex_lock(&sensor_data_mutex);

		osensor_data.values[0] = sensor_data[8];
		osensor_data.values[1] = sensor_data[9];
		osensor_data.values[2] = sensor_data[10];
		osensor_data.status = sensor_data[11];
		osensor_data.value_divide = CONVERT_O_DIV;

		mutex_unlock(&sensor_data_mutex);

		snprintf(buff, sizeof(buff), "%x %x %x %x %x", osensor_data.values[0], osensor_data.values[1],
			osensor_data.values[2],osensor_data.status,osensor_data.value_divide);
		if(copy_to_user(argp, buff, strlen(buff)+1))
		{
			return -EFAULT;
		}

		break;
#endif		
	default:
		MSE_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
		return -ENOIOCTLCMD;
		break;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static struct file_operations qmcX983_fops = {
//	.owner = THIS_MODULE,
	.open = qmcX983_open,
	.release = qmcX983_release,
	.unlocked_ioctl = qmcX983_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice qmcX983_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "qst_msensor",
    .fops = &qmcX983_fops,
};
/*----------------------------------------------------------------------------*/

static int qmcX983_m_open_report_data(int en)
{
	return 0;
}
static int qmcX983_m_set_delay(u64 delay)
{
	int value = (int)delay/1000/1000;
	struct qmcX983_i2c_data *data = NULL;

	if (unlikely(this_client == NULL)) {
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) {
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}

	if(value <= 10)
		qmcd_delay = 10;

	qmcd_delay = value;

	return 0;
}
static int qmcX983_m_enable(int en)
{
	struct qmcX983_i2c_data *data = NULL;

	if (unlikely(this_client == NULL)) 
	{
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) 
	{
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}

	if(en == 1)
	{
		v_open_flag |= 0x01;
		//we start measurement here.
		qmcX983_enable(this_client);
	}
	else
	{
		qmcX983_disable(this_client);
		v_open_flag &= 0x3e;
	}

	atomic_set(&open_flag, v_open_flag);
	
	MSE_ERR("qmcX983 v_open_flag = 0x%x,open_flag= 0x%x\n",v_open_flag, atomic_read(&open_flag));
	return 0;
}

static int qmcX983_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int qmcX983_flush(void)
{
	return mag_flush_report();
}

static int qmcX983_m_get_data(int *x, int *y, int *z, int *status)
{
    struct timespec time;
	struct qmcX983_i2c_data *data = NULL;
	int mag[3];
	int64_t cur_ns;
	if (unlikely(this_client == NULL)) {
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) {
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}
	qmcX983_read_mag_xyz(mag);
	
	*x = mag[0];
	*y = mag[1];
	*z = mag[2];
	*status = 3;
    time.tv_sec = time.tv_nsec = 0;
    get_monotonic_boottime(&time);
    cur_ns = time.tv_sec*1000000000LL+time.tv_nsec;

#if DEBUG
	if(atomic_read(&data->trace) & QMC_HWM_DEBUG)
	{				
		MSE_LOG("Hwm get m-sensor data: %d, %d, %d,status %d!\n", *x, *y, *z, *status);
	}		
#endif

	return 0;
}
static int qmcX983_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	struct qmcX983_i2c_data *data = NULL;
	int en = (enabledisable == true ? 1 : 0);
	int err;
    
	if (unlikely(this_client == NULL)) 
	{
		MSE_LOG("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) 
	{
		MSE_LOG("data is null!\n");
		return -EINVAL;
	}

	if(en == 1)
	{
		v_open_flag |= 0x01;
		//we start measurement here.
		qmcX983_enable(this_client);
	}
	else
	{
		qmcX983_disable(this_client);
		v_open_flag &= 0x3e;
	}

	atomic_set(&open_flag, v_open_flag);
	
	MSE_LOG("qmcX983 v_open_flag = 0x%x,open_flag= 0x%x\n",v_open_flag, atomic_read(&open_flag));

	err = qmcX983_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MSE_LOG("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}

static int qmcX983_factory_get_data(int32_t data[3], int *status)
{
	/* get raw data */
	return  qmcX983_m_get_data(&data[0], &data[1], &data[2], status);
}
static int qmcX983_factory_get_raw_data(int32_t data[3])
{
	MSE_LOG("do not support qmcX983_factory_get_raw_data!\n");
	return 0;
}
static int qmcX983_factory_enable_calibration(void)
{
	return 0;
}
static int qmcX983_factory_clear_cali(void)
{
	return 0;
}
static int qmcX983_factory_set_cali(int32_t data[3])
{
	return 0;
}
static int qmcX983_factory_get_cali(int32_t data[3])
{
	return 0;
}
static int qmcX983_factory_do_self_test(void)
{
	return 0;
}

static struct mag_factory_fops qmcX983_factory_fops = {
	.enable_sensor = qmcX983_factory_enable_sensor,
	.get_data = qmcX983_factory_get_data,
	.get_raw_data = qmcX983_factory_get_raw_data,
	.enable_calibration = qmcX983_factory_enable_calibration,
	.clear_cali = qmcX983_factory_clear_cali,
	.set_cali = qmcX983_factory_set_cali,
	.get_cali = qmcX983_factory_get_cali,
	.do_self_test = qmcX983_factory_do_self_test,
};

static struct mag_factory_public qmcX983_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &qmcX983_factory_fops,
};

/*----------------------------------------------------------------------------*/
#ifndef	CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int qmcX983_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(client);
    
	qmcX983_power(&obj->hw, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int qmcX983_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(client);
	
	qmcX983_power(&obj->hw, 1);

	return 0;
}

static int qmcX983_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, QMCX983_DEV_NAME, sizeof(info->type));
	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void qmcX983_early_suspend(struct early_suspend *h)
{
	struct qmcX983_i2c_data *obj = container_of(h, struct qmcX983_i2c_data, early_drv);

	if(NULL == obj)
	{
		MSE_ERR("null pointer!!\n");
		return;
	}
	
	qmcX983_power(&obj->hw, 0);
	
	if(qmcX983_SetPowerMode(obj->client, false))
	{
		MSE_LOG("qmcX983: write power control fail!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
static void qmcX983_late_resume(struct early_suspend *h)
{
	struct qmcX983_i2c_data *obj = container_of(h, struct qmcX983_i2c_data, early_drv);

	if(NULL == obj)
	{
		MSE_ERR("null pointer!!\n");
		return;
	}

	qmcX983_power(&obj->hw, 1);
	
	/// we can not start measurement , because we have no single measurement mode 

}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

static int qmcX983_device_check(void){
	unsigned char databuf[2] = {0};
	int ret = 0; 
	
	databuf[0] = 0x0d;
	ret = I2C_RxData(databuf, 1);
	if(ret < 0){
		MSE_ERR("%s: I2C_RxData failed\n",__func__);
		return ret;
	}
	
	if(0xff == databuf[0]){
		chip_id = QMC6983_A1_D1;
	}else if(0x31 == databuf[0]){
		chip_id = QMC6983_E1;
	}else if(0x32 == databuf[0]){
		
		//read otp 0x30
		databuf[0] = 0x2e;
		databuf[1] = 0x01;
		ret = I2C_TxData(databuf,2);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
		
		databuf[0] = 0x2f;
		ret = I2C_RxData(databuf, 1);
		if(ret < 0){
			MSE_ERR("%s: I2C_RxData failed\n",__func__);
			return ret;
		}		
		if(((databuf[0]&0x04 )>> 2))
		{
			chip_id = QMC6983_E1_Metal;
		}else{
			
			//read otp 0x3e
			databuf[0] = 0x2e;
			databuf[1] = 0x0f;
			ret = I2C_TxData(databuf,2);
			if(ret < 0)
			{
				MSE_ERR("%s: I2C_TxData failed\n",__func__);
				return ret;			
			}
			databuf[0] = 0x2f;
			ret = I2C_RxData(databuf, 1);
			if(ret < 0){
				MSE_ERR("%s: I2C_RxData failed\n",__func__);
				return ret;
			}
			if(0x02 == ((databuf[0]&0x3c)>>2)){
				chip_id = QMC7983_Vertical;
			}
			if(0x03 == ((databuf[0]&0x3c)>>2)){
				chip_id = QMC7983_Slope;
			}
		}
	}
	return ret;
}

static int qmcx983_get_OTP(void)
{
	unsigned char databuf[2] = {0};
	unsigned char value[2] = {0};
	int ret = 0;

	if(chip_id == QMC6983_A1_D1)
	{
		OTP_Kx = 0;
		OTP_Ky = 0;
		return 0;
	}
	else
	{
		//read otp_kx
		databuf[0] = 0x2e;
		databuf[1] = 0x0a;
		ret = I2C_TxData(databuf,2);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        mdelay(10);
		databuf[0] = 0x2f;
		ret = I2C_RxData(databuf, 1);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[0] = databuf[0];
    
		if(((value[0]&0x3f) >> 5) == 1)
		{
			OTP_Kx = (value[0]&0x1f)-32;
		}
		else
		{
			OTP_Kx = value[0]&0x1f;	
		}
		//read otp_ky
		databuf[0] = 0x2e;
		databuf[1] = 0x0d;
		ret = I2C_TxData(databuf,2);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        mdelay(10);
		databuf[0] = 0x2f;
		ret = I2C_RxData(databuf, 1);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[0] = databuf[0];
		mdelay(10);
		databuf[0] = 0x2e;
		databuf[1] = 0x0f;
		ret = I2C_TxData(databuf,2);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        mdelay(10);
		databuf[0] = 0x2f;
		ret = I2C_RxData(databuf, 1);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[1] = databuf[0];
		
		if((value[0] >> 7) == 1)
			OTP_Ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6))-32;
		else
			OTP_Ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6));	
	}
	return ret;
}

/*----------------------------------------------------------------------------*/
static int qmcX983_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct qmcX983_i2c_data *data = NULL;
    const char lib_name[64] = "libqmcX983";

	int err = 0;

	struct mag_control_path mag_ctl = {0};
	struct mag_data_path mag_data = {0};

	MSE_FUN();

	data = kmalloc(sizeof(struct qmcX983_i2c_data), GFP_KERNEL);
	if(data == NULL)
	{
		err = -ENOMEM;
		goto exit;
	}

	err = get_mag_dts_func(client->dev.of_node, &data->hw);
	if (err < 0) {
		MSE_ERR("%s. get dts info fail\n",__FUNCTION__);
		err = -EFAULT;
		goto exit_kfree;
	}

	err = hwmsen_get_convert(data->hw.direction, &data->cvt);	
	if (err) {
		MSE_ERR("QMCX983 invalid direction: %d\n", data->hw.direction);
		goto exit_kfree;
	}
	
	atomic_set(&data->layout, data->hw.direction);
	MSE_LOG("%s: direction: %d\n",__FUNCTION__,data->hw.direction);
	atomic_set(&data->trace, 0);

	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;

	/* read chip id */
	err = qmcX983_device_check();
	if(err < 0)
	{
		MSE_LOG("%s check ID faild!\n",__FUNCTION__);
		goto exit_kfree;
	}
	err = qmcx983_get_OTP();
	if(err < 0)
	{
		MSE_LOG("%s get OTP faild!\n",__FUNCTION__);
		goto exit_kfree;
	}

	err = qmcX983_create_attr(&(qmcX983_init_info.platform_diver_addr->driver));

	if (err < 0)
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = misc_register(&qmcX983_device);

	if(err < 0)
	{
		MSE_ERR("qmcX983_device register failed\n");
		goto exit_misc_device_register_failed;	
	}
	
	err = mag_factory_device_register(&qmcX983_factory_device);
	if (err) {
		MSE_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_factory_device_register_failed;
	}
	
	mag_ctl.enable = qmcX983_m_enable;
	mag_ctl.set_delay = qmcX983_m_set_delay;
	mag_ctl.open_report_data = qmcX983_m_open_report_data;
	mag_ctl.is_report_input_direct = false;
	mag_ctl.is_support_batch = data->hw.is_batch_supported;
	mag_ctl.batch = qmcX983_batch;
	mag_ctl.flush = qmcX983_flush;
	mag_ctl.libinfo.deviceid = chip_id;
	mag_ctl.libinfo.layout = data->hw.direction;
    memcpy(mag_ctl.libinfo.libname,"qmcX983",sizeof(lib_name));
	err = mag_register_control_path(&mag_ctl);
	if (err) {
		MAG_PR_ERR("register mag control path err\n");
		goto exit_hwm_attach_failed;
	}
	mag_data.div = CONVERT_M_DIV;
	mag_data.get_data = qmcX983_m_get_data;
	err = mag_register_data_path(&mag_data);
	if (err){
		MAG_PR_ERR("register data control path err\n");
		goto exit_hwm_attach_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend  = qmcX983_early_suspend,
	data->early_drv.resume   = qmcX983_late_resume,
	register_early_suspend(&data->early_drv);
#endif
	
    qmcX983_init_flag = 0;

	MSE_LOG("%s: OK\n", __func__);
	return 0;

exit_hwm_attach_failed:
	mag_factory_device_deregister(&qmcX983_factory_device);
exit_misc_factory_device_register_failed:
	misc_deregister(&qmcX983_device);
exit_misc_device_register_failed:
	qmcX983_delete_attr(&(qmcX983_init_info.platform_diver_addr->driver));
exit_sysfs_create_group_failed:
exit_kfree:
	kfree(data);
exit:
	MSE_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/



static int qmcX983_i2c_remove(struct i2c_client *client)
{
	int err;

	err = qmcX983_delete_attr(&(qmcX983_init_info.platform_diver_addr->driver));

	if (err < 0)
	{
		MSE_ERR("qmcX983_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&qmcX983_device);
	mag_factory_device_deregister(&qmcX983_factory_device);
	return 0;
}

static int qmcX983_local_init(void)
{
	qmcX983_power(qst_hw, 1);

	atomic_set(&dev_open_count, 0);

	if(i2c_add_driver(&qmcX983_i2c_driver))
	{
		MSE_ERR("add driver error\n");
		return -EINVAL;
	}

    if(-1 == qmcX983_init_flag)
    {
        MSE_ERR("%s failed!\n",__func__);
        return -EINVAL;
    }

	return 0;
}


static int qmcX983_local_remove(void)
{

	qmcX983_power(qst_hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&qmcX983_i2c_driver);
	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init qmcX983_init(void)
{
  
	mag_driver_add(&qmcX983_init_info);
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit qmcX983_exit(void)
{

}
/*----------------------------------------------------------------------------*/
module_init(qmcX983_init);
module_exit(qmcX983_exit);

MODULE_AUTHOR("QST Corp");
MODULE_DESCRIPTION("qmcX983 compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
