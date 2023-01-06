/* 
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

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "cust_alsps.h"
#include "ltr562.h"
#include "alsps.h"

#define GN_MTK_BSP_PS_DYNAMIC_CALI
//#define DELAYED_PS_CALI
//#define DEMO_BOARD
//#define LTR562_DEBUG
//#define SENSOR_DEFAULT_ENABLED
#define UPDATE_PS_THRESHOLD
//#define NO_ALS_CTRL_WHEN_PS_ENABLED
//#define REPORT_PS_ONCE_WHEN_ALS_ENABLED
#define NO_PS_CTRL_WHEN_SUSPEND_RESUME

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define LTR562_DEV_NAME			"ltr562"

/*----------------------------------------------------------------------------*/
#define APS_TAG					"[ALS/PS] "
#define APS_FUN(f)              printk(KERN_INFO 	APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)   printk(KERN_ERR  	APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)   printk(KERN_NOTICE	APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)   printk(KERN_ERR 	APS_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr562_i2c_id[] = {{LTR562_DEV_NAME,0},{}};
static unsigned long long int_top_time;
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

/*----------------------------------------------------------------------------*/
static int ltr562_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr562_i2c_remove(struct i2c_client *client);
static int ltr562_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int ltr562_i2c_suspend(struct device *dev);
static int ltr562_i2c_resume(struct device *dev);

#ifdef LTR562_DEBUG
static int ltr562_dump_reg(void);
#endif

static int als_gainrange;
static int als_intfac;

static int final_prox_val;
static int final_lux_val;

#ifdef ALS_USE_DRIVER_AVE
#define GREEN_MAX_DELTA	500			// max Delta that you will do averaging, delta more than this will ignore averaging
#define IR_MAX_DELTA	500			
#define GREEN_K_FAC		7			// Averaging sensitivity factor. The larger the value, the less averaging will be done.
#define IR_K_FAC		7

static int als_start_flag = 0;		// 0: 1st data, 1: non 1st data
static int g_old_data = 0;			// previous processed data
static int ir_old_data = 0;
#endif

#ifdef PS_USE_DRIVER_AVE
#define PS_MAX_DELTA	500			// max Delta that you will do averaging, delta more than this will ignore averaging
#define PS_K_FAC		7			// Averaging sensitivity factor. The larger the value, the less averaging will be done.

static int ps_start_flag = 0;		// 0: 1st data, 1: non 1st data
static int ps_old_data = 0;			// previous processed PS data
#endif

struct platform_device *alspsPltFmDev_ltr562;

typedef enum {
	TYPE_ALS = 1,
	TYPE_PS = 2,	
} DATA_TYPE;

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr562_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;
#ifdef REPORT_PS_ONCE_WHEN_ALS_ENABLED
	struct delayed_work check_ps_work;
#endif
#ifdef DELAYED_PS_CALI
	struct delayed_work cali_ps_work;
#endif

	/*misc*/
	u16 		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t 	trace;

#ifdef CONFIG_OF
	struct device_node *irq_node;
	int		irq;
#endif

	/*data*/
	u16			als;
	u16 		ps;
	u8			_align;
	u16			als_level_num;
	u16			als_value_num;
	u32			als_level[C_CUST_ALS_LEVEL-1];
	u32			als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;
	
	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	ulong		enable; 		/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/
};

struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;

static struct ltr562_priv *ltr562_obj = NULL;
static struct i2c_client *ltr562_i2c_client = NULL;

static DEFINE_MUTEX(ltr562_mutex);
static DEFINE_MUTEX(ltr562_i2c_mutex);

static int ltr562_local_init(void);
static int ltr562_remove(void);
static int ltr562_init_flag =-1; // 0<==>OK -1 <==> fail

static int ltr562_sensor_init(void);

static int ps_enabled = 0;
static int als_enabled = 0;
static int irq_enabled = 0;

#ifdef PS_USE_XT_OFFSET
static int ps_xt_offset_val = 400;
static int ps_val_zero_counter = 0;
static int ps_offset_update = 0;
#endif

static int als_dark_count_table[] = { 0, 20, 40, 60, 80, 100, 120, 140 };
static int als_dark_count;

static struct alsps_init_info ltr562_init_info = {
		.name = "ltr562",
		.init = ltr562_local_init,
		.uninit = ltr562_remove,	
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,ltr562"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ltr562_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr562_i2c_suspend, ltr562_i2c_resume)
};
#endif

static struct i2c_driver ltr562_i2c_driver = {	
	.probe      = ltr562_i2c_probe,
	.remove     = ltr562_i2c_remove,
	.detect     = ltr562_i2c_detect,
	.id_table   = ltr562_i2c_id,
	.driver = {
		.name           = LTR562_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif

#ifdef CONFIG_PM_SLEEP
		.pm = &ltr562_pm_ops,
#endif
	},
};

/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr562_dynamic_calibrate(void);
static int dynamic_calibrate = 0;
#endif
/*-----------------------------------------------------------------------------*/

/* 
 * #########
 * ## I2C ##
 * #########
 */
static int ltr562_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err;
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &beg, },
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data, }
	};

	mutex_lock(&ltr562_i2c_mutex);
	if (!client) {
		mutex_unlock(&ltr562_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&ltr562_i2c_mutex);
		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	mutex_unlock(&ltr562_i2c_mutex);
	if (err != 2) {
		APS_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else {
		err = 0;	/*no error */
	}
	return err;
}

static int ltr562_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&ltr562_i2c_mutex);
	if (!client) {
		mutex_unlock(&ltr562_i2c_mutex);
		return -EINVAL;
	}
	else if (len >= C_I2C_FIFO_SIZE) {
		APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&ltr562_i2c_mutex);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++) {
		buf[num++] = data[idx];
	}

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		APS_ERR("send command error!!\n");
		mutex_unlock(&ltr562_i2c_mutex);
		return -EFAULT;
	}
	mutex_unlock(&ltr562_i2c_mutex);
	return err;
}

/*----------------------------------------------------------------------------*/
static int ltr562_master_recv(struct i2c_client *client, u16 addr, u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&ltr562_obj->trace);
	int max_try = atomic_read(&ltr562_obj->i2c_retry);

	while (retry++ < max_try) {
		ret = ltr562_i2c_read_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & 0x8000)) {
			APS_LOG("(recv) %d/%d\n", retry - 1, max_try);
		}
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}

/*----------------------------------------------------------------------------*/
static int ltr562_master_send(struct i2c_client *client, u16 addr, u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&ltr562_obj->trace);
	int max_try = atomic_read(&ltr562_obj->i2c_retry);

	while (retry++ < max_try) {
		ret = ltr562_i2c_write_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & 0x8000)) {
			APS_LOG("(send) %d/%d\n", retry - 1, max_try);
		}
	}
	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}

/*----------------------------------------------------------------------------*/
static void ltr562_power(struct alsps_hw *hw, unsigned int on) 
{
#ifdef DEMO_BOARD
	static unsigned int power_on = 0;	

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "ltr562")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "ltr562")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
#endif
}

static int ltr562_get_efuse_data(void)
{
	int res;
	u8 buf;
	struct i2c_client *client = ltr562_obj->client;

	buf = 0x1c;
	res = ltr562_master_send(client, LTR562_DARK_CTRL, (char *)&buf, 1);
	if (res < 0)
	{
		APS_LOG("ltr562_get_efuse_data() DARK_CTRL error...\n");
		return res;
	}

	buf = 0x1d;
	res = ltr562_master_send(client, LTR562_DARK_CTRL, (char *)&buf, 1);
	if (res < 0)
	{
		APS_LOG("ltr562_get_efuse_data() DARK_CTRL error...\n");
		return res;
	}

	res = ltr562_master_recv(client, LTR562_DARK_CONFIG, &buf, 0x01);
	if (res < 0) {
		APS_LOG("ltr562_get_efuse_data() DARK_CONFIG error...\n");
		return res;
	}

	als_dark_count = als_dark_count_table[(buf >> 5)];
	APS_LOG("ltr562_get_efuse_data: als_dark_count = %d\n", als_dark_count);

	return res;
}

#ifdef ALS_USE_DRIVER_AVE
static int ltr562_data_average(struct i2c_client *client, int max_delta, int k_fac, int read_data, int old_data, int data_type)
{	
	int delta;				// delta of processed als_old_data and current unprocessed als_cur_data
	int delta_fac;
	int delta_fac_temp;
	int ps_avg1, ps_avg2;
	int processed_data;

	delta = read_data - old_data;
	APS_DBG("ltr562_data_average: delta = %d\n", delta);
	if (abs(delta) > max_delta)
	{
		processed_data = read_data;
		goto Exit;
	}
	else
	{
		delta_fac = delta * k_fac * abs(delta) * 10 / (1000 + abs(delta) * k_fac);	// Round up/down to nearest integer
		delta_fac_temp = delta_fac;
		if (abs(delta_fac) % 10 >= 5)
		{
			if (delta_fac > 0)
				delta_fac = delta_fac / 10 + 1;
			else
				delta_fac = delta_fac / 10 - 1;
		}
		else
			delta_fac = delta_fac / 10;
		APS_DBG("ltr562_data_average: delta_fac_temp = %d, delta_fac = %d\n", delta_fac_temp, delta_fac);

		if (data_type == TYPE_PS)
		{
			ps_avg1 = old_data + delta_fac;
			ps_avg2 = (old_data + read_data) / 2;
			APS_DBG("ltr562_data_average: ps_avg1 = %d, ps_avg2 = %d\n", ps_avg1, ps_avg2);

			if (abs(read_data - ps_avg1) < abs(read_data - ps_avg2))
				processed_data = ps_avg1;
			else
				processed_data = ps_avg2;
		}
		else
		{
			if (abs(delta_fac) > 0)
			{
				processed_data = old_data + delta_fac;
			}
			else
			{
				if (abs(delta) > 2)
				{
					if (delta > 0)
						processed_data = old_data + 2;
					else
						processed_data = old_data - 2;
				}
				else
				{
					processed_data = old_data + delta;
				}
			}
		}
	}

Exit:
	APS_DBG("ltr562_data_average: processed_data = %d\n", processed_data);	
	return processed_data;
}
#endif

/********************************************************************/
/* 
 * ###############
 * ## PS CONFIG ##
 * ###############
 */

#ifdef PS_USE_XT_OFFSET
static int ltr562_ps_set_offset(int offset)
{
	int res;
	u8 buf;
	struct i2c_client *client = ltr562_obj->client;

	buf = offset & 0x00ff;
	res = ltr562_master_send(client, LTR562_PS_CAN_0, (char *)&buf, 1);
	if (res < 0)
	{
		APS_LOG("ltr562_init_client() PS can_0 error...\n");
		return res;
	}

	buf = (offset & 0xff00) >> 8;
	res = ltr562_master_send(client, LTR562_PS_CAN_1, (char *)&buf, 1);
	if (res < 0)
	{
		APS_LOG("ltr562_init_client() PS can_1 error...\n");
		return res;
	}

	return res;
}

#define BUFFER_SIZE				8
#define PS_COUNT_MAX_VARIATION	10		// the maximum ps count variation when ps is stable
static u16 ps_data[BUFFER_SIZE];
static u16 data_counter = 0, buffer_full = 0, stable_counter = 0;

static u8 get_stable_ps(u16 ps_count)
{
	u16 ps_avg = 0;
	u16 ps_avg_h;
	u16 ps_avg_l;
	u8 i;
	u16 ps_data_sum = 0;

	if (data_counter >= BUFFER_SIZE)
		buffer_full = 1;

	if (buffer_full) {
		data_counter %= BUFFER_SIZE;
	}

	ps_data[data_counter] = ps_count;

	if (buffer_full) {
		for (i = 0; i < BUFFER_SIZE; i++) {			
			ps_data_sum += ps_data[i];
		}

		ps_avg = ps_data_sum / BUFFER_SIZE;
		ps_avg_h = ps_avg + PS_COUNT_MAX_VARIATION;
		ps_avg_l = ps_avg - PS_COUNT_MAX_VARIATION;

		for (i = 0; i < BUFFER_SIZE; i++)
		{
			if (ps_data[i] < ps_avg_h && ps_data[i] > ps_avg_l)
				stable_counter++;
			else
				stable_counter = 0;
		}
	}
	else {
		stable_counter = 0;
	}

	data_counter++;

	if (stable_counter >= BUFFER_SIZE - 1) {
		return 1;	// ps data stable
	}
	else {
		return 0;	// ps data not stable
	}
}

static void clear_ps_stable_counter(void)
{
	data_counter = 0;
	buffer_full = 0;
	stable_counter = 0;
}
#endif

static int ltr562_ps_get_thres(u16 noise)
{	
	int ps_thd_val_low, ps_thd_val_high;	
	struct ltr562_priv *obj = ltr562_obj;

	if (!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return -1;
	}

	if (noise < 100) {
		ps_thd_val_high = noise + 100;
		ps_thd_val_low = noise + 50;
	}
	else if (noise < 200) {
		ps_thd_val_high = noise + 150;
		ps_thd_val_low = noise + 60;
	}
	else if (noise < 300) {
		ps_thd_val_high = noise + 150;
		ps_thd_val_low = noise + 60;
	}
	else if (noise < 400) {
		ps_thd_val_high = noise + 150;
		ps_thd_val_low = noise + 60;
	}
	else if (noise < 600) {
		ps_thd_val_high = noise + 180;
		ps_thd_val_low = noise + 90;
	}
	else if (noise < 1000) {
		ps_thd_val_high = noise + 300;
		ps_thd_val_low = noise + 180;
	}
	else {
		ps_thd_val_high = 1600;
		ps_thd_val_low = 1400;
	}

	atomic_set(&obj->ps_thd_val_high, ps_thd_val_high);
	atomic_set(&obj->ps_thd_val_low, ps_thd_val_low);
	ps_cali.valid = 1;
	ps_cali.far_away = ps_thd_val_low;
	ps_cali.close = ps_thd_val_high;	

	APS_LOG("%s:noise = %d\n", __func__, noise);
	APS_LOG("%s:obj->ps_thd_val_high = %d\n", __func__, ps_thd_val_high);
	APS_LOG("%s:obj->ps_thd_val_low = %d\n", __func__, ps_thd_val_low);

	return 0;
}

static int ltr562_ps_set_thres(void)
{
	int res;
	u8 databuf[2];
	
	struct i2c_client *client = ltr562_obj->client;
	struct ltr562_priv *obj = ltr562_obj;

	APS_FUN();

	APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);

	if(1 == ps_cali.valid)
	{
		databuf[0] = LTR562_PS_THRES_LOW_0; 
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{			
			goto EXIT_ERR;
		}
		databuf[0] = LTR562_PS_THRES_LOW_1; 
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR562_PS_THRES_UP_0;	
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR562_PS_THRES_UP_1;	
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
        ps_cali.valid = 0;
	}
	else
	{
		databuf[0] = LTR562_PS_THRES_LOW_0; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR562_PS_THRES_LOW_1; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low )>> 8) & 0x00FF);
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR562_PS_THRES_UP_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR562_PS_THRES_UP_1;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}	
	}

	res = 0;
	return res;
	
EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	res = -1;
	return res;
}

static int ltr562_ps_enable(struct i2c_client *client, int enable)
{
	u8 regdata;
	int err;

	APS_LOG("ltr562_ps_enable() ...start!\n");

	if (enable != 0 && ps_enabled == 1)
	{
		APS_LOG("PS: Already enabled \n");
		return 0;
	}

	if (enable == 0 && ps_enabled == 0)
	{
		APS_LOG("PS: Already disabled \n");
		return 0;
	}

	err = ltr562_master_recv(client, LTR562_PS_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}
	
#ifdef PS_USE_XT_OFFSET
	regdata |= 0x18;
	regdata &= 0xFE;	// Clear reset bit
#else
	regdata |= 0x10;
	regdata &= 0xF6;	// Clear reset and offset bit
#endif
	if (enable != 0) {
		APS_LOG("PS: enable ps only \n");
		regdata |= 0x02;
	}
	else {
		APS_LOG("PS: disable ps only \n");
		regdata &= 0xFD;
	}

	err = ltr562_master_send(client, LTR562_PS_CTRL, (char *)&regdata, 1);
	if (err < 0)
	{
		APS_ERR("PS: enable ps err: %d en: %d \n", err, enable);
		return err;
	}
	mdelay(WAKEUP_DELAY);
	err = ltr562_master_recv(client, LTR562_PS_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}
	
	if (0 == ltr562_obj->hw->polling_mode_ps && enable != 0)
	{
#ifndef DELAYED_PS_CALI
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
		err = ltr562_dynamic_calibrate();
		if (err < 0)
		{
			APS_LOG("ltr562_dynamic_calibrate() failed\n");
		}
#endif
		ltr562_ps_set_thres();
#else
		cancel_delayed_work(&ltr562_obj->cali_ps_work);
		schedule_delayed_work(&ltr562_obj->cali_ps_work, msecs_to_jiffies(200));
#endif
	}
	else if (0 == ltr562_obj->hw->polling_mode_ps && enable == 0)
	{
		//cancel_work_sync(&ltr562_obj->eint_work);		
	}

	if (enable != 0)
	{
		ps_enabled = 1;
#ifdef PS_USE_XT_OFFSET
		ps_val_zero_counter = 0;
		ps_offset_update = 0;
#endif
#ifdef PS_USE_DRIVER_AVE
		ps_start_flag = 0;
#endif
	}
	else
	{
		ps_enabled = 0;		
	}

	if ((irq_enabled == 1) && (enable != 0))
	{
		irq_enabled = 2;
	}

	return err;
}

/********************************************************************/
static int ltr562_ps_read(struct i2c_client *client, u16 *data)
{
	int psdata, ps_status, ret = 0;
	u8 buf[2];
	u8 regdata;

	ret = ltr562_master_recv(client, LTR562_PS_PULSES, buf, 0x01);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	if (buf[0] != PS_DEF_AVG_PULSE)
	{
		APS_DBG("Something bad happens! Re-init sensor!\n");
		ret = ltr562_sensor_init();

		if (als_enabled == 1)
		{
			regdata = ALS_DEF_GAIN_REG | 0x21;
			ltr562_master_send(client, LTR562_ALS_CTRL, (char *)&regdata, 1);
		}

		if (ps_enabled == 1)
		{
#ifdef PS_USE_XT_OFFSET
			regdata |= 0x18;
			regdata &= 0xFE;	// Clear reset bit
#else
			regdata |= 0x10;
			regdata &= 0xF6;	// Clear reset and offset bit
#endif
			regdata |= 0x02;
			ltr562_master_send(client, LTR562_PS_CTRL, (char *)&regdata, 1);
		}

		mdelay(WAKEUP_DELAY);
	}

	ret = ltr562_master_recv(client, LTR562_PS_STATUS, buf, 0x01);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	APS_DBG("ps_status = %d\n", buf[0]);
	ps_status = buf[0];

	ret = ltr562_master_recv(client, LTR562_PS_DATA_0, buf, 0x02);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	APS_DBG("ps_rawdata_lo = %d\n", buf[0]);	
    APS_DBG("ps_rawdata_hi = %d\n", buf[1]);	
	
	psdata = ((buf[1] & 0x07) << 8) | (buf[0]);	
    APS_DBG("ltr562_ps_read: ps_rawdata = %d\n", psdata);

#if 0
	if ((ps_status & 0x04) == 0x04)
	{
		APS_DBG("ltr562_ps_read: ps sunlight saturation\n");
		psdata = 0;
	}
	else
#endif
	{
#ifdef PS_USE_XT_OFFSET
		if (psdata <= 0)
		{
			APS_DBG("ltr562_ps_read: Update ps_val_zero_counter: %d\n", ps_val_zero_counter);
			ps_val_zero_counter++;

			if (ps_val_zero_counter > PS_OFFSET_CHANGE_CONTER)
			{
				ps_val_zero_counter = 0;	// reset counter
				if (ps_xt_offset_val != 0)
				{
					ps_offset_update = 1;		// To trigger threshold change
					clear_ps_stable_counter();
					if (ps_xt_offset_val > PS_XT_TARGET)
						ps_xt_offset_val -= PS_XT_TARGET;
					else
						ps_xt_offset_val = 0;

					ltr562_ps_set_offset(ps_xt_offset_val);
					APS_DBG("ltr562_ps_read: Update ps_xt_offset_val: %d\n", ps_xt_offset_val);
				}
			}
		}
		else
		{
			ps_val_zero_counter = 0;	// reset counter

			if ((ps_offset_update == 1) && (get_stable_ps(psdata) == 1))
			{
				ps_offset_update = 0;
				ltr562_ps_get_thres(psdata);
				ltr562_ps_set_thres();				
				APS_DBG("ltr562_ps_read: Update ps_xtalk_avg: %d\n", psdata);				
			}
		}
#endif		
	}

#ifdef PS_USE_DRIVER_AVE
	if (ps_start_flag == 0)		// 1st PS data
	{
		ps_start_flag = 1;
		ps_old_data = psdata;
	}
	else
	{
		psdata = ltr562_data_average(client, PS_MAX_DELTA, PS_K_FAC, psdata, ps_old_data, TYPE_PS);
	}
#endif

	*data = psdata;
	final_prox_val = psdata;	
	return psdata;
}

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr562_dynamic_calibrate(void)
{
	int i = 0;
	int data;
	int data_total = 0;
	int noise = 0;
	int count = 5;	
	int res;
	u8 buf;
	
#ifdef PS_USE_XT_OFFSET
	ltr562_ps_set_offset(0);	// set ps xt offset to 0 first
#endif
	
	buf = 0x00;	// 11bits & 6.125ms time 
	res = ltr562_master_send(ltr562_obj->client, LTR562_PS_MEAS_RATE, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr562_dynamic_calibrate() PS time error...\n");
		return -1;
	}

	for (i = 0; i < count; i++) {
		// wait for ps value be stable, ps measurement Time is set to 50ms
		msleep(7);

		data = ltr562_ps_read(ltr562_obj->client, &ltr562_obj->ps);
		if (data < 0) {
			i--;
			continue;
		}			

		data_total += data;
	}

	noise = data_total / count;

#ifdef PS_USE_XT_OFFSET
	if (noise > PS_XT_TARGET)
	{
		ps_xt_offset_val = noise - PS_XT_TARGET;
		dynamic_calibrate = noise = PS_XT_TARGET;
	}
	else
	{
		ps_xt_offset_val = 0;
		dynamic_calibrate = noise;
	}

	ltr562_ps_set_offset(ps_xt_offset_val);
#else
	dynamic_calibrate = noise;
#endif

	ltr562_ps_get_thres(noise);	

	buf = PS_DEF_MEAS_RATE;
	res = ltr562_master_send(ltr562_obj->client, LTR562_PS_MEAS_RATE, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr562_dynamic_calibrate() PS time error...\n");
		return -1;
	}

	return 0;
}
#endif
/********************************************************************/
/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr562_als_enable(struct i2c_client *client, int enable)
{
	int err = 0;
	u8 regdata = 0;

	if (enable != 0 && als_enabled == 1)
	{
		APS_LOG("ALS: Already enabled \n");
		return 0;
	}

	if (enable == 0 && als_enabled == 0)
	{
		APS_LOG("ALS: Already disabled \n");
		return 0;
	}
#ifdef NO_ALS_CTRL_WHEN_PS_ENABLED
	if (ps_enabled == 1)
	{
		APS_LOG("ALS: PS enabled, do nothing \n");
		return 0;
	}
#endif
	err = ltr562_master_recv(client, LTR562_ALS_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}	
	
	if (enable != 0) {
		APS_LOG("ALS(1): enable als only \n");
		regdata |= 0x01;		
	}
	else {
		APS_LOG("ALS(1): disable als only \n");
		regdata &= 0xFE;
	}

	err = ltr562_master_send(client, LTR562_ALS_CTRL, (char *)&regdata, 1);
	if (err < 0)
	{
		APS_ERR("ALS: enable als err: %d en: %d \n", err, enable);
		return err;
	}

	mdelay(WAKEUP_DELAY);

	if (enable != 0)
	{
		als_enabled = 1;
#ifdef ALS_USE_DRIVER_AVE
		als_start_flag = 0;		
#endif
	}
	else
		als_enabled = 0;

	return 0;
}

static int ltr562_als_read(struct i2c_client *client, u16* data)
{
	int alsval = 0, irval = 0;
	int luxdata_int;	
	u8 buf[2];
	//u8 sar_code;
	int ret;

	ret = ltr562_master_recv(client, LTR562_ALS_DATA_0, buf, 0x02);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	alsval = (buf[1] * 256) + buf[0];
	APS_DBG("alsval_0 = %d,alsval_1=%d,alsval=%d\n", buf[0], buf[1], alsval);

	ret = ltr562_master_recv(client, LTR562_IR_DATA_0, buf, 0x02);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	irval = (buf[1] * 256) + buf[0];
	APS_DBG("irval_0 = %d,irval_1=%d,irval=%d\n", buf[0], buf[1], irval);

	if (alsval == 0)
		irval = 0;

#ifdef ALS_USE_DRIVER_AVE
	if (als_start_flag == 0)		// 1st ALS data
	{
		als_start_flag = 1;
		g_old_data = alsval;		
		ir_old_data = irval;
	}
	else
	{
		g_old_data = ltr562_data_average(client, GREEN_MAX_DELTA, GREEN_K_FAC, alsval, g_old_data, TYPE_ALS);
		APS_DBG("ltr562_als_read: green processed data = %d\n", g_old_data);		

		ir_old_data = ltr562_data_average(client, IR_MAX_DELTA, IR_K_FAC, irval, ir_old_data, TYPE_ALS);
		APS_DBG("ltr562_als_read: IR processed data = %d\n", ir_old_data);
	}
#else
	g_old_data = alsval;
	ir_old_data = irval;
#endif

	APS_DBG("ltr562_als_read: als_dark_count = %d\n", als_dark_count);
	if (g_old_data > als_dark_count * als_gainrange / 512)
		luxdata_int = (g_old_data - als_dark_count * als_gainrange / 512) * ALS_WIN_FACTOR * 182 / als_gainrange / als_intfac / 1000;
	else
		luxdata_int = 0;

	APS_DBG("ltr562_als_read: als_value_lux = %d\n", luxdata_int);

	*data = luxdata_int;
	final_lux_val = luxdata_int;
	return luxdata_int;
}
/********************************************************************/
static int ltr562_get_ps_value(struct ltr562_priv *obj, u16 ps)
{
	int val = 1;
	int invalid = 0;

	if((ps > atomic_read(&obj->ps_thd_val_high)))
	{
		val = 0;  /*close*/
		intr_flag_value = 1;
	}
	else if((ps < atomic_read(&obj->ps_thd_val_low)))
	{
		val = 1;  /*far away*/
		intr_flag_value = 0;
	}
		
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 50000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}
}
/********************************************************************/
static int ltr562_get_als_value(struct ltr562_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	APS_DBG("als  = %d\n",als); 
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ltr562_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&ltr562_obj->i2c_retry), atomic_read(&ltr562_obj->als_debounce), 
		atomic_read(&ltr562_obj->ps_mask), atomic_read(&ltr562_obj->ps_thd_val), atomic_read(&ltr562_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&ltr562_obj->i2c_retry, retry);
		atomic_set(&ltr562_obj->als_debounce, als_deb);
		atomic_set(&ltr562_obj->ps_mask, mask);
		atomic_set(&ltr562_obj->ps_thd_val, thres);        
		atomic_set(&ltr562_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr562_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&ltr562_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_als(struct device_driver *ddri, char *buf)
{
	int res;
		
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	res = ltr562_als_read(ltr562_obj->client, &ltr562_obj->als);
	return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);
	
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	res = ltr562_ps_read(ltr562_obj->client, &ltr562_obj->ps);
    return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_reg(struct device_driver *ddri, char *buf)
{
	int i,len=0;
	int reg[]={0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x91,0x92,0x93,
			   0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,0xa4,0xb4,0xb6,0xb7,0xb9};
	int ret;
	u8 buffer;

	for(i=0;i<29;i++)
	{
		ret = ltr562_master_recv(ltr562_obj->client, reg[i], &buffer, 0x01);
		if (ret < 0) {
			APS_DBG("i2c error: %d\n", ret);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%04X value: 0x%04X\n", reg[i], buffer);
	}
	return len;
}

#ifdef LTR562_DEBUG
static int ltr562_dump_reg(void)
{
	int i=0;
	int reg[]={0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x91,0x92,0x93,
			   0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,0xa4,0xb4,0xb6,0xb7,0xb9};
	int ret;
	u8 buffer;

	for(i=0;i<29;i++)
	{
		ret = ltr562_master_recv(ltr562_obj->client, reg[i], &buffer, 0x01);
		if (ret < 0) {
			APS_DBG("i2c error: %d\n", ret);
		}

		APS_DBG("reg:0x%04X value: 0x%04X\n", reg[i], buffer);
	}
	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	
	if(ltr562_obj->hw)
	{	
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr562_obj->hw->i2c_num, ltr562_obj->hw->power_id, ltr562_obj->hw->power_vol);		
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}


	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr562_obj->als_suspend), atomic_read(&ltr562_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr562_priv *obj, const char* buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < ltr562_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr562_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr562_obj->als_level, ltr562_obj->hw->als_level, sizeof(ltr562_obj->als_level));
	}
	else if(ltr562_obj->als_level_num != read_int_from_buf(ltr562_obj, buf, count, 
			ltr562_obj->hw->als_level, ltr562_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < ltr562_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr562_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr562_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr562_obj->als_value, ltr562_obj->hw->als_value, sizeof(ltr562_obj->als_value));
	}
	else if(ltr562_obj->als_value_num != read_int_from_buf(ltr562_obj, buf, count, 
			ltr562_obj->hw->als_value, ltr562_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, ltr562_show_als,		NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, ltr562_show_ps,		NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, ltr562_show_config,	ltr562_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, ltr562_show_alslv,	ltr562_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, ltr562_show_alsval,	ltr562_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, ltr562_show_trace,	ltr562_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, ltr562_show_status,	NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, ltr562_show_send,	ltr562_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, ltr562_show_recv,	ltr562_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, ltr562_show_reg,		NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr562_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int ltr562_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr562_attr_list)/sizeof(ltr562_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, ltr562_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ltr562_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr562_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr562_attr_list)/sizeof(ltr562_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ltr562_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------interrupt functions--------------------------------*/
#ifdef REPORT_PS_ONCE_WHEN_ALS_ENABLED
static void ltr562_check_ps_work(struct work_struct *work)
{
	struct ltr562_priv *obj = ltr562_obj;	
	struct hwm_sensor_data sensor_data;
	APS_FUN();

	if (test_bit(CMC_BIT_PS, &obj->enable)) {
		ltr562_ps_read(obj->client, &obj->ps);
		APS_LOG("ltr562_check_ps_work rawdata ps=%d high=%d low=%d\n", obj->ps, atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));

		sensor_data.values[0] = ltr562_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

		if (ps_report_interrupt_data(sensor_data.values[0]))
		{
			APS_ERR("call ps_report_interrupt_data fail \n");
		}
	}

	return;
}
#endif

#ifdef DELAYED_PS_CALI
static void ltr562_cali_ps_work(struct work_struct *work)
{
	int err = 0;

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
	err = ltr562_dynamic_calibrate();
	if (err < 0)
	{
		APS_LOG("ltr562_dynamic_calibrate() failed\n");
	}
#endif		
	ltr562_ps_set_thres();
}
#endif
/*----------------------------------------------------------------------------*/
static void ltr562_eint_work(struct work_struct *work)
{
	struct ltr562_priv *obj = (struct ltr562_priv *)container_of(work, struct ltr562_priv, eint_work);	
	int res = 0;
	int value = 1;
#ifdef UPDATE_PS_THRESHOLD
	u8 databuf[2];
#endif

	//get raw data
	obj->ps = ltr562_ps_read(obj->client, &obj->ps);
	if (obj->ps < 0)
	{
		goto EXIT_INTR;
	}
				
	APS_DBG("ltr562_eint_work: rawdata ps=%d!\n",obj->ps);
	value = ltr562_get_ps_value(obj, obj->ps);
	APS_DBG("intr_flag_value=%d\n",intr_flag_value);
	if(intr_flag_value){
#ifdef UPDATE_PS_THRESHOLD
		databuf[0] = LTR562_PS_THRES_LOW_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
		databuf[0] = LTR562_PS_THRES_LOW_1;	
		databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
		databuf[0] = LTR562_PS_THRES_UP_0;	
		databuf[1] = (u8)(0x00FF);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
		databuf[0] = LTR562_PS_THRES_UP_1; 
		databuf[1] = (u8)((0xFF00) >> 8);;
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
#endif
	}
	else{	
#ifndef DELAYED_PS_CALI
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
		if(obj->ps > 20 && obj->ps < (dynamic_calibrate - 50)){ 
        	if(obj->ps < 100){			
        		atomic_set(&obj->ps_thd_val_high,  obj->ps+100);
        		atomic_set(&obj->ps_thd_val_low, obj->ps+50);
        	}else if(obj->ps < 200){
        		atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
        		atomic_set(&obj->ps_thd_val_low, obj->ps+60);
        	}else if(obj->ps < 300){
        		atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
        		atomic_set(&obj->ps_thd_val_low, obj->ps+60);
        	}else if(obj->ps < 400){
        		atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
        		atomic_set(&obj->ps_thd_val_low, obj->ps+60);
        	}else if(obj->ps < 600){
        		atomic_set(&obj->ps_thd_val_high,  obj->ps+180);
        		atomic_set(&obj->ps_thd_val_low, obj->ps+90);
        	}else if(obj->ps < 1000){
        		atomic_set(&obj->ps_thd_val_high,  obj->ps+300);
        		atomic_set(&obj->ps_thd_val_low, obj->ps+180);	
        	}else if(obj->ps < 1250){
        		atomic_set(&obj->ps_thd_val_high,  obj->ps+400);
        		atomic_set(&obj->ps_thd_val_low, obj->ps+300);
        	}
        	else{
        		atomic_set(&obj->ps_thd_val_high,  1400);
        		atomic_set(&obj->ps_thd_val_low, 1000);        			
        	}
        		
        	dynamic_calibrate = obj->ps;
        }	        
#endif
#ifdef UPDATE_PS_THRESHOLD
		databuf[0] = LTR562_PS_THRES_LOW_0;	
		databuf[1] = (u8)(0 & 0x00FF);
		//databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
		databuf[0] = LTR562_PS_THRES_LOW_1;
		databuf[1] = (u8)((0 & 0xFF00) >> 8);
		//databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
		databuf[0] = LTR562_PS_THRES_UP_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
		databuf[0] = LTR562_PS_THRES_UP_1; 
		databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
		res = i2c_master_send(obj->client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_INTR;
		}
#endif
#else
		cancel_delayed_work(&ltr562_obj->cali_ps_work);
		schedule_delayed_work(&ltr562_obj->cali_ps_work, msecs_to_jiffies(2000));
#endif
	}
	//let up layer to know
	res = ps_report_interrupt_data(value);

EXIT_INTR:	
#ifdef CONFIG_OF
	enable_irq(obj->irq);
#endif
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void ltr562_eint_func(void)
{
	struct ltr562_priv *obj = ltr562_obj;
	if(!obj)
	{
		return;
	}	
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}
#ifdef CONFIG_OF
static irqreturn_t ltr562_eint_handler(int irq, void *desc)
{	
	if (irq_enabled == 2)
	{
		disable_irq_nosync(ltr562_obj->irq);
		ltr562_eint_func();
	}

	return IRQ_HANDLED;
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int ltr562_setup_eint(struct i2c_client *client)
{
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = { 0, 0 };	

	APS_FUN();


	alspsPltFmDev_ltr562 = get_alsps_platformdev();

	/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev_ltr562->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	
	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
	}

	/* eint request */
	if (ltr562_obj->irq_node) {
		of_property_read_u32_array(ltr562_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		ltr562_obj->irq = irq_of_parse_and_map(ltr562_obj->irq_node, 0);
		APS_LOG("ltr562_obj->irq = %d\n", ltr562_obj->irq);
		if (!ltr562_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		
		if (request_irq(ltr562_obj->irq, ltr562_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq_wake(ltr562_obj->irq);
		//enable_irq(ltr562_obj->irq);
		irq_enabled = 1;
	}
	else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	return 0;
}
/**********************************************************************************************/

/*-------------------------------MISC device related------------------------------------------*/
static int ltr562_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr562_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int ltr562_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long ltr562_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr562_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_cali;
	int threshold[2];
	APS_DBG("cmd= %d\n", cmd); 
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			err = ltr562_ps_enable(obj->client, enable);
			if (err < 0)
			{
				APS_ERR("enable ps fail: %d en: %d\n", err, enable);
				goto err_out;
			}
			if (enable)
				set_bit(CMC_BIT_PS, &obj->enable);
			else
				clear_bit(CMC_BIT_PS, &obj->enable);				
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			APS_DBG("ALSPS_GET_PS_DATA\n"); 
			obj->ps = ltr562_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}
			
			dat = ltr562_get_ps_value(obj, obj->ps);
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			obj->ps = ltr562_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}
			dat = obj->ps;
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			err = ltr562_als_enable(obj->client, enable);
			if (err < 0)
			{
				APS_ERR("enable als fail: %d en: %d\n", err, enable);
				goto err_out;
			}
			if (enable)
				set_bit(CMC_BIT_ALS, &obj->enable);
			else
				clear_bit(CMC_BIT_ALS, &obj->enable);
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			obj->als = ltr562_als_read(obj->client, &obj->als);
			if (obj->als < 0)
			{
				goto err_out;
			}

			dat = ltr562_get_als_value(obj, obj->als);
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			obj->als = ltr562_als_read(obj->client, &obj->als);
			if (obj->als < 0)
			{
				goto err_out;
			}

			dat = obj->als;
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			obj->ps = ltr562_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}
			if(obj->ps > atomic_read(&obj->ps_thd_val_low))
				dat = 1;
			else	
				dat = 0;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}				   
			break;

		case ALSPS_IOCTL_CLR_CALI:
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;
			break;

		case ALSPS_IOCTL_GET_CALI:
			ps_cali = obj->ps_cali ;
			if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_SET_CALI:
			if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
			obj->ps_cali = ps_cali;
			break;
		
		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

			ltr562_ps_set_thres();
			break;
				
		case ALSPS_GET_PS_THRESHOLD_HIGH:
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
				
		case ALSPS_GET_PS_THRESHOLD_LOW:
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
/*------------------------------------------------------------------------------------------*/

		default:
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations ltr562_fops = {
	.owner = THIS_MODULE,
	.open = ltr562_open,
	.release = ltr562_release,
	.unlocked_ioctl = ltr562_unlocked_ioctl,
};

static struct miscdevice ltr562_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr562_fops,
};

/*--------------------------------------------------------------------------------*/
static int ltr562_sensor_init(void)
{
	int res;	
	u8 buf;
	struct i2c_client *client = ltr562_obj->client;
	struct ltr562_priv *obj = ltr562_obj;
	
	/* ===============
	* ** IMPORTANT **
	* ===============
	* Other settings like timing and threshold to be set here, if required.
	* Not set and kept as device default for now.
	*/
	buf = 0x07;
	res = ltr562_master_send(client, LTR562_IR_AMBIENT_THRES, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr562_init_client() IR AMBIENT THRESHOLD setting error...\n");
		goto EXIT_ERR;
	}

	buf = 0x10;
	res = ltr562_master_send(client, LTR562_DSS_CTRL, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr562_init_client() DSS CONTROL setting error...\n");
		goto EXIT_ERR;
	}

	buf = 0x00;
	res = ltr562_master_send(client, LTR562_LED_DRIVE, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr562_init_client() LED DRIVE setting error...\n");
		goto EXIT_ERR;
	}	

#ifdef PS_USE_DRIVER_AVE
	buf = 0x07; // 8 pulses & no averaging
#else
	buf = PS_DEF_AVG_PULSE;	
#endif	
	res = ltr562_master_send(client, LTR562_PS_PULSES, (char *)&buf, 1);	
	if (res<0)
	{
		APS_LOG("ltr562_init_client() PS Pulses error...\n");
		goto EXIT_ERR;
	}

	buf = 0x7d;	// 32us & 100mA 
	res = ltr562_master_send(client, LTR562_PS_LED, (char *)&buf, 1);	
	if (res<0)
	{
		APS_LOG("ltr562_init_client() PS LED error...\n");
		goto EXIT_ERR;
	}

	buf = PS_DEF_MEAS_RATE;
	res = ltr562_master_send(client, LTR562_PS_MEAS_RATE, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr562_init_client() PS time error...\n");
		goto EXIT_ERR;
	}

#ifdef PS_USE_XT_OFFSET
	ltr562_ps_set_offset(ps_xt_offset_val);
#endif	

	/*for interrup work mode support */
	if (0 == obj->hw->polling_mode_ps)
	{
		ltr562_ps_set_thres();

		buf = 0x89;
		res = ltr562_master_send(client, LTR562_INT_CFG, (char *)&buf, 1);		
		if (res < 0)
		{
			goto EXIT_ERR;			
		}

		buf = 0x10;
		res = ltr562_master_send(client, LTR562_INT_PST, (char *)&buf, 1);		
		if (res < 0)
		{
			goto EXIT_ERR;
		}
	}

	// Enable ALS to default gain at startup
	buf = ALS_DEF_GAIN_REG | 0x22;
	res = ltr562_master_send(client, LTR562_ALS_CTRL, (char *)&buf, 1);
	als_gainrange = ALS_DEF_GAIN;//Set global variable
	APS_ERR("ALS sensor gainrange %d!\n", als_gainrange);

	buf = ALS_DEF_INT_MEAS | 0xa0;	
	res = ltr562_master_send(client, LTR562_ALS_INT_TIME, (char *)&buf, 1);
	APS_ERR("ALS sensor integration & measurement rate: %d!\n", ALS_DEF_INT_MEAS);
	als_intfac = 1;
	return 0;

EXIT_ERR:
	APS_ERR("sensor init: %d\n", res);
	return 1;
}

static int ltr562_init_client(void)
{
	int res;		
	struct i2c_client *client = ltr562_obj->client;	

	mdelay(PON_DELAY);
	
	res = ltr562_sensor_init();
	if (res != 0)
	{
		APS_LOG("ltr562_init_client() ltr562_sensor_init error...\n");
		goto EXIT_ERR;
	}
		
	ltr562_get_efuse_data();
	
#ifdef SENSOR_DEFAULT_ENABLED
	res = ltr562_ps_enable(client, 1);
	if (res < 0)
	{
		APS_ERR("enable ps fail: %d\n", res);
		goto EXIT_ERR;
	}

	res = ltr562_als_enable(client, 1);
	if (res < 0)
	{
		APS_ERR("enable als fail: %d\n", res);
		goto EXIT_ERR;
	}
#endif
	if ((res = ltr562_setup_eint(client)) != 0)
	{
		APS_ERR("setup eint: %d\n", res);
		goto EXIT_ERR;
	}

	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return 1;
}
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("ltr562_obj als enable value = %d\n", en);

	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return -1;
	}	

	res = ltr562_als_enable(ltr562_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	mutex_lock(&ltr562_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr562_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr562_obj->enable);
	mutex_unlock(&ltr562_mutex);	

#ifdef REPORT_PS_ONCE_WHEN_ALS_ENABLED
	cancel_delayed_work(&ltr562_obj->check_ps_work);
	schedule_delayed_work(&ltr562_obj->check_ps_work, msecs_to_jiffies(300));
#endif

	return 0;
}

static int als_set_delay(u64 ns)
{
	// Do nothing
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static int als_get_data(int* value, int* status)
{
	int err = 0;
	
	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return -1;
	}

	ltr562_obj->als = ltr562_als_read(ltr562_obj->client, &ltr562_obj->als);
	if (ltr562_obj->als < 0)
		err = -1;
	else {
		*value = ltr562_get_als_value(ltr562_obj, ltr562_obj->als);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("ltr562_obj ps enable value = %d\n", en);

	if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return -1;
	}
	
	res = ltr562_ps_enable(ltr562_obj->client, en);
	if (res < 0) {
		APS_ERR("ps_enable_nodata is failed!!\n");
		return -1;
	}

	mutex_lock(&ltr562_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr562_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &ltr562_obj->enable);
	mutex_unlock(&ltr562_mutex);
	
	return 0;
}

static int ps_set_delay(u64 ns)
{
	// Do nothing
	return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;

    if(!ltr562_obj)
	{
		APS_ERR("ltr562_obj is null!!\n");
		return -1;
	}
    
	ltr562_obj->ps = ltr562_ps_read(ltr562_obj->client, &ltr562_obj->ps);
	if (ltr562_obj->ps < 0)
		err = -1;
	else {
		*value = ltr562_get_ps_value(ltr562_obj, ltr562_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
    
	return err;
}
/*-----------------------------------------------------------------------------------*/

/*-----------------------------------i2c operations----------------------------------*/
static int ltr562_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr562_priv *obj = NULL;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	int err = 0;

	APS_FUN();
	/* get customization and power on */
	err = get_alsps_dts_func(client->dev.of_node, hw);
	if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		return -EFAULT;
	}

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr562_obj = obj;
	
	obj->hw = hw;
	INIT_WORK(&obj->eint_work, ltr562_eint_work);	
#ifdef REPORT_PS_ONCE_WHEN_ALS_ENABLED
	INIT_DELAYED_WORK(&obj->check_ps_work, ltr562_check_ps_work);
#endif
#ifdef DELAYED_PS_CALI
	INIT_DELAYED_WORK(&obj->cali_ps_work, ltr562_cali_ps_work);
#endif

	obj->client = client;
	i2c_set_clientdata(client, obj);	
	
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	//atomic_set(&obj->als_cmd_val, 0xDF);
	//atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);
	//obj->irq_node = client->dev.of_node;
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek,als_ps");

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100	
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);

#ifdef SENSOR_DEFAULT_ENABLED
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);
#else
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);
#endif

	APS_LOG("ltr562_init_client() start...!\n");
	ltr562_i2c_client = client;
	err = ltr562_init_client();
	if(err)
	{
		goto exit_init_failed;
	}
	APS_LOG("ltr562_init_client() OK!\n");
	
	err = misc_register(&ltr562_device);
	if(err)
	{
		APS_ERR("ltr562_device register failed\n");
		goto exit_misc_device_register_failed;
	}

    als_ctl.is_use_common_factory =false;
	ps_ctl.is_use_common_factory = false;
	
	/*------------------------ltr562 attribute file for debug--------------------------------------*/
	//err = ltr562_create_attr(&(ltr562_init_info.platform_diver_addr->driver));
	err = ltr562_create_attr(&(ltr562_i2c_driver.driver));
	if(err)
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------ltr562 attribute file for debug--------------------------------------*/
	
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;
 	
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	
	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	ps_ctl.is_polling_mode = obj->hw->polling_mode_ps;

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}	

	ltr562_init_flag =0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
	misc_deregister(&ltr562_device);
exit_init_failed:
	kfree(obj);
exit:
	ltr562_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr562_init_flag =-1;
	return err;
}

static int ltr562_i2c_remove(struct i2c_client *client)
{
	int err;

	//err = ltr562_delete_attr(&(ltr562_init_info.platform_diver_addr->driver));
	err = ltr562_delete_attr(&(ltr562_i2c_driver.driver));
	if(err)
	{
		APS_ERR("ltr562_delete_attr fail: %d\n", err);
	}

	misc_deregister(&ltr562_device);
/*
	err = misc_deregister(&ltr562_device);
	if(err)
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	*/	
	ltr562_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int ltr562_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, LTR562_DEV_NAME);
	return 0;
}

static int ltr562_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr562_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
		
	atomic_set(&obj->als_suspend, 1);
	err = ltr562_als_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable als: %d\n", err);
		return err;
	}

#ifndef NO_PS_CTRL_WHEN_SUSPEND_RESUME
	atomic_set(&obj->ps_suspend, 1);
	err = ltr562_ps_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable ps:  %d\n", err);
		return err;
	}
		
	ltr562_power(obj->hw, 0);
#endif

	return 0;
}

static int ltr562_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr562_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
#ifndef NO_PS_CTRL_WHEN_SUSPEND_RESUME
	ltr562_power(obj->hw, 1);

	atomic_set(&obj->ps_suspend, 0);
	if (test_bit(CMC_BIT_PS, &obj->enable))
	{
		err = ltr562_ps_enable(obj->client, 1);
		if (err < 0)
		{
			APS_ERR("enable ps fail: %d\n", err);
		}
	}
#endif
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		err = ltr562_als_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int ltr562_remove(void)
{
	APS_FUN();

	ltr562_power(hw, 0);	
	i2c_del_driver(&ltr562_i2c_driver);
	ltr562_init_flag = -1;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int  ltr562_local_init(void)
{
	APS_FUN();

	ltr562_power(hw, 1);
	
	if(i2c_add_driver(&ltr562_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if(-1 == ltr562_init_flag)
	{
	   return -1;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init ltr562_init(void)
{
	alsps_driver_add(&ltr562_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr562_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
module_init(ltr562_init);
module_exit(ltr562_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Liteon");
MODULE_DESCRIPTION("LTR-562ALSPS Driver");
MODULE_LICENSE("GPL");

