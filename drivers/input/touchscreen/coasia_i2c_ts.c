/* drivers/input/touchscreen/pixcir_i2c_ts.c
 *
 *  Coasia Microelectronics Corp.
 *
 *  Release date: 2011/06/15
 *
 * pixcir_i2c_ts.c V1.0  support multi touch
 * pixcir_i2c_ts.c V2.0  add tuning function including follows function:
 *
 * CALIBRATION_FLAG	1
 * NORMAL_MODE		2
 * DEBUG_MODE		3
 * INTERNAL_MODE	4
 * RASTER_MODE		5
 * VERSION_FLAG		6
 * BOOTLOADER_MODE	7
 * EE_SLAVE_READ
 * RAM_SLAVE_READ
 * POWER
 * WRITE_EE2SLAVE
 * SET_OFFSET_FLAG
 * CLEAR_SPECOP
 * CRC_FLAG
 * RASTER_MODE_1
 * RASTER_MODE_2
 * READ_XN_YN_FLAG
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/slab.h>  /* for mini6410 2.6.36 kree(),kmalloc() */
#include <linux/delay.h>
#include <linux/irq.h>

#include <plat/gpio-cfg.h>

#include <mach/gpio.h>
#include <mach/irqs.h>

#define R8C_3GA_2TG

#ifdef R8C_AUO_I2C
  #ifndef R8C_3GA_2TG
  #define R8C_3GA_2TG
  #endif
#endif

#define TOUCHSCREEN_MINX 0
#define TOUCHSCREEN_MAXX 2560
#define TOUCHSCREEN_MINY 0
#define TOUCHSCREEN_MAXY 1600

#define ATTB		EXYNOS5_GPX2(5)
#define get_attb_value	gpio_get_value
#define TS_RST		EXYNOS5_GPX2(4)
#define RESETPIN_CFG	s3c_gpio_cfgpin(TS_RST, S3C_GPIO_OUTPUT)
#define RESETPIN_SET0	gpio_direction_output(TS_RST, 0)
#define RESETPIN_SET1	gpio_direction_output(TS_RST, 1)

#define PIXCIR_DEBUG		0

/* Touch Finger Numbers */
#define NO_TOUCH		0x0
#define ONE_FINGER_TOUCH	0x1
#define TWO_FINGER_TOUCH	0x2
#define THREE_FINGER_TOUCH	0x3
#define FOUR_FINGER_TOUCH	0x4

#define SLAVE_ADDR		0x5c
#define BOOTLOADER_ADDR		0x5d

#ifndef I2C_MAJOR
#define I2C_MAJOR		125
#endif

#define I2C_MINORS		256

#define CALIBRATION_FLAG	1
#define NORMAL_MODE		8
#define PIXCIR_DEBUG_MODE	3
#define INTERNAL_MODE		4
#define RASTER_MODE		5
#define VERSION_FLAG		6
#define BOOTLOADER_MODE		7

#define ENABLE_IRQ		10
#define DISABLE_IRQ		11

#define SPECOP			0x37
#define reset

int global_irq;

static unsigned char status_reg;
unsigned char read_XN_YN_flag;

unsigned char global_touching, global_oldtouching;
unsigned char global_posx1_low, global_posx1_high, global_posy1_low,
		global_posy1_high, global_posx2_low, global_posx2_high,
		global_posy2_low, global_posy2_high;

unsigned char Tango_number;

unsigned char interrupt_flag;

unsigned char x_nb_electrodes;
unsigned char y_nb_electrodes;
unsigned char x2_nb_electrodes;
unsigned char x1_x2_nb_electrodes;

signed char xy_raw1[(TOUCHSCREEN_MAXX*2+3)];
signed char xy_raw2[TOUCHSCREEN_MAXX*2];
signed char xy_raw12[(TOUCHSCREEN_MAXX*4+3)];

struct i2c_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

struct finger_info {
	int id;
	int status;
	int pos_x;
	int pos_y;
};

static struct i2c_driver pixcir_i2c_ts_driver;
static struct class *i2c_dev_class;
static LIST_HEAD(i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	i2c_dev = NULL;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list)
	{
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;

found:
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
				adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

void read_XN_YN_value(struct i2c_client *client)
{
	char Wrbuf[4], Rdbuf[2];
	int ret;

	memset(Wrbuf, 0, sizeof(Wrbuf));
	memset(Rdbuf, 0, sizeof(Rdbuf));

	Wrbuf[0] = SPECOP;	/* specop addr */
	Wrbuf[1] = 1;		/* write 1 to read eeprom */
	Wrbuf[2] = 64;
	Wrbuf[3] = 0;
	ret = i2c_master_send(client, Wrbuf, 4);
	if (ret != 4)
		printk(KERN_DEBUG "send ret = %d\n", ret);
	mdelay(8);
	ret = i2c_master_recv(client, Rdbuf, 2);
	if (ret != 2)
		printk(KERN_DEBUG "recv ret = %d\n", ret);
	x_nb_electrodes = Rdbuf[0];

	if (Tango_number == 1) {
		x2_nb_electrodes = 0;

		memset(Wrbuf, 0, sizeof(Wrbuf));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		Wrbuf[0] = SPECOP;	/* specop addr */
		Wrbuf[1] = 1;		/* write to eeprom */
		Wrbuf[2] = 203;
		Wrbuf[3] = 0;
		ret = i2c_master_send(client, Wrbuf, 4);
		mdelay(4);

		ret = i2c_master_recv(client, Rdbuf, 2);
		if (ret != 2)
			printk(KERN_DEBUG "recv y_nb, ret = %d\n", ret);
		y_nb_electrodes = Rdbuf[0];
	} else if (Tango_number == 2) {
		memset(Wrbuf, 0, sizeof(Wrbuf));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		Wrbuf[0] = SPECOP;	/* specop addr */
		Wrbuf[1] = 1;		/* write to eeprom */
	#ifdef R8C_3GA_2TG
		Wrbuf[2] = 151;
		Wrbuf[3] = 0;
	#endif

	#ifdef	R8C_AUO_I2C
		Wrbuf[2] = 211;
		Wrbuf[3] = 0;
	#endif

		ret = i2c_master_send(client, Wrbuf, 4);
		mdelay(4);
		ret = i2c_master_recv(client, Rdbuf, 2);
		x2_nb_electrodes = Rdbuf[0];

		memset(Wrbuf, 0, sizeof(Wrbuf));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		Wrbuf[0] = SPECOP;	/* specop addr */
		Wrbuf[1] = 1;		/* write to eeprom */
	#ifdef R8C_3GA_2TG
		Wrbuf[2] = 238;
		Wrbuf[3] = 0;
	#endif

	#ifdef	R8C_AUO_I2C
		Wrbuf[2] = 151;
		Wrbuf[3] = 0;
	#endif
		ret = i2c_master_send(client, Wrbuf, 4);
		mdelay(4);

		ret = i2c_master_recv(client, Rdbuf, 2);
		y_nb_electrodes = Rdbuf[0];
	}

	if (x2_nb_electrodes)
		x1_x2_nb_electrodes = x_nb_electrodes + x2_nb_electrodes - 1;
	else
		x1_x2_nb_electrodes = x_nb_electrodes;

	read_XN_YN_flag = 1;
}

void read_XY_tables(struct i2c_client *client, signed char *xy_raw1_buf,
		signed char *xy_raw2_buf)
{
	u_int8_t Wrbuf[1];
	int ret;

	memset(Wrbuf, 0, sizeof(Wrbuf));
	#ifdef R8C_AUO_I2C
	/* xy_raw1[0] rawdata X register address for AUO */
	Wrbuf[0] = 128;
	#endif

	#ifdef R8C_3GA_2TG
	/* xy_raw1[0] rawdata X register address for PIXCIR R8C_3GA_2TG */
	Wrbuf[0] = 61;
	#endif
	ret = i2c_master_send(client, Wrbuf, 1);
	if (ret != 1)
		printk(KERN_DEBUG "send xy_raw1[0] register address error\
			in read_XY_tables function ret = %d\n", ret);

	ret = i2c_master_recv(client, xy_raw1_buf, (TOUCHSCREEN_MAXX-1)*2);
	if (ret != (TOUCHSCREEN_MAXX-1)*2)
		printk(KERN_DEBUG "receive xy_raw1 error\
			in read_XY_tables function ret = %d\n", ret);

	#ifdef R8C_AUO_I2C
	/* xy_raw2[0] rawdata Y register address for AUO */
	Wrbuf[0] = 43;
	#endif

	#ifdef R8C_3GA_2TG
	/* xy_raw2[0] rawdata Y register address for PIXCIR R8C_3GA_2TG */
	Wrbuf[0] = 125;
	#endif

	ret = i2c_master_send(client, Wrbuf, 1);
	if (ret != 1)
		printk(KERN_DEBUG "send xy_raw2[0] register address error\
			in read_XY_tables function ret = %d\n", ret);

	ret = i2c_master_recv(client, xy_raw2_buf, (TOUCHSCREEN_MAXX-1)*2);
	if (ret != (TOUCHSCREEN_MAXX-1)*2)
		printk(KERN_DEBUG "receive xy_raw2 error\
			in read_XY_tables function ret = %d\n", ret);

}

static struct workqueue_struct *pixcir_wq;

struct pixcir_i2c_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	int irq;
};

static void reset_touch_hw_init(void)
{
	/* config for reset pin */
	if (gpio_request(TS_RST, "TS_RST"))	{
		printk(KERN_ERR "%s :TS_RST request port error!\n", __func__);
	} else {
		s3c_gpio_setpull(TS_RST, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(TS_RST, S3C_GPIO_SFN(1));
		gpio_direction_output(TS_RST, 1);

		msleep(2);

		gpio_direction_output(TS_RST, 0);
		gpio_free(TS_RST);
		printk(KERN_ERR "pixcir:TS_RST request port OK\n");
	}
}

static void pixcir_ts_poscheck(struct work_struct *work)
{
	struct pixcir_i2c_ts_data *tsdata =
		container_of(work, struct pixcir_i2c_ts_data, work.work);

	unsigned char touch_count;
	struct finger_info finger[4];
	unsigned char Rdbuf[33], Wrbuf[1];
	int ret;
	int z = 50;
	int w = 15;

	printk(KERN_DEBUG "pixcir:pixcir_ts_poscheck start\n");

	interrupt_flag = 1;
	memset(finger, 0, sizeof(finger));
	memset(Wrbuf, 0, sizeof(Wrbuf));
	memset(Rdbuf, 0, sizeof(Rdbuf));

	Wrbuf[0] = 0;
	ret = i2c_master_send(tsdata->client, Wrbuf, 1);

#if PIXCIR_DEBUG
	printk(KERN_DEBUG "master send ret:%d\n", ret);
#endif

	if (ret != 1) {
		dev_err(&tsdata->client->dev,
			"Unable to write to i2c, ret =%d\n", ret);
		/* config for reset pin */
		reset_touch_hw_init();
		goto out;
	}

	/* Read data from 0 to 32 */
	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));

	if (ret != sizeof(Rdbuf)) {
		dev_err(&tsdata->client->dev,
			"Unable to read i2c page, ret = %d\n", ret);
		/* config for reset pin */
		reset_touch_hw_init();
		goto out;
	}

	touch_count = Rdbuf[0]; /* Number of fingers touching (up/down) */
	finger[0].id = ((Rdbuf[1] & 0xF0) >> 4);
	finger[0].status = (Rdbuf[1] & 0x0F);
	finger[0].pos_x = ((Rdbuf[3] << 8) | Rdbuf[2]);
	finger[0].pos_y = ((Rdbuf[5] << 8) | Rdbuf[4]);
	finger[1].id = ((Rdbuf[10] & 0xF0) >> 4);
	finger[1].status = (Rdbuf[10] & 0x0F);
	finger[1].pos_x = ((Rdbuf[12] << 8) | Rdbuf[11]);
	finger[1].pos_y = ((Rdbuf[14] << 8) | Rdbuf[13]);
	finger[2].id = ((Rdbuf[19] & 0xF0) >> 4);
	finger[2].status = (Rdbuf[19] & 0x0F);
	finger[2].pos_x = ((Rdbuf[21] << 8) | Rdbuf[20]);
	finger[2].pos_y = ((Rdbuf[23] << 8) | Rdbuf[22]);
	finger[3].id = ((Rdbuf[28] & 0xF0) >> 4);
	finger[3].status = (Rdbuf[28] & 0x0F);
	finger[3].pos_x = ((Rdbuf[30] << 8) | Rdbuf[29]);
	finger[3].pos_y = ((Rdbuf[32] << 8) | Rdbuf[31]);

#if PIXCIR_DEBUG
	printk(KERN_ERR "New_count:%d, fineger_info(status, posx, posy):\
		(%d,%4d,%4d), (%d,%4d,%4d), (%d,%4d,%4d), (%d,%4d,%4d)\n",
		touch_count,
		finger[0].status, finger[0].pos_x, finger[0].pos_y,
		finger[1].status, finger[1].pos_x, finger[1].pos_y,
		finger[2].status, finger[2].pos_x, finger[2].pos_y,
		finger[3].status, finger[3].pos_x, finger[3].pos_y);
#endif
	if (touch_count) {
		input_report_abs(tsdata->input, ABS_X, finger[0].pos_x);
		input_report_abs(tsdata->input, ABS_Y, finger[0].pos_y);
		input_report_key(tsdata->input, BTN_TOUCH, 1);
		input_report_key(tsdata->input, ABS_PRESSURE, 1);
	} else {
		input_report_key(tsdata->input, BTN_TOUCH, 0);
		input_report_abs(tsdata->input, ABS_PRESSURE, 0);
	}

	switch (touch_count) {
	case FOUR_FINGER_TOUCH:
		if ((finger[0].status == 0x1) &&
			(finger[1].status == 0x1) &&
			(finger[2].status == 0x1) &&
			(finger[3].status == 0x1)) {
			input_mt_sync(tsdata->input);
			printk(KERN_DEBUG "NO fingers touch now!\n");
			goto fun_end;
		} else {
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(tsdata->input,	ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(tsdata->input,
					ABS_MT_POSITION_X, finger[3].pos_x);
			input_report_abs(tsdata->input,
					ABS_MT_POSITION_Y, finger[3].pos_y);
			input_mt_sync(tsdata->input);
		}
	case THREE_FINGER_TOUCH:
		if ((finger[0].status == 0x1) &&
			(finger[1].status == 0x1) &&
			(finger[2].status == 0x1)) {
			input_mt_sync(tsdata->input);
			printk(KERN_DEBUG "NO fingers touch now!\n");
			goto fun_end;
		} else {
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(tsdata->input,
					ABS_MT_POSITION_X, finger[2].pos_x);
			input_report_abs(tsdata->input,
					ABS_MT_POSITION_Y, finger[2].pos_y);
			input_mt_sync(tsdata->input);
		}
	case TWO_FINGER_TOUCH:
		if ((finger[0].status == 0x1) &&
			(finger[1].status == 0x1)) {
			input_mt_sync(tsdata->input);
			printk(KERN_DEBUG "NO fingers touch now!\n");
			goto fun_end;
		} else {
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(tsdata->input,
					ABS_MT_POSITION_X, finger[1].pos_x);
			input_report_abs(tsdata->input,
					ABS_MT_POSITION_Y, finger[1].pos_y);
			input_mt_sync(tsdata->input);
		}
	case ONE_FINGER_TOUCH:
		if (finger[0].status == 0x1) {
			input_mt_sync(tsdata->input);
			printk(KERN_DEBUG "NO fingers touch now!\n");
			goto fun_end;
		} else {
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(tsdata->input,
					ABS_MT_POSITION_X, finger[0].pos_x);
			input_report_abs(tsdata->input,
					ABS_MT_POSITION_Y, finger[0].pos_y);
			input_mt_sync(tsdata->input);
		}
		break;
	default:
		printk(KERN_ERR "touch_count > 4, NOT report\n");
		break;
	}
	/* sync after groups of events */
fun_end:
	input_sync(tsdata->input);
out:
	enable_irq(tsdata->irq);
	printk(KERN_DEBUG "pixcir:pixcir_ts_poscheck end\n");
}

static irqreturn_t pixcir_ts_isr(int irq, void *dev_id)
{
	struct pixcir_i2c_ts_data *tsdata = dev_id;

#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_ts_isr+\n");
#endif

	if ((status_reg == 0) || (status_reg == NORMAL_MODE)) {
		disable_irq_nosync(irq);
		queue_work(pixcir_wq, &tsdata->work.work);
	}

#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_ts_isr-\n");
#endif

	return IRQ_HANDLED;
}

static int pixcir_ts_open(struct input_dev *dev)
{
#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_ts_open\n");
#endif
	return 0;
}

static void pixcir_ts_close(struct input_dev *dev)
{
#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_ts_close\n");
#endif
}

static int pixcir_i2c_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct pixcir_i2c_ts_data *tsdata;
	struct input_dev *input;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int error;

#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_i2c_ts_probe\n");
#endif
	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data!\n");
		error = -ENOMEM;
		dev_set_drvdata(&client->dev, NULL);
		return error;
	}

	dev_set_drvdata(&client->dev, tsdata);

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device!\n");
		error = -ENOMEM;
		input_free_device(input);
		kfree(tsdata);
	}

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);		/* Support Key func */
	set_bit(EV_ABS, input->evbit);		/* Support Touch */
	set_bit(BTN_TOUCH, input->keybit);	/* Single Touch */
	input_set_abs_params(input, ABS_X,
		TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_Y,
		TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X,
		TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
		TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,	0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR,	0, 25, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID,	0, 4, 0, 0);

	/* input->name = client->name; pixcir_ts */
	input->name = "egalax_i2c";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = pixcir_ts_open;
	input->close = pixcir_ts_close;

	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input = input;

	INIT_WORK(&tsdata->work.work, pixcir_ts_poscheck);

	tsdata->irq = client->irq;
	global_irq = client->irq;

	if (input_register_device(input)) {
		input_free_device(input);
		kfree(tsdata);
	}

	if (gpio_request(TS_RST, "GPX2")) {
		error = -EIO;
		return error;
	}

	RESETPIN_CFG;
	RESETPIN_SET0;
	mdelay(20);
	RESETPIN_SET1;

	mdelay(30);

	if (request_irq(tsdata->irq, pixcir_ts_isr,
		IRQF_TRIGGER_FALLING, client->name, tsdata)) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input = NULL;
	}

	device_init_wakeup(&client->dev, 1);

	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}

	dev = device_create(i2c_dev_class, &client->adapter->dev,
		MKDEV(I2C_MAJOR, client->adapter->nr), NULL,
			"pixcir_i2c_ts%d", client->adapter->nr);

	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		return error;
	}

	dev_err(&tsdata->client->dev, "insmod successfully!\n");
	return 0;

}

static int pixcir_i2c_ts_remove(struct i2c_client *client)
{
	int error;
	struct i2c_dev *i2c_dev;
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_i2c_ts_remove\n");
#endif

	free_irq(tsdata->irq, tsdata);
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}
	return_i2c_dev(i2c_dev);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, client->adapter->nr));
	input_unregister_device(tsdata->input);
	kfree(tsdata);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static int pixcir_i2c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(tsdata->irq);

	return 0;
}

static int pixcir_i2c_ts_resume(struct i2c_client *client)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tsdata->irq);

	return 0;
}

static int pixcir_open(struct inode *inode, struct file *file)
{
	int subminor;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;
	int ret = 0;

#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir_open function\n");
#endif

	subminor = iminor(inode);

#if PIXCIR_DEBUG
	printk(KERN_DEBUG "subminor=%d\n", subminor);
#endif

	i2c_dev = i2c_dev_get_by_minor(subminor);
	if (!i2c_dev) {
		printk(KERN_ERR "error i2c_dev_get_by_minor\n");
		return -ENODEV;
	}

	adapter = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adapter) {
		printk(KERN_ERR "error i2c_get_adapter\n");
		return -ENODEV;
	}

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		i2c_put_adapter(adapter);
		ret = -ENOMEM;
	}

	snprintf(client->name, I2C_NAME_SIZE, "pixcir_i2c_ts%d", adapter->nr);
	client->driver = &pixcir_i2c_ts_driver;
	client->adapter = adapter;
	file->private_data = client;
#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_open OK\n");
#endif
	return 0;
}

static long pixcir_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *) file->private_data;
#if PIXCIR_DEBUG
	printk(KERN_DEBUG "cmd = %d,arg = %d\n", cmd, arg);
	printk(KERN_DEBUG "pixcir:pixcir_ioctl\n");
#endif

	switch (cmd) {
	case CALIBRATION_FLAG: /* CALIBRATION_FLAG = 1 */
#if PIXCIR_DEBUG
		printk(KERN_DEBUG "CALIBRATION\n");
#endif
		client->addr = SLAVE_ADDR;
		status_reg = 0;
		status_reg = CALIBRATION_FLAG;
		break;

	case NORMAL_MODE:
		client->addr = SLAVE_ADDR;
#if PIXCIR_DEBUG
		printk(KERN_DEBUG "NORMAL_MODE\n");
#endif
		status_reg = 0;
		status_reg = NORMAL_MODE;
		break;

	case PIXCIR_DEBUG_MODE:
		client->addr = SLAVE_ADDR;
#if PIXCIR_DEBUG
		printk(KERN_DEBUG "PIXCIR_DEBUG_MODE\n");
#endif
		status_reg = 0;
		status_reg = PIXCIR_DEBUG_MODE;

		Tango_number = arg;
		#ifdef R8C_3GA_2TG
		Tango_number = 2;
		#endif
		break;

	case BOOTLOADER_MODE: /* BOOTLOADER_MODE = 7 */

#if PIXCIR_DEBUG
		printk(KERN_DEBUG "BOOTLOADER_MODE\n");
#endif
		status_reg = 0;
		status_reg = BOOTLOADER_MODE;
		disable_irq_nosync(global_irq);

	#ifdef reset
		client->addr = BOOTLOADER_ADDR;

		RESETPIN_CFG;
		RESETPIN_SET0;
		mdelay(20);
		RESETPIN_SET1;

		#ifdef R8C_3GA_2TG
		mdelay(50);
		#else
		mdelay(30);
		#endif

	#else		/* normal */
		client->addr = SLAVE_ADDR;
		tmp[0] = SPECOP;	/* specop addr */
		tmp[1] = 5;		/* change to bootloader */
		ret = i2c_master_send(client, tmp, 2);

		client->addr = BOOTLOADER_ADDR;
	#endif
		break;

		case ENABLE_IRQ:
			enable_irq(global_irq);
		break;

		case DISABLE_IRQ:
			disable_irq_nosync(global_irq);
		break;

	default:
		break; /* return -ENOTTY; */
	}
	return 0;
}

static ssize_t pixcir_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	int ret = 0;
	unsigned char normal_tmp[10];
#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_read\n");
#endif
	switch (status_reg) {
	case NORMAL_MODE:
		memset(normal_tmp, 0, sizeof(normal_tmp));
		if (interrupt_flag) {
			normal_tmp[0] = global_touching;
			normal_tmp[1] = global_oldtouching;
			normal_tmp[2] = global_posx1_low;
			normal_tmp[3] = global_posx1_high;
			normal_tmp[4] = global_posy1_low;
			normal_tmp[5] = global_posy1_high;
			normal_tmp[6] = global_posx2_low;
			normal_tmp[7] = global_posx2_high;
			normal_tmp[8] = global_posy2_low;
			normal_tmp[9] = global_posy2_high;
			printk(KERN_DEBUG "global_touching:%-3d,global_oldtouching:%-3d\n",
					normal_tmp[0], normal_tmp[1]);
			ret = copy_to_user(buf, normal_tmp, 10);
			if (ret != 10)
				printk(KERN_DEBUG "interrupt_flag = %d,\
					NORMAL_MODE, copy_to_user error,\
					ret = %d\n", interrupt_flag, ret);
		}
		interrupt_flag = 0;
		break;
	case PIXCIR_DEBUG_MODE:
		if (read_XN_YN_flag == 0) {
			unsigned char buf[2];
			memset(buf, 0, sizeof(buf));
			read_XN_YN_value(client);
			#ifdef R8C_AUO_I2C
			#else
			buf[0] = 194;
			buf[1] = 0;
			ret = i2c_master_send(client, buf, 2);
			if (ret != 2)
				printk(KERN_DEBUG "PIXCIR_DEBUG_MODE,\
					master send %d,%d error, ret = %d\n",
					buf[0], buf[1], ret);

			#endif
		} else {
			memset(xy_raw1, 0, sizeof(xy_raw1));
			memset(xy_raw2, 0, sizeof(xy_raw2));
			read_XY_tables(client, xy_raw1, xy_raw2);
		}

		if (Tango_number == 1) {
			xy_raw1[TOUCHSCREEN_MAXX*2] = x_nb_electrodes;
			xy_raw1[TOUCHSCREEN_MAXX*2+1] = y_nb_electrodes;
			if (copy_to_user(buf, xy_raw1, TOUCHSCREEN_MAXX*2+2))
				printk(KERN_DEBUG "PIXCIR_DEBUG_MODE,\
					Tango_number= 1 copy_to_user error,\
					ret = %d\n", ret);
		} else if (Tango_number == 2) {
			xy_raw1[TOUCHSCREEN_MAXX*2] = x_nb_electrodes;
			xy_raw1[TOUCHSCREEN_MAXX*2+1] = y_nb_electrodes;
			xy_raw1[TOUCHSCREEN_MAXX*2+2] = x2_nb_electrodes;

			for (ret = 0; ret < (TOUCHSCREEN_MAXX*2+3); ret++)
				xy_raw12[ret] = xy_raw1[ret];

			for (ret = 0; ret < (TOUCHSCREEN_MAXX*2-1); ret++)
				xy_raw12[(TOUCHSCREEN_MAXX*2+3)+ret] = xy_raw2[ret];

			if (copy_to_user(buf, xy_raw12, TOUCHSCREEN_MAXX*4+3))
				printk(KERN_DEBUG "PIXCIR_DEBUG_MODE,\
				Tango_number= 2 copy_to_user error,\
				ret = %d\n", ret);
		}
		break;
	default:
		break;
	}

	return ret;
}

static ssize_t pixcir_write(struct file *file,
			const char __user *buf, size_t count, loff_t *ppos)
{
	struct i2c_client *client;
	char *tmp, bootload_data[143], Rdbuf[1], Rdverbuf[2];
	static int ret, ret2, stu, re_value;

#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_write\n");
#endif
	client = file->private_data;

	switch (status_reg)	{
	case CALIBRATION_FLAG: /* CALIBRATION_FLAG=1 */
		tmp = kmalloc(count, GFP_KERNEL);
		if (tmp == NULL)
			return -ENOMEM;
		if (copy_from_user(tmp, buf, count)) {
			printk(KERN_DEBUG "copy_from_user error\n");
			kfree(tmp);
			return -EFAULT;
		}
		ret = i2c_master_send(client, tmp, count);
#if PIXCIR_DEBUG
		printk(KERN_DEBUG "CALIBRATION_FLAG,\
			i2c_master_send ret = %d\n", ret);
#endif
		mdelay(100);
		if (ret != count)
			printk(KERN_DEBUG "Unable to write to i2c page for calb!\n");
		kfree(tmp);

		status_reg = 0;
		break;

	case BOOTLOADER_MODE: /* BOOTLOADER_MODE=7 */
#if PIXCIR_DEBUG
		printk(KERN_DEBUG "BOOT ");
#endif
		memset(bootload_data, 0, sizeof(bootload_data));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		if (copy_from_user(bootload_data, buf, count)) {
			printk(KERN_DEBUG "COPY FAIL ");
			return -EFAULT;
		}

#if PIXCIR_DEBUG
		static int i;
		for (i = 0; i < 143; i++) {
			if (bootload_data[i] < 0x10)
				printk(KERN_DEBUG "0%x", bootload_data[i]);
			else
				printk(KERN_DEBUG "%x", bootload_data[i]);
		}
#endif

		stu = bootload_data[0];

		#ifdef R8C_3GA_2TG
		if (stu == 0x01) {
			ret2 = i2c_master_recv(client, Rdverbuf, 2);
			if ((ret2 != 2) || (Rdverbuf[1] != 0xA5)) {
				printk(KERN_DEBUG "i2c_master_recv boot status\
					error ret2=%d,bootloader status=%x",
					ret2, Rdverbuf[1]);
			    ret = 0;
			    break;
			}
			printk(KERN_DEBUG "\n");
			printk(KERN_DEBUG "Bootloader Status:%x%x\n",
				Rdverbuf[0], Rdverbuf[1]);
		}
		#endif

		ret = i2c_master_send(client, bootload_data, count);
		if (ret != count) {
			printk(KERN_DEBUG "bootload 143 bytes error,\
				ret = %d\n", ret);
			break;
		}

		if (stu != 0x01) {
			mdelay(1);
			while (get_attb_value(ATTB))
				;
			mdelay(1);

		#ifdef R8C_3GA_2TG
			if (stu == 0x03) {
				ret2 = i2c_master_recv(client, Rdbuf, 1);
				if (ret2 != 1) {
					ret = 0;
					break;
				}
			}
		#else
			ret2 = i2c_master_recv(client, Rdbuf, 1);
			if (ret2 != 1) {
				ret = 0;
				printk("Read I2C slave error:%d\n", ret2);
				break;

			}
			re_value = Rdbuf[0];
#if PIXCIR_DEBUG
			printk(KERN_DEBUG "re_value = %d\n", re_value);
#endif
		#endif

		} else {
			mdelay(100);
			status_reg = 0;
			enable_irq(global_irq);
		}

		if ((re_value&0x80) && (stu != 0x01)) {
			printk(KERN_DEBUG "Failed:(re_value&0x80)&&(stu!=0x01)=1\n");
			ret = 0;
		}
		break;

		default:
		break;
	}
	return ret;
}

static int pixcir_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;
#if PIXCIR_DEBUG
	printk(KERN_DEBUG "enter pixcir_release funtion\n");
#endif
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

static const struct file_operations pixcir_i2c_ts_fops = {
	.owner = THIS_MODULE,
	.read = pixcir_read,
	.write = pixcir_write,
	.open = pixcir_open,
	.unlocked_ioctl = pixcir_ioctl,
	.release = pixcir_release,
};

static const struct i2c_device_id pixcir_i2c_ts_id[] = {
	{ "pixcir_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pixcir_i2c_ts_id);

static struct i2c_driver pixcir_i2c_ts_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = "pixcir_ts",
		},
	.probe = pixcir_i2c_ts_probe,
	.remove = pixcir_i2c_ts_remove,
	.suspend = pixcir_i2c_ts_suspend,
	.resume = pixcir_i2c_ts_resume,
	.id_table = pixcir_i2c_ts_id,
};

static int __init pixcir_i2c_ts_init(void)
{
	int ret;
#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_i2c_init\n");
#endif
	pixcir_wq = create_singlethread_workqueue("pixcir_wq");
	if (!pixcir_wq)
		return -ENOMEM;
	ret = register_chrdev(I2C_MAJOR, "pixcir_i2c_ts", &pixcir_i2c_ts_fops);
	if (ret) {
		printk(KERN_ERR "%s:register chrdev failed\n", __FILE__);
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE, "pixcir_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	return i2c_add_driver(&pixcir_i2c_ts_driver);
}

static void __exit pixcir_i2c_ts_exit(void)
{
#if PIXCIR_DEBUG
	printk(KERN_DEBUG "pixcir:pixcir_i2c_ts_exit\n");
#endif
	i2c_del_driver(&pixcir_i2c_ts_driver);
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR, "pixcir_i2c_ts");
	if (pixcir_wq)
		destroy_workqueue(pixcir_wq);
}

module_init(pixcir_i2c_ts_init);
module_exit(pixcir_i2c_ts_exit);

MODULE_AUTHOR("Dongsu Ha <dsfine.ha@samsung.com>, "
	      "Bee<http://www.pixcir.com.cn>, "
	      "Samsung Electronics <http://www.samsung.com>");

MODULE_DESCRIPTION("Pixcir I2C Touchscreen Driver with tune fuction");
MODULE_LICENSE("GPL");
