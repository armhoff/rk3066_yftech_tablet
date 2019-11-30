/*
 *  ADC based battery driver
 *
 * Battery driver for YF GPS
 * Copyright (C) 2009 YuanFeng <liqm@yfgps.com>
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <linux/adc.h>
#include <mach/yfmach.h>
static struct adc_client * adc_client;

#ifndef GPIO_VALID
#define GPIO_VALID(x)            ((x) != 0xFFFFFFFF)
#endif

struct adc_item {
	int capacity;
	int voltage;
	struct adc_item * pre;
	struct adc_item * next;
};
struct adc_bat {
	int poll_time;
	int poll_count;
	int poll_sum;
	int poll_index;
	int poll_init;
	struct adc_item * poll_head;
	struct adc_item * poll_items;

	int bat_status;
	int capacity;
	int voltage;
	struct mutex work_lock; /* protects data */
	struct delayed_work bat_work;
	int * v2c_table; //voltage to capaicty table
	int * dc_table;

	int poll_supply;
	int supply_status;
	int running;
	struct delayed_work supply_work;

	int irq1; //ac or status irq
	int irq2; //charge irq
	int irq_type;
	int ac_gpio;
	int ac_level;
	int usb_gpio;
	int usb_level;
	int charge_gpio;
	int charge_level;

	void *private_data;
	int (*init)(void * private_data,int remove); //init or remove driver
	int (*get_voltage)(void * private_data, int discharge);
	int (*get_status)(void * private_data);
	void (*update_capacity)(void * private_data, int capacity);
};

static struct adc_bat bat_data;
static int adc_debug;
static int adc_offset;

static int adc_bat_get_property(struct power_supply *supply_bat,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = (bat_data.bat_status & STATUS_CHARGE_DONE) ? POWER_SUPPLY_STATUS_FULL : 
		              (bat_data.bat_status & STATUS_CHARGING) ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat_data.capacity + adc_offset;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat_data.voltage*1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4300*1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat_data.v2c_table[0] * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (bat_data.supply_status & STATUS_AC_IN) ? 1 : 0;
		else
			val->intval = (bat_data.supply_status & STATUS_USB_IN) ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int get_status(void * private_data)
{
	int status = 0;
	if(GPIO_VALID(bat_data.ac_gpio) && gpio_get_value(bat_data.ac_gpio) == bat_data.ac_level) {
		status |= STATUS_CHARGING;
		if(GPIO_VALID(bat_data.charge_gpio) && gpio_get_value(bat_data.charge_gpio) == bat_data.charge_level) {
			status |= STATUS_CHARGE_DONE;
		}
	}
	return status;
}

static void adc_bat_external_power_changed(struct power_supply *supply_bat)
{
	cancel_delayed_work(&bat_data.bat_work);
	schedule_delayed_work(&bat_data.bat_work,msecs_to_jiffies(200));
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	if(bat_data.running)
		schedule_delayed_work(&bat_data.supply_work,msecs_to_jiffies(100)); //filter
	return IRQ_HANDLED;
}

static enum power_supply_property adc_bat_main_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_PRESENT,
};

static enum power_supply_property power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *power_supplied_to[] = {
	"battery",
};

struct power_supply supply_bat = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= adc_bat_main_props,
	.num_properties		= ARRAY_SIZE(adc_bat_main_props),
	.get_property		= adc_bat_get_property,
	.external_power_changed = adc_bat_external_power_changed,
	.use_for_apm		= 1,
};

static struct power_supply supply_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = power_supplied_to,
	.num_supplicants = ARRAY_SIZE(power_supplied_to),
	.properties = power_props,
	.num_properties = ARRAY_SIZE(power_props),
	.get_property = power_get_property,
};

static struct power_supply supply_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = power_supplied_to,
	.num_supplicants = ARRAY_SIZE(power_supplied_to),
	.properties = power_props,
	.num_properties = ARRAY_SIZE(power_props),
	.get_property = power_get_property,
};

static void supply_work(struct work_struct *work)
{
	int status;
	mutex_lock(&bat_data.work_lock);
	status = bat_data.get_status(bat_data.private_data);
	if(status & STATUS_CHARGING) {
		status |= (GPIO_VALID(bat_data.usb_gpio) && gpio_get_value(bat_data.usb_gpio) == bat_data.usb_level) ? 
		          STATUS_USB_IN : STATUS_AC_IN;
	}
	if(status != bat_data.supply_status) {
		printk("adc_battery status changed from %x to %x\n", bat_data.supply_status, status);
		if((status & STATUS_AC_IN) != (bat_data.supply_status & STATUS_AC_IN)) {
			power_supply_changed(&supply_ac);
		}
		if((status & STATUS_USB_IN) != (bat_data.supply_status & STATUS_USB_IN)) {
			power_supply_changed(&supply_usb);
		}
		if((status & STATUS_CHARGE_DONE) != (bat_data.supply_status & STATUS_CHARGE_DONE)) {
			adc_bat_external_power_changed(&supply_bat);
		}
		/*
		else if(!(status & STATUS_CHARGING)) {
			//filter out voltage drop caused by plug out of DC
			cancel_delayed_work(&bat_data.bat_work);
			schedule_delayed_work(&bat_data.bat_work,msecs_to_jiffies(2000));
		}
		*/
		power_changed(bat_data.supply_status ^ status, status);
		bat_data.supply_status = status;
	}
	mutex_unlock(&bat_data.work_lock);
}

static int get_capacity(int status,int voltage)
{
	int * table;
	int capaicity;
	if(status & STATUS_CHARGE_DONE) return 100;
	
	if(status & STATUS_CHARGING) {
		table = bat_data.dc_table;
	}
	else {
		table = bat_data.v2c_table;
	}
	if(voltage <= *table) return 0;
	
	while(voltage > table[3]) table += 3;
	capaicity = (voltage* table[1] + table[2]) >> 16;
	if(capaicity > 100) capaicity = 100;
	return capaicity;
}

static struct adc_item * adc_add_item(struct adc_item * head, struct adc_item * item)
{
	int first = 1;
	int capacity = item->capacity;
	int voltage = item->voltage;
	struct adc_item * cur = head;
	while(cur->capacity > capacity || (cur->capacity == capacity && cur->voltage > voltage)) {
		first = 0;
		cur = cur->next;
		if(cur == head) {
			break;
		}
	}
	item->next = cur;
	item->pre = cur->pre;
	cur->pre->next = item;
	cur->pre = item;
	return first ? item : head;
}

static int adc_get_average(struct adc_item * head, int count,int charging)
{
	int skip, sum = 0;
	if(charging) {
		skip = 2;
		count -= 4;
	}
	else {
		skip = 1;
		count -= ((count >> 1) + 1);
	}
	while(skip--) head = head->next;
	skip = count;
	while(skip--) {
		sum += head->capacity;
		head = head->next;
	}
	return (sum + (count >> 1)) / count;
}

static void adc_bat_work(struct work_struct *work)
{
	struct adc_item * item, * head;
	int status, capacity, voltage, delay, update = 0, count = bat_data.poll_count;

	if(bat_data.poll_supply || bat_data.poll_init == 1) {
		supply_work(0);
	}
	mutex_lock(&bat_data.work_lock);
	status = bat_data.supply_status & STATUS_CHARGE_MASK;
	if (status != bat_data.bat_status) {
		bat_data.bat_status = status;
		update = 1;
		if(bat_data.poll_init) {
			//reset sample buffer
			bat_data.poll_init = 1;
		}
		goto update;
	}
	voltage = bat_data.get_voltage(bat_data.private_data, status ? 0 : 1);
	bat_data.voltage = voltage;
	capacity = get_capacity(status, voltage);
	if(!bat_data.poll_init && ((!status) == (capacity > bat_data.capacity))) {
		capacity = bat_data.capacity;
	}
	item = bat_data.poll_items + bat_data.poll_index;
	item->capacity = capacity;
	item->voltage = voltage;
	item->next->pre = item->pre;
	item->pre->next = item->next;
	if(bat_data.poll_head == item) {
		bat_data.poll_head = item->next;
	}
	head = adc_add_item(bat_data.poll_head, item);
	if(adc_debug) {
		struct adc_item * cur = head;
		printk("battery capacity %d:", capacity);
		while(1) {
			printk("%d ", cur->capacity);
			cur = cur->next;
			if(cur == head) {
				break;
			}
		}
		printk("\n");
	}
	bat_data.poll_head = head;

	bat_data.poll_index++;
	if(bat_data.poll_index == count) {
		bat_data.poll_index = 0;
	}
	if(bat_data.poll_init) {
		if(bat_data.poll_init == count) {
			if(head->capacity + 4 > head->pre->capacity) {
				bat_data.poll_init = 0;
				update = 2;
			}
			else {
				printk("battery init min %d, max %d\n", head->capacity, head->pre->capacity);
				if(bat_data.capacity == 100) {
					bat_data.capacity = adc_get_average(head, count, status);
					if(bat_data.capacity == 0) {
						bat_data.capacity = 1; //not valid vol,should not poweroff now
					}
				}
			}
		}
		else {
			bat_data.poll_init++;
		}
	}
	if(bat_data.poll_init == 0 && capacity != bat_data.capacity) {
		capacity = adc_get_average(head, count, status);
		if(capacity == 100 && status == STATUS_CHARGING) {
			capacity = 99;
		}
		if(capacity != bat_data.capacity) {
			if(update != 2) {
				if((status ? 1 : 0) == (capacity > bat_data.capacity)) {
					bat_data.capacity += status ? 1 : -1;
					update = 2;
				}
			}
			else {
				if(status & STATUS_CHARGING) {
					if(capacity >= 95 && capacity < 100) {
						//work around for constant-voltage charge mode
						capacity = 100;
					}
				}
				bat_data.capacity = capacity;
			}
			printk("battery report %d %d real %d\n", voltage, bat_data.capacity, capacity);
			if(bat_data.update_capacity) {
				bat_data.update_capacity(bat_data.private_data, bat_data.capacity + adc_offset);
			}
		}
	}
update:
	if(bat_data.poll_init == 0) {
		if(adc_debug >= 1000) {
			delay = adc_debug;
		}
		else {
			delay = bat_data.poll_time;
		}
		if (update) {
			power_supply_changed(&supply_bat);
		}
	}
	else {
		delay = 150;
	}

	if(delay) {
		schedule_delayed_work(&bat_data.bat_work,msecs_to_jiffies(delay));
	}
	mutex_unlock(&bat_data.work_lock);
}

#ifdef CONFIG_PM
static int adc_bat_suspend(struct platform_device *dev, pm_message_t state)
{
	bat_data.running = 0;
	cancel_delayed_work(&bat_data.bat_work);
	cancel_delayed_work(&bat_data.supply_work);
	flush_scheduled_work();
	return 0;
}

static int adc_bat_resume(struct platform_device *dev)
{
	bat_data.poll_init = 1;
	schedule_delayed_work(&bat_data.supply_work,msecs_to_jiffies(10));
	schedule_delayed_work(&bat_data.bat_work,msecs_to_jiffies(10));
	bat_data.running = 1;
	return 0;
}
#else
#define adc_bat_suspend NULL
#define adc_bat_resume NULL
#endif

//return -1 need poll
static int request_gpio_irq(int gpio,int type,int * pirq)
{
	int ret;
	if(!GPIO_VALID(gpio)) {
		*pirq = -1;
		return 0;
	}
	*pirq = gpio_to_irq(gpio);
	if(*pirq < 0) {
		printk("adc_battery get irq for 0x%x fail\n",gpio);
		return *pirq;
	}
	
	ret = request_irq(*pirq, gpio_irq_handler,type, supply_bat.name, 0);
	if (ret < 0) {
		printk("adc_battery request irq for 0x%x fail\n",gpio);
	}
	return ret;
}

static int voltage_max_now;
static void adjust_voltage(int voltage);
static ssize_t adc_show_debug(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "battery debug %d\nmax voltage %d\n", adc_debug, voltage_max_now);
}

static ssize_t adc_store_debug(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	if(!memcmp(buf, "voltage ", 8)) {
		int max = 0;
		if(sscanf(buf+8, "%d", &max) == 1) {
			adjust_voltage(max);
			printk("battery max voltage set to %d\n", voltage_max_now);
		}
	}
	else {
		int debug = 0;
		if(!memcmp(buf, "debug ", 6)) buf += 6;
		if(sscanf(buf, "%d", &debug) == 1) {
			adc_debug = debug;
			printk("battery debug set to %d\n", adc_debug);
		}
	}
	return count;
}

static ssize_t adc_show_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",adc_offset);
}

static ssize_t adc_store_offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &adc_offset);
	printk("adc offset set to %d\n", adc_offset);
	return count;
}

static int get_adc(struct adc_client * client)
{
	int i;
	int t;
	int sum = 0, min = 0xffff, max = 0;
	for(i = 0; i < 6; i++) {
		t = adc_sync_read(client);
		if(t > max) max = t;
		if(t < min) min = t;
		sum += t;
	}
	return sum-min-max;
}

static ssize_t adc_show_voltage_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",get_adc(adc_client));
}

static ssize_t adc_store_voltage_raw(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

#define ADC_CHG_ATTR(_name)					\
{									\
	.attr = { .name = #_name,.mode = 0644 },					\
	.show =  adc_show_##_name,				\
	.store = adc_store_##_name, \
}

static struct device_attribute adc_charger_attrs[] = {
	ADC_CHG_ATTR(debug),
	ADC_CHG_ATTR(offset),
	ADC_CHG_ATTR(voltage_raw),
};

static void adc_init_attrs(struct power_supply *psy)
{
	int j, ret;
	for (j = 0; j < ARRAY_SIZE(adc_charger_attrs); j++) {
		ret = device_create_file(psy->dev, &adc_charger_attrs[j]);
	}
}

static int adc_bat_probe(struct platform_device *dev)
{
	int ret;
	bat_data.poll_init = 1;
	bat_data.capacity = 100;

	if(bat_data.get_status == 0) {
		if(GPIO_VALID(bat_data.ac_gpio)) {
			bat_data.get_status = get_status;
		}
		else {
			return -EINVAL;
		}
	}

	printk("adc_battery ac 0x%x usb 0x%x charge 0x%x\n", bat_data.ac_gpio, bat_data.usb_gpio, bat_data.charge_gpio);
	if(bat_data.init) {
		ret = bat_data.init(bat_data.private_data,0);
		if(ret) return ret;
	}

	mutex_init(&bat_data.work_lock);
	INIT_DELAYED_WORK(&bat_data.bat_work, adc_bat_work);
	INIT_DELAYED_WORK(&bat_data.supply_work, supply_work);

	ret = power_supply_register(&dev->dev, &supply_bat);
	if(ret)
		goto bat_supply_failed;
	ret = power_supply_register(&dev->dev, &supply_ac);
	if(ret)
		goto ac_supply_failed;
	ret = power_supply_register(&dev->dev, &supply_usb);
	if(ret)
		goto usb_supply_failed;

	bat_data.irq1 = bat_data.irq2 = -1;
	if(bat_data.irq_type) {
		if(request_gpio_irq(bat_data.ac_gpio,bat_data.irq_type,&bat_data.irq1) < 0) {
			bat_data.poll_supply = 1;
		}
		if(request_gpio_irq(bat_data.charge_gpio,bat_data.irq_type,&bat_data.irq2) < 0) {
			bat_data.poll_supply = 1;
		}
	}
	else {
		bat_data.poll_supply = 1;
	}
	if(bat_data.poll_supply) {
		printk("dc detect run in poll mode\n");
	}
	adc_init_attrs(&supply_bat);
	schedule_delayed_work(&bat_data.supply_work,msecs_to_jiffies(10));
	bat_data.running = 1;
	return 0;

usb_supply_failed:
	power_supply_unregister(&supply_ac);
ac_supply_failed:
	power_supply_unregister(&supply_bat);
bat_supply_failed:
	bat_data.init(&bat_data.private_data,1);
	return ret;
}

static int __devexit adc_bat_remove(struct platform_device *dev)
{
	cancel_delayed_work(&bat_data.bat_work);
	cancel_delayed_work(&bat_data.supply_work);
	flush_scheduled_work();
	power_supply_unregister(&supply_bat);
	power_supply_unregister(&supply_ac);
	power_supply_unregister(&supply_usb);
	if(bat_data.irq1 >= 0) free_irq(bat_data.irq1,0);
	if(bat_data.irq2 >= 0) free_irq(bat_data.irq2,0);
	bat_data.init(&bat_data.private_data,1);
	return 0;
}

static struct platform_driver adc_bat_driver = {
	.probe		= adc_bat_probe,
	.remove		= __devexit_p(adc_bat_remove),
	.suspend	= adc_bat_suspend,
	.resume		= adc_bat_resume,
	.driver		= {
		.name	= "adc-battery",
		.owner	= THIS_MODULE,
	},
};

//============ platform specific ==============//
#include <linux/slab.h>
#include <mach/yfmach.h>
static struct platform_device adc_battery_device;

static int voltage_cof1;
static int voltage_cof2;
static int pmu_capacity = -1;

static void update_pmu_capacity(void * private_data, int capacity)
{
	if(pmu_capacity >= 0) {
		if(capacity <= 0) {
			capacity = 0x80;
		}
		else if(capacity <= 5) {
			capacity |= (pmu_capacity & 0x80);
		}
		if(capacity != pmu_capacity) {
			pmu_data_write(PMU_BAT_CAP, capacity);
			pmu_capacity = capacity;
		}
	}
	power_changed(STATUS_CAPACITY, capacity);
}
extern int back_brightness;
extern int charge_mode;
static int old_brightness;
static int brightness_cof;
static int charge_cof;
static int charge_max1;
static int charge_max2;
static int brightness_cof1;
static int brightness_cof2;

static int last_voltage;
static int last_status;
static int status_cof1;
static int status_cof2;
static int status_offset;

static int voltage_num;
static int voltage_max;
static int voltage_begin;
static int voltage_status;
static int voltage_max_now;
static int voltage_max_cfg;
static int voltage_stable_time;
static unsigned long voltage_incre_time;

#define PMU_BAT_ADJ 1

static void adjust_coff(int offset)
{
	int voltage_max_new;
	if(offset) {
		voltage_max_new = 4100 - 1 + offset;
	}
	else {
		voltage_max_new = voltage_max_cfg;
	}
	if(voltage_max_now != voltage_max_new) {
		voltage_max_now = voltage_max_new;
		voltage_cof1 = env_get_u32("power_vol_coff1",320000);
		voltage_cof1 *= voltage_max_cfg;
		voltage_cof1 /= voltage_max_now;
		printk("battery coff update to %d\n", voltage_cof1);
	}
}

static void adjust_voltage(int voltage)
{
	int offset = 0;
	if(voltage > 0) {
		offset = voltage - 4100;
		if(offset > 254) offset = 254;
		else if(offset < 0) offset = 0;
		if(voltage_max_now == 4100 + offset) {
			return;
		}
		offset++;
	}
	pmu_data_write(PMU_BAT_ADJ, offset);
	adjust_coff(offset);
}

static int get_voltage(void * private_data, int discharge)
{
	int voltage;
	int cofs = voltage_cof2;
	if(bat_data.poll_init == 1) {
		last_status = discharge;
		status_offset = 0;
		voltage_num = 0;
		voltage_status = 0;
	}
	if(discharge && brightness_cof1) {
		int curr_birightness;
		while(old_brightness != (curr_birightness = back_brightness)) {
			msleep(3);
			if(curr_birightness) {
			    brightness_cof = curr_birightness * brightness_cof1 + brightness_cof2;
			}
			else {
				brightness_cof = 0;
			}
			old_brightness = curr_birightness;
		}
	}
	voltage = get_adc(adc_client);
	voltage = ((voltage * voltage_cof1) >> 18) + cofs;
	//voltage may return normal curve slowly
	if(discharge != last_status) {
		if(status_cof1) {
			if(last_status && voltage >= charge_max2) {
				status_offset = 0;
			}
			else {
				status_offset = ((voltage - last_voltage) * status_cof1) / 65536;
				if(last_status && voltage > charge_max1) {
					status_offset = status_offset * (charge_max2 - voltage) / (charge_max2 - charge_max1);
				}
			}
		}
		voltage_num = 0;
		voltage_status = 0;
		last_status = discharge;
	}
	last_voltage = voltage;
	if(!discharge && voltage > 4100 && voltage_status != 3) {
		int count = bat_data.poll_count;
		if(voltage_num >= count) {
			int num, sum, full = bat_data.bat_status & STATUS_CHARGE_DONE;
			struct adc_item * head = bat_data.poll_head->next->next;
			num = count = count - 4;
			sum = count >> 1;
			while(num-- > 0) {
				sum += head->voltage;
				//printk("%d ", head->voltage);
				head = head->next;
			}
			sum /= count;
			//printk(" : %d\n", sum);
			if(sum > voltage_max+1) { //+1 for debounce
				if(voltage_max == 0) {
					voltage_begin = sum;
					voltage_status = 1;
				}
				voltage_max = sum;
				voltage_incre_time = jiffies + voltage_stable_time;
				if(adc_debug) printk("battery max voltage %d\n", voltage_max);
				if(voltage_status == 1 && voltage_max > voltage_begin + 30) {
					voltage_status = 2;
					printk("battery max voltage ready\n");
				}
			}
			else if(!full) {
				if(voltage_stable_time && bat_data.capacity < 100) {
					unsigned long time = jiffies;
					if(time_after(time, voltage_incre_time)) {
						voltage_incre_time = time + voltage_stable_time;
						bat_data.capacity++;
						if(bat_data.capacity == 100) {
							full = 1;
						}
						power_supply_changed(&supply_bat);
					}
				}
			}
			if(full) {
				printk("battery full detect %d\n", sum);
				if(voltage_status == 2) {
					if(voltage_max < sum + 10) {
						voltage_max = sum + 10;
						printk("battery voltage fall not detect\n");
					}
					if(voltage_max < voltage_max_cfg - 8 || voltage_max > voltage_max_cfg + 8) {
						voltage_max *= voltage_max_now;
						voltage_max /= voltage_max_cfg;
						printk("battery adjust voltage to %d\n", voltage_max);
						adjust_voltage(voltage_max);
					}
				}
				voltage_status = 3;
			}
		}
		else {
			if(!voltage_num) {
				voltage_max = 0;
			}
			voltage_num++;
		}
	}
	if(status_offset) {
		if(adc_debug) printk("status offset %d, %d\n", voltage, status_offset);
		voltage += status_offset;
		status_offset = (status_offset * status_cof2) / 65536;
	}
	if(discharge) {
		if(brightness_cof) {
			int temp = brightness_cof / voltage;
			voltage += temp;
			cofs += temp;
		}
	}
	else if(charge_mode) {
		//calculate charge mode offset
		if(voltage <= charge_max1) {
			voltage -= charge_cof;
		}
		else if(voltage < charge_max2) {
			voltage -= charge_cof * (charge_max2 - voltage) / (charge_max2 - charge_max1);
		}
	}
	if(adc_debug) printk("battery vol %d, bright %d, offset %d\n", voltage, old_brightness, cofs);
	return voltage;
}

static int env_get_gpio(unsigned gpio, char *label)
{
	gpio = env_get_u32(label, gpio);
	if(gpio != INVALID_GPIO) {
		int ret = gpio_request(gpio, label);
		if (ret != 0) {
			printk("env_get_gpio request %s fail %d\n", label, ret);
			return INVALID_GPIO;
		}
		gpio_direction_input(gpio);
	}
	return gpio;
}

static int * __init v2c_init(int * v2cs, int len)
{
	int i, k;
	int v1, v2, c1, c2;
	int * v2c = kzalloc(len * 3 * sizeof(int), GFP_KERNEL);
	int * v2cd = v2c;
	v1 = *v2cs++;
	c1 = *v2cs++;
	for(i = 1; i < len; i++) {
		v2 = *v2cs++;
		c2 = *v2cs++;
		*v2cd++ = v1;
		k = (c2 - c1) * 0x10000 / (v2 - v1);
		*v2cd++ = k;
		*v2cd++ = c1 * 0x10000 - v1 * k;
		v1 = v2;
		c1 = c2;
	}
	*v2cd++ = v1;
	*v2cd++ = 0;
	*v2cd++ = 100 * 0x10000;
	*v2cd++ = 0xFFFFFFF;
	return v2c;
}

static int __initdata v2c_table[] = {
	3600,  0, 3648, 10, 3680, 20, 3744, 40, 3855, 60, 
	3916, 80, 4060, 90, 4200, 100
};

static int __initdata dc_table[] = {
	3690,  0, 3744, 10, 3776, 20, 3828, 40, 3936, 60, 
	4060, 80, 4140, 90, 4230, 100
};

static void __init poll_init(struct adc_item * head, int count)
{
	struct adc_item * item = head;
	while(1) {
		item->next = item+1;
		item->pre = item-1;
		if(count == 1) break;
		count--;
		item++;
	}
	item->next = head;
	head->pre = item;
}
static int __init platform_init(void)
{
	int len;
	int adc;
	int v2c[40];

	adc = env_get_u32("power_adc_channel", 0);
	adc_client = adc_register(adc, 0, 0);
	if(adc_client) {
		adc_battery_device.name = "adc-battery";
		platform_device_register(&adc_battery_device);
	}
	else {
		printk("failed to register adc %d\n", adc);
		return -1;
	}
	status_cof1 = env_get_u32("power_status_coff1", 0);
	status_cof2 = env_get_u32("power_status_coff2", 0);
	voltage_stable_time = env_get_u32("power_stable_time", 1800) * HZ;

	voltage_cof1  = env_get_u32("power_vol_coff1",320000);//1024:5V
	voltage_cof2  = env_get_u32("power_vol_coff2",0);
	//charge mode offset
	charge_cof  = env_get_u32("power_charge_coff",0);
	charge_max1  = env_get_u32("power_charge_max1",4190);
	charge_max2  = env_get_u32("power_charge_max2",4215);
	brightness_cof1  = env_get_u32("power_bright_coff1",0);//1024:5V
	brightness_cof2  = env_get_u32("power_bright_coff2",0);

	bat_data.get_voltage = get_voltage;
	bat_data.ac_gpio = env_get_gpio(INVALID_GPIO,"power_ac_gpio");
	bat_data.usb_gpio = env_get_gpio(INVALID_GPIO,"power_usb_gpio");
	bat_data.charge_gpio = env_get_gpio(INVALID_GPIO,"power_charge_gpio");
	bat_data.ac_level = env_get_u32("power_ac_level", 1);
	bat_data.usb_level = env_get_u32("power_usb_level", 1);
	bat_data.charge_level = env_get_u32("power_charge_level", 1);
	bat_data.poll_time  = env_get_u32("power_poll_time",10000);
	bat_data.poll_count  = env_get_u32("power_poll_count",12);
	if(bat_data.poll_count < 6) bat_data.poll_count = 6;
	bat_data.poll_head = kzalloc(bat_data.poll_count * sizeof(struct adc_item), GFP_KERNEL);
	bat_data.poll_items = bat_data.poll_head;
	poll_init(bat_data.poll_head, bat_data.poll_count);

	len = env_cpy_u32s("power_v2c_table",v2c,40);
	if(len < 4) {
		bat_data.v2c_table = v2c_init(v2c_table, sizeof(v2c_table) >> 3);
	}
	else {
		bat_data.v2c_table = v2c_init(v2c, len >> 1);
	}

	len = env_cpy_u32s("power_dc_table",v2c,40);
	if(len < 4) {
		bat_data.dc_table = v2c_init(dc_table, sizeof(dc_table) >> 3);
		voltage_max_cfg = 4220;
	}
	else {
		bat_data.dc_table = v2c_init(v2c, len >> 1);
		voltage_max_cfg = v2c[len - 2];
	}
	printk("battery max voltage %d\n", voltage_max_cfg);
	//update voltage max
	adjust_coff(pmu_data_read(PMU_BAT_ADJ));

	bat_data.irq_type = IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING;
	pmu_capacity = pmu_data_read(PMU_BAT_CAP);
	if(pmu_capacity >= 0) {
		bat_data.update_capacity = update_pmu_capacity;
		printk("adc battery last capacity is %d\n", pmu_capacity);
	}
	return 0;
}

static int __init adc_bat_init(void)
{
	if(platform_init()) return -EINVAL;
	return platform_driver_register(&adc_bat_driver);
}

static void __exit adc_bat_exit(void)
{
	kfree(bat_data.v2c_table);
	kfree(bat_data.dc_table);
	kfree(bat_data.poll_items);
	platform_driver_unregister(&adc_bat_driver);
}

fs_initcall(adc_bat_init);
module_exit(adc_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("liqiangman@yftech.com");
MODULE_DESCRIPTION("ADC based battery driver");

