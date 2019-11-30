
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <mach/yfmach.h>

static int s_wifi_config;
static const char * s_wifi_name;

static const char * wifi_get_name(void)
{
	if(!s_wifi_config) {
		s_wifi_name = env_get_str("wifi_supproted", 0);
		s_wifi_config = 1;
	}
	return s_wifi_name;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t wifi_chip_read(struct class *cls, struct class_attribute *attr, char *_buf)
#else
static ssize_t wifi_chip_read(struct class *cls, char *_buf)
#endif
{
    int count = 0;
    const char * name;
    if((name = wifi_get_name())) {
        count = sprintf(_buf, "%s", name);
        printk("Current WiFi chip is %s.\n", name);
        return count;
    }

#ifdef CONFIG_BCM4329
    count = sprintf(_buf, "%s", "BCM4329");
    printk("Current WiFi chip is BCM4329.\n");
#endif

#ifdef CONFIG_RTL8192CU
    count = sprintf(_buf, "%s", "RTL8188CU");
    printk("Current WiFi chip is RTL8188CU.\n");
#endif

#ifdef CONFIG_RTL8188EU
    count = sprintf(_buf, "%s", "RTL8188EU");
    printk("Current WiFi chip is RTL8188EU.\n");
#endif

#ifdef CONFIG_BCM4330
    count = sprintf(_buf, "%s", "BCM4330");
    printk("Current WiFi chip is BCM4330.\n");
#endif

#ifdef CONFIG_RK901
    count = sprintf(_buf, "%s", "RK901");
    printk("Current WiFi chip is RK901.\n");
#endif

#ifdef CONFIG_RK903
    count = sprintf(_buf, "%s", "RK903");
    printk("Current WiFi chip is RK903.\n");
#endif

#ifdef CONFIG_MT5931
    count = sprintf(_buf, "%s", "MT5931");
    printk("Current WiFi chip is MT5931.\n");
#endif

    return count;
}

/*
static ssize_t wifi_channel_write(struct class *cls, const char *_buf, size_t _count)
{
    int ret, channel;
    
    if (wifi_enabled == 0)
    {
        printk("WiFi is disabled.\n");
        return _count;
    }
    
    channel = simple_strtol(_buf, NULL, 10);
    
    ret = wifi_emi_set_channel(channel);
    if (ret != 0)
    {
        //printk("Set channel=%d fail.\n", channel);
    }
    else
    {
        //printk("Set channel=%d successfully.\n", channel);
        wifi_channel = channel;
    }
    
    return _count;
}
*/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t wifi_p2p_read(struct class *cls, struct class_attribute *attr, char *_buf)
#else
static ssize_t wifi_p2p_read(struct class *cls, char *_buf)
#endif
{
	int count = 0;
	int p2p = 0;
    const char * name;

    if((name = wifi_get_name())) {
        p2p = !strcmp(name, "RTL8188CU") || !strcmp(name, "RTL8188EU") || !strcmp(name, "RK903") || !strcmp(name, "BCM4330");
    }
    else {

#ifdef CONFIG_RTL8192CU
        p2p = 1;
#endif

#ifdef CONFIG_RTL8188EU
        p2p = 1;
#endif

#ifdef CONFIG_RK903
        p2p = 1;
#endif

#ifdef CONFIG_BCM4330
        p2p = 1;
#endif

    }

    count = sprintf(_buf, "%s", p2p ? "true" : "false");
	return count;
}

static struct class *rkwifi_class = NULL;
static CLASS_ATTR(chip, 0664, wifi_chip_read, NULL);
static CLASS_ATTR(p2p, 0664, wifi_p2p_read, NULL);

int rkwifi_sysif_init(void)
{
    int ret;
    
    printk("Rockchip WiFi SYS interface (V1.00) ... \n");
    
    rkwifi_class = NULL;
    
    rkwifi_class = class_create(THIS_MODULE, "rkwifi");
    if (IS_ERR(rkwifi_class)) 
    {   
        printk("Create class rkwifi_class failed.\n");
        return -ENOMEM;
    }
    
    ret =  class_create_file(rkwifi_class, &class_attr_chip);
    ret =  class_create_file(rkwifi_class, &class_attr_p2p);
    
    return 0;
}

void rkwifi_sysif_exit(void)
{
    // need to remove the sys files and class
    class_remove_file(rkwifi_class, &class_attr_chip);
    class_destroy(rkwifi_class);
    
    rkwifi_class = NULL;
}

module_init(rkwifi_sysif_init);
module_exit(rkwifi_sysif_exit);

MODULE_AUTHOR("Yongle Lai");
MODULE_DESCRIPTION("WiFi SYS @ Rockchip");
MODULE_LICENSE("GPL");

