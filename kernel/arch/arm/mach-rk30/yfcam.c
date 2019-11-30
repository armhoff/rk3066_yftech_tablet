
#include <mach/iomux.h>
#include <media/soc_camera.h>
#include <linux/android_pmem.h>
#include <mach/rk30_camera.h>
#include <mach/yfmach.h>

/*****************************************************************************************
 * camera  devices
 * author: ddl@rock-chips.com
 *****************************************************************************************/
#ifdef CONFIG_VIDEO_RK29 

#define dprintk(fmt, arg...) do {if(0) printk(KERN_WARNING "rk_cam_io: " fmt , ## arg); } while (0)

static int s_cam_reserved = 0xc00000; //default for 200M
//buffer from 6M to 8M for 720p preview, needed for cts
static int s_ipp_reserved = 0x800000;
#define PMEM_CAM_NECESSARY s_cam_reserved
#define PMEM_CAMIPP_NECESSARY s_ipp_reserved
#ifdef CONFIG_VIDEO_RK29_CAMMEM_ION
#undef PMEM_CAM_NECESSARY
#endif

static int rk_sensor_iomux(int pin)
{
    yf_mux_api_set(pin, 0);
    return 0;
}

static struct rk29camera_platform_data rk_camera_platform_data ;

static rk_sensor_user_init_data_s rk_init_data_sensor[RK_CAM_NUM] = {
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    }
};

static u64 rockchip_device_camera_dmamask = 0xffffffffUL;
static struct resource rk_camera_resource_host_0[] = {
	[0] = {
		.start = RK30_CIF0_PHYS,
		.end   = RK30_CIF0_PHYS + RK30_CIF0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CIF0,
		.end   = IRQ_CIF0,
		.flags = IORESOURCE_IRQ,
	}
};
static struct resource rk_camera_resource_host_1[] = {
	[0] = {
		.start = RK30_CIF1_PHYS,
		.end   = RK30_CIF1_PHYS + RK30_CIF1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CIF1,
		.end   = IRQ_CIF1,
		.flags = IORESOURCE_IRQ,
	}
};
/*platform_device : */
 struct platform_device rk_device_camera_host_0 = {
	.name		  = RK29_CAM_DRV_NAME,
	.id 	  = RK_CAM_PLATFORM_DEV_ID_0,				/* This is used to put cameras on this interface */
	.num_resources	  = ARRAY_SIZE(rk_camera_resource_host_0),
	.resource	  = rk_camera_resource_host_0,
	.dev			= {
		.dma_mask = &rockchip_device_camera_dmamask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data	= &rk_camera_platform_data,
	}
};
/*platform_device : */
 struct platform_device rk_device_camera_host_1 = {
	.name		  = RK29_CAM_DRV_NAME,
	.id 	  = RK_CAM_PLATFORM_DEV_ID_1,				/* This is used to put cameras on this interface */
	.num_resources	  = ARRAY_SIZE(rk_camera_resource_host_1),
	.resource	  = rk_camera_resource_host_1,
	.dev			= {
		.dma_mask = &rockchip_device_camera_dmamask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data	= &rk_camera_platform_data,
	}
};

static char s_camera_name[RK_CAM_NUM][32] = {"back_", "front_","back2_", "front2_","back3_", "front3_"};

#if 1
static int sensor_power_default_cb (struct rk29camera_gpio_res *res, int on)
{
    struct regulator *ldo_18,*ldo_28;
	ldo_28 = regulator_get(NULL, "vmmc");	// vcc28_cif
	ldo_18 = regulator_get(NULL, "vdig1");	// vcc18_cif
	if (ldo_28 == NULL || IS_ERR(ldo_28) || ldo_18 == NULL || IS_ERR(ldo_18)){
        printk("get cif ldo failed!\n");
		return -1;
	}
    if(on == 0){
    	regulator_disable(ldo_28);
    	regulator_put(ldo_28);
    	regulator_disable(ldo_18);
    	regulator_put(ldo_18);
    }
    else{
    	regulator_set_voltage(ldo_28, 2800000, 2800000);
    	regulator_enable(ldo_28);
    	regulator_put(ldo_28);

    	regulator_set_voltage(ldo_18, 1800000, 1800000);
    	regulator_enable(ldo_18);
    	regulator_put(ldo_18);
    	msleep(50);
    }
    return 0;
}
#else
static int sensor_power_default_cb (struct rk29camera_gpio_res *res, int on)
{
    int camera_power = res->gpio_power;
    int camera_ioflag = res->gpio_flag;
    int camera_io_init = res->gpio_init;
    int ret = 0;
    
    if (camera_power != INVALID_GPIO)  {
		if (camera_io_init & RK29_CAM_POWERACTIVE_MASK) {
            if (on) {
            	gpio_set_value(camera_power, ((camera_ioflag&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
    			dprintk("%s..%s..PowerPin=%d ..PinLevel = %x   \n",__FUNCTION__,res->dev_name, camera_power, ((camera_ioflag&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
    			msleep(10);
    		} else {
    			gpio_set_value(camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
    			dprintk("%s..%s..PowerPin=%d ..PinLevel = %x   \n",__FUNCTION__,res->dev_name, camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
    		}
		} else {
			ret = RK29_CAM_EIO_REQUESTFAIL;
			printk("%s..%s..PowerPin=%d request failed!\n",__FUNCTION__,res->dev_name,camera_power);
	    }        
    } else {
		ret = RK29_CAM_EIO_INVALID;
    } 

    return ret;
}
#endif

static int sensor_reset_default_cb (struct rk29camera_gpio_res *res, int on)
{
    int camera_reset = res->gpio_reset;
    int camera_ioflag = res->gpio_flag;
    int camera_io_init = res->gpio_init;  
    int ret = 0;
    
    if (camera_reset != INVALID_GPIO) {
		if (camera_io_init & RK29_CAM_RESETACTIVE_MASK) {
			if (on) {
	        	gpio_set_value(camera_reset, ((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
	        	dprintk("%s..%s..ResetPin=%d ..PinLevel = %x \n",__FUNCTION__,res->dev_name,camera_reset, ((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
			} else {
				gpio_set_value(camera_reset,(((~camera_ioflag)&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
        		dprintk("%s..%s..ResetPin= %d..PinLevel = %x   \n",__FUNCTION__,res->dev_name, camera_reset, (((~camera_ioflag)&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
	        }
		} else {
			ret = RK29_CAM_EIO_REQUESTFAIL;
			printk("%s..%s..ResetPin=%d request failed!\n",__FUNCTION__,res->dev_name,camera_reset);
		}
    } else {
		ret = RK29_CAM_EIO_INVALID;
    }

    return ret;
}

static int sensor_powerdown_default_cb (struct rk29camera_gpio_res *res, int on)
{
    int camera_powerdown = res->gpio_powerdown;
    int camera_ioflag = res->gpio_flag;
    int camera_io_init = res->gpio_init;  
    int ret = 0;    

    if (camera_powerdown != INVALID_GPIO) {
		if (camera_io_init & RK29_CAM_POWERDNACTIVE_MASK) {
			if (on) {
	        	gpio_set_value(camera_powerdown, ((camera_ioflag&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
	        	dprintk("%s..%s..PowerDownPin=%d ..PinLevel = %x \n",__FUNCTION__,res->dev_name,camera_powerdown, ((camera_ioflag&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
			} else {
				gpio_set_value(camera_powerdown,(((~camera_ioflag)&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
        		dprintk("%s..%s..PowerDownPin= %d..PinLevel = %x   \n",__FUNCTION__,res->dev_name, camera_powerdown, (((~camera_ioflag)&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
	        }
		} else {
			ret = RK29_CAM_EIO_REQUESTFAIL;
			dprintk("%s..%s..PowerDownPin=%d request failed!\n",__FUNCTION__,res->dev_name,camera_powerdown);
		}
    } else {
		ret = RK29_CAM_EIO_INVALID;
    }
    return ret;
}


static int sensor_flash_default_cb (struct rk29camera_gpio_res *res, int on)
{
    int camera_flash = res->gpio_flash;
    int camera_ioflag = res->gpio_flag;
    int camera_io_init = res->gpio_init;  
    int ret = 0;    

    if (camera_flash != INVALID_GPIO) {
		if (camera_io_init & RK29_CAM_FLASHACTIVE_MASK) {
            switch (on)
            {
                case Flash_Off:
                {
                    gpio_set_value(camera_flash,(((~camera_ioflag)&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
        		    dprintk("\n%s..%s..FlashPin= %d..PinLevel = %x   \n",__FUNCTION__,res->dev_name, camera_flash, (((~camera_ioflag)&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS)); 
        		    break;
                }

                case Flash_On:
                {
                    gpio_set_value(camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
	        	    dprintk("%s..%s..FlashPin=%d ..PinLevel = %x \n",__FUNCTION__,res->dev_name,camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
	        	    break;
                }

                case Flash_Torch:
                {
                    gpio_set_value(camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
	        	    dprintk("%s..%s..FlashPin=%d ..PinLevel = %x \n",__FUNCTION__,res->dev_name,camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
	        	    break;
                }

                default:
                {
                    printk("%s..%s..Flash command(%d) is invalidate \n",__FUNCTION__,res->dev_name,on);
                    break;
                }
            }
		} else {
			ret = RK29_CAM_EIO_REQUESTFAIL;
			printk("%s..%s..FlashPin=%d request failed!\n",__FUNCTION__,res->dev_name,camera_flash);
		}
    } else {
		ret = RK29_CAM_EIO_INVALID;
    }
    return ret;
}

static int rk_sensor_io_deinit(int sensor)
{
    unsigned int camera_reset = INVALID_GPIO, camera_power = INVALID_GPIO;
	unsigned int camera_powerdown = INVALID_GPIO, camera_flash = INVALID_GPIO;
	struct rk29camera_platform_data* plat_data = &rk_camera_platform_data;
    
    camera_reset = plat_data->gpio_res[sensor].gpio_reset;
    camera_power = plat_data->gpio_res[sensor].gpio_power;
	camera_powerdown = plat_data->gpio_res[sensor].gpio_powerdown;
    camera_flash = plat_data->gpio_res[sensor].gpio_flash;

	if (plat_data->gpio_res[sensor].gpio_init & RK29_CAM_POWERACTIVE_MASK) {
	    if (camera_power != INVALID_GPIO) {
	        gpio_direction_input(camera_power);
	        gpio_free(camera_power);
	    }
	}

	if (plat_data->gpio_res[sensor].gpio_init & RK29_CAM_RESETACTIVE_MASK) {
	    if (camera_reset != INVALID_GPIO)  {
	        gpio_direction_input(camera_reset);
	        gpio_free(camera_reset);
	    }
	}

	if (plat_data->gpio_res[sensor].gpio_init & RK29_CAM_POWERDNACTIVE_MASK) {
	    if (camera_powerdown != INVALID_GPIO)  {
	        gpio_direction_input(camera_powerdown);
	        gpio_free(camera_powerdown);
	    }
	}

	if (plat_data->gpio_res[sensor].gpio_init & RK29_CAM_FLASHACTIVE_MASK) {
	    if (camera_flash != INVALID_GPIO)  {
	        gpio_direction_input(camera_flash);
	        gpio_free(camera_flash);
	    }
	}
	plat_data->gpio_res[sensor].gpio_init = 0;
	
    return 0;
}

static void rk29_sensor_fps_get(int idx, unsigned int *val, int w, int h)
{
	if ((w==1280) && (h==720)) *val = 30000;
	else *val = 15000;
}
static int rk_sensor_io_init(void)
{
    int ret = 0, i,j;
    unsigned int camera_reset = INVALID_GPIO, camera_power = INVALID_GPIO;
	unsigned int camera_powerdown = INVALID_GPIO, camera_flash = INVALID_GPIO;
	unsigned int camera_ioflag;
	static bool is_init = false;
	struct rk29camera_platform_data* plat_data = &rk_camera_platform_data;

    if(is_init) {		
		return 0;
	} else {
		is_init = true;
	}

	for(i = 0;i < RK_CAM_NUM; i++){
        if (plat_data->gpio_res[i].dev_name == NULL)
            continue;
		camera_reset = plat_data->gpio_res[i].gpio_reset;
		camera_power = plat_data->gpio_res[i].gpio_power;
		camera_powerdown = plat_data->gpio_res[i].gpio_powerdown;
		camera_flash = plat_data->gpio_res[i].gpio_flash;
		camera_ioflag = plat_data->gpio_res[i].gpio_flag;
		plat_data->gpio_res[i].gpio_init = 0;

        if (camera_power != INVALID_GPIO) {
            ret = gpio_request(camera_power, "camera power");
            if (ret) {
                for (j=0; j<i; j++) {
                    if (camera_power == plat_data->gpio_res[j].gpio_power)
                        break;
                }
                if (i==j) {
                    printk(KERN_ERR"rk_cam_io: %s..%s..power pin(%d) init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_power);
                    goto sensor_io_init_erro;
                }
            }

            if (rk_camera_platform_data.iomux(camera_power) < 0) {
                printk(KERN_ERR "rk_cam_io: %s..%s..power pin(%d) iomux init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_power);
                goto sensor_io_init_erro;
            }
            
			plat_data->gpio_res[i].gpio_init |= RK29_CAM_POWERACTIVE_MASK;
            gpio_set_value(camera_reset, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
            gpio_direction_output(camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));

			dprintk("%s....power pin(%d) init success(0x%x)  \n",__FUNCTION__,camera_power,(((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));

        }

        if (camera_reset != INVALID_GPIO) {
            ret = gpio_request(camera_reset, "camera reset");
            if (ret) {
                for (j=0; j<i; j++) {
                    if (camera_reset == plat_data->gpio_res[j].gpio_reset) {                        
                        break;
                    }
                }
                if (i==j) {
                    printk(KERN_ERR"rk_cam_io: %s..%s..reset pin(%d) init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_reset);
                    goto sensor_io_init_erro;
                }
                
                printk("%s..%s..reset pin(%d) init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_reset);
                goto sensor_io_init_erro;
            }

            if (rk_camera_platform_data.iomux(camera_reset) < 0) {
                printk(KERN_ERR"rk_cam_io: %s..%s..reset pin(%d) iomux init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_reset);
                goto sensor_io_init_erro;
            }
            
			plat_data->gpio_res[i].gpio_init |= RK29_CAM_RESETACTIVE_MASK;
            gpio_set_value(camera_reset, ((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
            gpio_direction_output(camera_reset, ((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));

			dprintk("%s....reset pin(%d) init success(0x%x)\n",__FUNCTION__,camera_reset,((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));

        }

		if (camera_powerdown != INVALID_GPIO) {
            ret = gpio_request(camera_powerdown, "camera powerdown");
            if (ret) {
                for (j=0; j<i; j++) {
                    if (camera_powerdown == plat_data->gpio_res[j].gpio_powerdown) {                        
                        break;
                    }
                }
                if (i==j) {
                    printk(KERN_ERR"rk_cam_io: %s..%s..powerdown pin(%d) init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_powerdown);
                    goto sensor_io_init_erro;
                }
                printk("%s..%s..powerdown pin(%d) init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_powerdown);
                //goto sensor_io_init_erro;
            }

            if (rk_camera_platform_data.iomux(camera_powerdown) < 0) {
                printk(KERN_ERR "rk_cam_io: %s..%s..powerdown pin(%d) iomux init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_powerdown);
                goto sensor_io_init_erro;
            }
            
			plat_data->gpio_res[i].gpio_init |= RK29_CAM_POWERDNACTIVE_MASK;
            gpio_set_value(camera_powerdown, ((camera_ioflag&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
            gpio_direction_output(camera_powerdown, ((camera_ioflag&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));

			dprintk("%s....powerdown pin(%d) init success(0x%x) \n",__FUNCTION__,camera_powerdown,((camera_ioflag&RK29_CAM_POWERDNACTIVE_BITPOS)>>RK29_CAM_POWERDNACTIVE_BITPOS));

        }

		if (camera_flash != INVALID_GPIO) {
            ret = gpio_request(camera_flash, "camera flash");
            if (ret) {
                for (j=0; j<i; j++) {
                    if (camera_flash == plat_data->gpio_res[j].gpio_flash) {                        
                        break;
                    }
                }
                if (i==j) {
                    printk(KERN_ERR"rk_cam_io: %s..%s..flash pin(%d) init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_flash);
                    goto sensor_io_init_erro;
                }
                printk("%s..%s..flash pin(%d) init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_flash);
				goto sensor_io_init_erro;
            }

            if (rk_camera_platform_data.iomux(camera_flash) < 0) {
                printk(KERN_ERR "rk_cam_io: %s..%s..flash pin(%d) iomux init failed\n",__FUNCTION__,plat_data->gpio_res[i].dev_name,camera_flash);                
            }
            
			plat_data->gpio_res[i].gpio_init |= RK29_CAM_FLASHACTIVE_MASK;
            gpio_set_value(camera_flash, ((~camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));    /* falsh off */
            gpio_direction_output(camera_flash, ((~camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));

			dprintk("%s....flash pin(%d) init success(0x%x) \n",__FUNCTION__,camera_flash,((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));

        }  

        
        for (j=0; j<10; j++) {
            memset(&plat_data->info[i].fival[j],0x00,sizeof(struct v4l2_frmivalenum));

            if (j==0) {
                plat_data->info[i].fival[j].width = 176;
                plat_data->info[i].fival[j].height = 144;
            } else if (j==1) {
                plat_data->info[i].fival[j].width = 320;
                plat_data->info[i].fival[j].height = 240;
            } else if (j==2) {
                plat_data->info[i].fival[j].width = 352;
                plat_data->info[i].fival[j].height = 288;
            } else if (j==3) {
                plat_data->info[i].fival[j].width = 640;
                plat_data->info[i].fival[j].height = 480;
            } else if (j==4) {
                plat_data->info[i].fival[j].width = 720;
                plat_data->info[i].fival[j].height = 480;
            } else if (j==5) {
                plat_data->info[i].fival[j].width = 1280;
                plat_data->info[i].fival[j].height = 720;
            } else if (j==6) {
                plat_data->info[i].fival[j].width = 240;
                plat_data->info[i].fival[j].height = 160;
            }
            if (plat_data->info[i].fival[j].width && plat_data->info[i].fival[j].height) {
                rk29_sensor_fps_get(i,&plat_data->info[i].fival[j].discrete.denominator,
                    plat_data->info[i].fival[j].width,plat_data->info[i].fival[j].height);
                plat_data->info[i].fival[j].discrete.numerator= 1000;
                plat_data->info[i].fival[j].index = 0;
                plat_data->info[i].fival[j].pixel_format = V4L2_PIX_FMT_NV12;
                plat_data->info[i].fival[j].type = V4L2_FRMIVAL_TYPE_DISCRETE;
            }
        }
        
	continue;
sensor_io_init_erro:
		rk_sensor_io_deinit(i);
	}
	return 0;
}

static int rk_sensor_ioctrl(struct device *dev,enum rk29camera_ioctrl_cmd cmd, int on)
{
    struct rk29camera_gpio_res *res = NULL;    
	int ret = RK29_CAM_IO_SUCCESS,i = 0;

	struct rk29camera_platform_data* plat_data = &rk_camera_platform_data;
	//for test reg
	for(i = 0;i < RK_CAM_NUM;i++){
		if(plat_data->gpio_res[i].dev_name &&  (strcmp(plat_data->gpio_res[i].dev_name, dev_name(dev)) == 0)) {
			res = (struct rk29camera_gpio_res *)&plat_data->gpio_res[i];
			break;
	    } 
    } 
    
    if (res == NULL) {
        printk(KERN_ERR "rk_cam_io: %s is not regisiterd in rk29_camera_platform_data!!\n",dev_name(dev));
        ret = RK29_CAM_EIO_INVALID;
        goto rk_sensor_ioctrl_end;
    }
	
	switch (cmd)
 	{
 		case Cam_Power:
		{
			sensor_power_default_cb(res, on);
			break;
		}
		case Cam_Reset:
		{
			sensor_reset_default_cb(res, on);
			break;
		}

		case Cam_PowerDown:
		{
			sensor_powerdown_default_cb(res, on);
			break;
		}

		case Cam_Flash:
		{
			sensor_flash_default_cb(res, on);
			break;
		}
		default:
		{
			printk("%s cmd(0x%x) is unknown!\n",__FUNCTION__, cmd);
			break;
		}
 	}
rk_sensor_ioctrl_end:
    return ret;
}
static int rk_sensor_powerdown(struct device *dev, int on)
{
	return rk_sensor_ioctrl(dev,Cam_PowerDown,on);
}
static int rk_sensor_power(struct device *dev, int on)
{
	rk_sensor_ioctrl(dev,Cam_Power,on);
	if(!on) rk_sensor_powerdown(dev,1);
	msleep(50);
	return 0;
}
static int rk_sensor_reset(struct device *dev)
{
	rk_sensor_ioctrl(dev,Cam_Reset,1);
	msleep(2);
	rk_sensor_ioctrl(dev,Cam_Reset,0);
	return 0;
}
static void rk_init_camera_plateform_data(void)
{
	int i,j, dev_idx;
	const char * cam_name;
	char back_name[] = "back1_";
	char front_name[]= "front1_";
	char cfg_name[] = "cam_name0";
	char cfg_reset[] = "cam_reset0";
	char cfg_power[] = "cam_power0";
	char cfg_pd[] = "cam_pd0";
	char cfg_flash[] = "cam_flash0";
	char cfg_flag[] = "cam_flag0";
	char cfg_orient[] = "cam_orient0";
	char cfg_addr[] = "cam_addr0";
	struct rk29camera_gpio_res * gpios = rk_camera_platform_data.gpio_res;
	struct rk_camera_device_register_info * infos = rk_camera_platform_data.register_dev;
    rk_camera_platform_data.io_init = rk_sensor_io_init,
    rk_camera_platform_data.io_deinit = rk_sensor_io_deinit,
    rk_camera_platform_data.iomux = rk_sensor_iomux,
    rk_camera_platform_data.sensor_ioctrl = rk_sensor_ioctrl;
	
	for(i=0; i<2; i++) {
		for(j=0; j<RK_CAM_NUM/2; j++) {
			cfg_name[sizeof(cfg_name) - 2] = '0' + i + 2*j;
			cam_name = env_get_str(cfg_name, 0);
			if(cam_name == 0) {
				gpios->gpio_reset = INVALID_GPIO;
				gpios->gpio_power = INVALID_GPIO;
				gpios->gpio_powerdown = INVALID_GPIO;
				gpios->gpio_flash = INVALID_GPIO;
			}
			else {
				cfg_reset[sizeof(cfg_reset) - 2] = '0' + i + 2*j;
				cfg_power[sizeof(cfg_power) - 2] = '0' + i + 2*j;
				cfg_pd[sizeof(cfg_pd) - 2] = '0' + i + 2*j;
				cfg_flash[sizeof(cfg_flash) - 2] = '0' + i + 2*j;
				cfg_flag[sizeof(cfg_flag) - 2] = '0' + i + 2*j;
				cfg_orient[sizeof(cfg_orient) - 2] = '0' + i + 2*j;
				cfg_addr[sizeof(cfg_addr) - 2] = '0' + i + 2*j;
				if(i==0){
					back_name[sizeof(back_name)-3]=j+1+'0';
	   	 			strcpy(s_camera_name[i+ 2*j],back_name);
				}else{
					front_name[sizeof(front_name)-3]=j+1+'0';
	   	 			strcpy(s_camera_name[i+ 2*j],front_name);
				}
				strcat(s_camera_name[i+ 2*j], cam_name);
				gpios->gpio_reset = env_get_u32(cfg_reset, INVALID_GPIO);
				gpios->gpio_power = env_get_u32(cfg_power, INVALID_GPIO);
				gpios->gpio_powerdown = env_get_u32(cfg_pd, INVALID_GPIO);
				gpios->gpio_flash = env_get_u32(cfg_flash, INVALID_GPIO);
				gpios->gpio_flag = env_get_u32(cfg_flag, 0);
				gpios->dev_name = s_camera_name[i+2*j];
				rk_camera_platform_data.info[(RK_CAM_NUM/2)*i+j].dev_name = s_camera_name[i+2*j];
				rk_camera_platform_data.info[(RK_CAM_NUM/2)*i+j].orientation = env_get_u32(cfg_orient, 0);
				strcpy(infos->i2c_cam_info.type , cam_name);
				infos->i2c_cam_info.addr = (unsigned short) env_get_u32(cfg_addr, 0);
				infos->link_info.module_name = cam_name;
				infos->link_info.bus_id = RK_CAM_PLATFORM_DEV_ID_0;
	            infos->link_info.power = rk_sensor_power;
	            infos->link_info.reset = rk_sensor_reset;
	            infos->link_info.powerdown = rk_sensor_powerdown,
	            infos->link_info.i2c_adapter_id = 3;
				infos->device_info.name = "soc-camera-pdrv";
				infos->device_info.dev.init_name = s_camera_name[i+2*j];
			}
			gpios++;
			infos++;
		}
	}
    dev_idx = 0;
	for (i=0; i<RK_CAM_NUM; i++) {
       rk_init_data_sensor[i].rk_sensor_init_width = INVALID_VALUE;
       rk_init_data_sensor[i].rk_sensor_init_height = INVALID_VALUE;
       rk_init_data_sensor[i].rk_sensor_init_bus_param = INVALID_VALUE;
       rk_init_data_sensor[i].rk_sensor_init_pixelcode = INVALID_VALUE;
       rk_init_data_sensor[i].rk_sensor_init_data = NULL;
       rk_init_data_sensor[i].rk_sensor_init_winseq = NULL;
       rk_init_data_sensor[i].rk_sensor_winseq_size = 0;
       rk_init_data_sensor[i].rk_sensor_init_data_size = 0;
	}
    for (i=0; i<RK_CAM_NUM; i++) {
        rk_camera_platform_data.sensor_init_data[i] = &rk_init_data_sensor[i];
        if (rk_camera_platform_data.register_dev[i].device_info.name) {            
            rk_camera_platform_data.register_dev[i].link_info.board_info = 
                &rk_camera_platform_data.register_dev[i].i2c_cam_info;
            rk_camera_platform_data.register_dev[i].device_info.id = dev_idx;
            rk_camera_platform_data.register_dev[i].device_info.dev.platform_data = 
                &rk_camera_platform_data.register_dev[i].link_info;
            dev_idx++;
        }
    }
}

static void rk30_camera_request_reserve_mem(void)
{
	//should init before reserve memory
	rk_init_camera_plateform_data();
#ifdef CONFIG_VIDEO_RK29_WORK_IPP
    #ifdef CONFIG_VIDEO_RKCIF_WORK_SIMUL_OFF
        rk_camera_platform_data.meminfo.name = "camera_ipp_mem";
        rk_camera_platform_data.meminfo.start = board_mem_reserve_add("camera_ipp_mem",PMEM_CAMIPP_NECESSARY);
        rk_camera_platform_data.meminfo.size= PMEM_CAMIPP_NECESSARY;

        memcpy(&rk_camera_platform_data.meminfo_cif1,&rk_camera_platform_data.meminfo,sizeof(struct rk29camera_mem_res));
    #else
        rk_camera_platform_data.meminfo.name = "camera_ipp_mem_0";
        rk_camera_platform_data.meminfo.start = board_mem_reserve_add("camera_ipp_mem_0",PMEM_CAMIPP_NECESSARY_CIF_0);
        rk_camera_platform_data.meminfo.size= PMEM_CAMIPP_NECESSARY_CIF_0;
        
        rk_camera_platform_data.meminfo_cif1.name = "camera_ipp_mem_1";
        rk_camera_platform_data.meminfo_cif1.start =board_mem_reserve_add("camera_ipp_mem_1",PMEM_CAMIPP_NECESSARY_CIF_1);
        rk_camera_platform_data.meminfo_cif1.size= PMEM_CAMIPP_NECESSARY_CIF_1;
    #endif
 #endif
 #ifdef PMEM_CAM_NECESSARY
        android_pmem_cam_pdata.start = board_mem_reserve_add((char*)(android_pmem_cam_pdata.name),PMEM_CAM_NECESSARY);
        android_pmem_cam_pdata.size= PMEM_CAM_NECESSARY;
 #endif

}
static int rk_register_camera_devices(void)
{
    int i;
    int host_registered_0,host_registered_1;

    host_registered_0 = 0;
    host_registered_1 = 0;
    for (i=0; i<RK_CAM_NUM; i++) {
        if (rk_camera_platform_data.register_dev[i].device_info.name) {
            if (rk_camera_platform_data.register_dev[i].link_info.bus_id == RK_CAM_PLATFORM_DEV_ID_0) {
                if (!host_registered_0) {
                    platform_device_register(&rk_device_camera_host_0);
                    host_registered_0 = 1;
                }
            } else if (rk_camera_platform_data.register_dev[i].link_info.bus_id == RK_CAM_PLATFORM_DEV_ID_1) {
                if (!host_registered_1) {
                    platform_device_register(&rk_device_camera_host_1);
                    host_registered_1 = 1;
                }
            } 
        }
    }

    for (i=0; i<RK_CAM_NUM; i++) {
        if (rk_camera_platform_data.register_dev[i].device_info.name) {
            platform_device_register(&rk_camera_platform_data.register_dev[i].device_info);
        }
    }
 #ifdef PMEM_CAM_NECESSARY
	platform_device_register(&android_pmem_cam_device);
 #endif

	return 0;
}

module_init(rk_register_camera_devices);
#endif //#ifdef CONFIG_VIDEO_RK
