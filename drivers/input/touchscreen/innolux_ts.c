#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
//#include <linux/bu21018mwv.h>
//#include <linux/bu21018mwv_fw.h>
//#include <linux/bu21018mwv_fw1.h>
#include "../../../arch/arm/mach-msm/smd_private.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include <linux/slab.h>	// temp for BSP 4040
//
extern unsigned int fih_get_product_id(void);
extern unsigned int fih_get_product_phase(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void innolux_ts_early_suspend(struct early_suspend *h);
static void innolux_ts_late_resume(struct early_suspend *h);
#endif

int innolux_ts_active = 0;

//
static int t_max_x, t_min_x, t_max_y, t_min_y;

//
static u8 buffer[12];

//
static int gPrevious = 0;

// for FQC firmware version
static int msm_cap_touch_fw_version;

struct innolux_ts_info {
    struct i2c_client *client;
    struct input_dev  *touch_input;
    struct input_dev  *keyevent_input;
    struct work_struct wqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend es;
#endif
    int irq;
    bool suspend_state;
} *innolux_ts;

module_param_named(
    fw_version, msm_cap_touch_fw_version, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static int innolux_ts_recv(u8 *buf, u32 count)
{
    struct i2c_msg msgs[] = {
        [0] = {
            .addr = innolux_ts->client->addr,
            .flags = I2C_M_RD,
            .len = count,
            .buf = buf,
        },
    };

	return (i2c_transfer(innolux_ts->client->adapter, msgs, 1) < 0) ? -EIO : 0;
}

static int innolux_ts_send(u8 *buf, u32 count)
{
	struct i2c_msg msgs[]= {
		[0] = {
            .addr = innolux_ts->client->addr,
			.flags = 0,
			.len = count,
			.buf = buf,
        },
    };

	return (i2c_transfer(innolux_ts->client->adapter, msgs, 1) < 0) ? -EIO : 0;
}

static void innolux_ts_isr_workqueue(struct work_struct *work)
{
    struct innolux_ts_info *ts = container_of(work, struct innolux_ts_info, wqueue);
    
#ifdef CONFIG_FIH_FTM
	int x1, y1;
#else
    int x1, y1, x2, y2;
#endif
    char cmd[2] = { 0x0b, 0x80 };
    char cmd1[1] = { 0x00 };

    disable_irq(ts->irq);
    
    innolux_ts_send(cmd1, ARRAY_SIZE(cmd1));
    
    innolux_ts_recv(buffer, ARRAY_SIZE(buffer));

    printk(KERN_INFO "[Touch] %s: buffer[0] = %x\n", __func__, buffer[0]);
    
    if (buffer[0] == 0x81 || buffer[0] == 0x82)
    {
//        for (i=0;i<12;i++)
//            printk(KERN_INFO "buffer[%d] = %x\n", i, buffer[i]);

        x1 = (buffer[1] << 8) + buffer[2];
        y1 = (buffer[3] << 8) + buffer[4];

#ifdef CONFIG_FIH_FTM
		input_report_abs(ts->touch_input, ABS_X, x1);
		input_report_abs(ts->touch_input, ABS_Y, y1);
		input_report_abs(ts->touch_input, ABS_PRESSURE, 255);
		input_report_key(ts->touch_input, BTN_TOUCH, 1);
#else
		input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 255);
		input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x1);
		input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y1);
		input_mt_sync(ts->touch_input);
//		input_sync(ts->touch_input);

        if (buffer[0] == 0x82)
        {
            x2 = (buffer[5] << 8) + buffer[6];
            y2 = (buffer[7] << 8) + buffer[8];
            
            input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 255);
            input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x2);
            input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y2);
            input_mt_sync(ts->touch_input);
        }
#endif
        input_sync(ts->touch_input);
		gPrevious = 0;
        printk(KERN_INFO "x1 = %d , y1 = %d\n", x1, y1);
    }
    
    if (buffer[0] == 0x83)
    {
    	printk(KERN_INFO "buffer[9] = %d\n", buffer[9]);
    	
    	if (buffer[9] == 1)
    		input_report_key(ts->keyevent_input, KEY_BACK, 1);
    	else if (buffer[9] == 2)
    		input_report_key(ts->keyevent_input, KEY_MENU, 1);
    	else if (buffer[9] == 4)
    		input_report_key(ts->keyevent_input, KEY_HOME, 1);
    	else if (buffer[9] == 8)
    		input_report_key(ts->keyevent_input, KEY_SEARCH, 1);
    		
    	gPrevious = buffer[9];
	}

    if (buffer[0] == 0x80)
    {
    	if (gPrevious == 0)
    	{
        	innolux_ts_send(cmd, ARRAY_SIZE(cmd));
        	innolux_ts_send(cmd1, ARRAY_SIZE(cmd1));
#ifdef CONFIG_FIH_FTM
			input_report_abs(ts->touch_input, ABS_PRESSURE, 0);
			input_report_key(ts->touch_input, BTN_TOUCH, 0);
#else
			input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(ts->touch_input);
#endif
			input_sync(ts->touch_input);
		}
		else
		{
        	if (gPrevious == 1)
		    	input_report_key(ts->keyevent_input, KEY_BACK, 0);
        	else if (gPrevious == 2)
            	input_report_key(ts->keyevent_input, KEY_MENU, 0);
        	else if (gPrevious == 4)
            	input_report_key(ts->keyevent_input, KEY_HOME, 0);
        	else if (gPrevious == 8)
            	input_report_key(ts->keyevent_input, KEY_SEARCH, 0);
        
			input_sync(ts->keyevent_input);
			gPrevious = 0;
		}
    }

    enable_irq(ts->irq);
}

static irqreturn_t innolux_ts_isr(int irq, void * handle)
{
    struct innolux_ts_info *ts = handle;

    if (!ts->suspend_state)
        schedule_work(&ts->wqueue);

    return IRQ_HANDLED;
}

static int innolux_ts_misc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    char cmd1[2] = {0x0a, 0x08};
    char cmd2[1] = {0x00};
    
	switch (cmd)
	{
		case 0:
		    printk(KERN_INFO "[Touch] Write 0x08\n");
		    innolux_ts_send(cmd1, ARRAY_SIZE(cmd1));
			break;
		case 10001:
            innolux_ts_recv(buffer, ARRAY_SIZE(buffer));
            printk(KERN_INFO "[Touch] buffer[0] = %x\n", buffer[0]);
			break;
		case 10002:
		if (gpio_tlmm_config(GPIO_CFG(56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
        	printk(KERN_ERR "[Touch] %s: gpio_tlmm_config: 56 failed.\n", __func__);
    	else
    	{
        	gpio_set_value(56, 0);  // 0
        	msleep(1);
        	gpio_set_value(56, 1);
        }
			break;
		case 10003:
		    printk(KERN_INFO "[Touch] Write 0x00\n");
		    innolux_ts_send(cmd2, ARRAY_SIZE(cmd2));
			break;
		default:
			printk(KERN_ERR "[Touch] %s: unknow IOCTL.\n", __func__);
			break;
	}

	return 0;
}

static struct file_operations innolux_ts_misc_fops = {
	.owner =   THIS_MODULE,
	.ioctl =   innolux_ts_misc_ioctl,
};

static struct miscdevice innolux_ts_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "innolux_ts",
	.fops = &innolux_ts_misc_fops,
};

static int innolux_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *touch_input;
	struct input_dev *keyevent_input;
	
    char cmd1[1] = { 0x00 };
    
    // Check I2C
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ERR "[Touch] %s: Check I2C functionality failed.\n", __func__);
        return -ENODEV;
    }

    innolux_ts = kzalloc(sizeof(struct innolux_ts_info), GFP_KERNEL);
    
    if (innolux_ts == NULL)
    {
        printk(KERN_ERR "[Touch] %s: Can not allocate memory.\n", __func__);
        return -ENOMEM;
    }

    // Confirm Touch Chip
    innolux_ts->client = client;
    
    i2c_set_clientdata(client, innolux_ts);
    
    innolux_ts->client->addr = 0x00;
#if 0    
	// Initial power domain
	vreg_ldo12 = vreg_get(NULL, "gp9");

	if (IS_ERR(vreg_ldo12))
    {
		printk(KERN_ERR "%s: gp9 vreg get failed (%ld)\n", __func__, PTR_ERR(vreg_ldo12));
    	goto err1;
	}
	
	rc = vreg_set_level(vreg_ldo12, 3000);
    
	if (rc)
    {
		printk(KERN_ERR "%s: vreg LDO12 set level failed (%d)\n", __func__, rc);
    	goto err1;
	}
			
	rc = vreg_enable(vreg_ldo12);
    
	if (rc)
    {
        printk(KERN_ERR "%s: LDO12 vreg enable failed (%d)\n", __func__, rc);
    	goto err1;
    }
#endif

    if (innolux_ts_send(cmd1, ARRAY_SIZE(cmd1)) < 0)
    {
    	printk(KERN_ERR "[Touch] %s: innolux_ts_send() < 0\n", __func__);
    	goto err1;
    }
    
    if (innolux_ts_recv(buffer, ARRAY_SIZE(buffer)) < 0)
    {
    	printk(KERN_ERR "[Touch] %s: innolux_ts_recv() < 0\n", __func__);
		goto err1;
	}
    
    if (buffer[0] == 0 || buffer[0] == 0x55)
    {
    	printk(KERN_ERR "[Touch] %s: INNOLUX touch I2C fail!\n", __func__);
    	goto err1;
    }
    
	if (gpio_tlmm_config(GPIO_CFG(56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		printk(KERN_ERR "[Touch] %s: gpio_tlmm_config: 56 failed.\n", __func__);
	else
	{
		gpio_set_value(56, 1);
	}
	
	//
	touch_input = input_allocate_device();
	
    if (touch_input == NULL)
	{
		printk(KERN_ERR "[Touch] %s: Can not allocate memory for touch input device.\n", __func__);
		goto err1;
	}
	
	keyevent_input = input_allocate_device();
	
    if (keyevent_input == NULL)
	{
		printk(KERN_ERR "[Touch] %s: Can not allocate memory for key input device.\n", __func__);
		goto err2;
	}
	
	touch_input->name  = "innolux_ts";
	touch_input->phys  = "innolux_ts/input0";
	set_bit(EV_KEY, touch_input->evbit);
	set_bit(EV_ABS, touch_input->evbit);
	set_bit(EV_SYN, touch_input->evbit);
	
	t_max_x = 480;//479;
	t_min_x = 0;
	t_max_y = 800;//799;
	t_min_y = 0;
	
#ifdef CONFIG_FIH_FTM
	set_bit(BTN_TOUCH, touch_input->keybit);
	input_set_abs_params(touch_input, ABS_X, t_min_x, t_max_x, 0, 0);
	input_set_abs_params(touch_input, ABS_Y, t_min_y, t_max_y, 0, 0);
	input_set_abs_params(touch_input, ABS_PRESSURE, 0, 255, 0, 0);
#else
    input_set_abs_params(touch_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(touch_input, ABS_MT_POSITION_X, t_min_x, t_max_x, 0, 0);
    input_set_abs_params(touch_input, ABS_MT_POSITION_Y, t_min_y, t_max_y, 0, 0);
#endif
	
    keyevent_input->name  = "innolux_ts_key";
    keyevent_input->phys  = "innolux_ts/input1";
    set_bit(EV_KEY, keyevent_input->evbit);
    set_bit(KEY_MENU, keyevent_input->keybit);
    set_bit(KEY_HOME, keyevent_input->keybit);
    set_bit(KEY_BACK, keyevent_input->keybit);
    set_bit(KEY_SEARCH, keyevent_input->keybit);
    
	innolux_ts->touch_input = touch_input;
	
	if (input_register_device(innolux_ts->touch_input))
	{
		printk(KERN_ERR "[Touch] %s: Can not register touch input device.\n", __func__);
		goto err3;
	}
    
	innolux_ts->keyevent_input = keyevent_input;
	
	if (input_register_device(innolux_ts->keyevent_input))
	{
		printk(KERN_ERR "[Touch] %s: Can not register key input device.\n", __func__);
		goto err4;
	}
	
	if (misc_register(&innolux_ts_misc_dev))
	{
		printk(KERN_ERR "[Touch] %s: Can not register misc device.\n", __func__);
		goto err5;
	}

   	INIT_WORK(&innolux_ts->wqueue, innolux_ts_isr_workqueue);
    
    gpio_tlmm_config(GPIO_CFG(42, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    innolux_ts->irq = MSM_GPIO_TO_INT(42);
    
    if (request_irq(innolux_ts->irq, innolux_ts_isr, IRQF_TRIGGER_FALLING, client->dev.driver->name, innolux_ts))
    {
        printk(KERN_ERR "[Touch] %s: Request IRQ failed.\n", __func__);
        goto err6;
    }
    
    innolux_ts->suspend_state = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	innolux_ts->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	innolux_ts->es.suspend = innolux_ts_early_suspend;
	innolux_ts->es.resume = innolux_ts_late_resume;
	register_early_suspend(&innolux_ts->es);
#endif

	innolux_ts_active = 1;
    return 0;
    
err6:
	misc_deregister(&innolux_ts_misc_dev);
err5:
    input_unregister_device(innolux_ts->keyevent_input);
err4:
    input_unregister_device(innolux_ts->touch_input);
err3:
    input_free_device(keyevent_input);
err2:
    input_free_device(touch_input);
err1:
    dev_set_drvdata(&client->dev, 0);
    kfree(innolux_ts);
    printk(KERN_ERR "[Touch] %s: Failed.\n", __func__);
    return -1;
}

static int innolux_ts_remove(struct i2c_client * client)
{
	printk(KERN_INFO "[Touch] %s\n", __func__);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&innolux_ts->es);
#endif

	misc_deregister(&innolux_ts_misc_dev);
	input_unregister_device(innolux_ts->keyevent_input);
	input_unregister_device(innolux_ts->touch_input);
    dev_set_drvdata(&client->dev, 0);
    kfree(innolux_ts);
    return 0;
}

static int innolux_ts_suspend(struct i2c_client *client, pm_message_t state)
{
    return 0;
}

static int innolux_ts_resume(struct i2c_client *client)
{
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void innolux_ts_early_suspend(struct early_suspend *h)
{
    char cmd[2] = {0x0a, 0x08};
    printk(KERN_INFO "[Touch] %s\n", __func__);
    
    innolux_ts->suspend_state = 1;
    disable_irq(innolux_ts->irq);
    
    innolux_ts_send(cmd, ARRAY_SIZE(cmd));
    
    if (gpio_tlmm_config(GPIO_CFG(56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
       	printk(KERN_ERR "[Touch] %s: gpio_tlmm_config: 56 failed.\n", __func__);
    else
       	gpio_set_value(56, 0);  // 1
}

static void innolux_ts_late_resume(struct early_suspend *h)
{
    printk(KERN_INFO "[Touch] %s\n", __func__);
    
	if (gpio_tlmm_config(GPIO_CFG(56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
       	printk(KERN_ERR "[Touch] %s: gpio_tlmm_config: 56 failed.\n", __func__);
    else
    {
       	gpio_set_value(56, 0);  // 0
       	msleep(1);
       	gpio_set_value(56, 1);
    }
    
    innolux_ts->suspend_state = 0;
    enable_irq(innolux_ts->irq);
}
#endif

static const struct i2c_device_id innolux_ts_id[] = {
    { "innolux_ts", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, innolux_ts_id);

static struct i2c_driver innolux_ts_i2c_driver = {
   .driver = {
      .name   = "innolux_ts",
   },
   .id_table   = innolux_ts_id,
   .probe      = innolux_ts_probe,
   .remove     = innolux_ts_remove,
   .suspend    = innolux_ts_suspend,
   .resume     = innolux_ts_resume,
};

static int __init innolux_ts_init( void )
{
    printk(KERN_INFO "[Touch] %s\n", __func__);
    return i2c_add_driver(&innolux_ts_i2c_driver);
}

static void __exit innolux_ts_exit( void )
{
    printk(KERN_INFO "[Touch] %s\n", __func__);
    i2c_del_driver(&innolux_ts_i2c_driver);
}

module_init(innolux_ts_init);
module_exit(innolux_ts_exit);

MODULE_DESCRIPTION("innolux_ts Touchscreen driver");
MODULE_AUTHOR("FIH Div2 SW2 BSP");
MODULE_LICENSE("GPL");
