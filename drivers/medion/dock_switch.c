#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/slab.h>
#include <linux/gpio.h>   
#include <linux/interrupt.h> 
#include <linux/delay.h>   

#define DRIVER_NAME     "dock"

struct dock_switch_data {
    struct switch_dev sdev;
    unsigned gpio;   
    unsigned irq;     
    struct work_struct work;
};

static ssize_t switch_print_name(struct switch_dev *sdev, char *buf)                                  
{
    return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t switch_print_state(struct switch_dev *sdev, char *buf)
{
    return sprintf(buf, "%s\n", (switch_get_state(sdev) ? "1" : "0"));
}

#if 0
extern void enable_usb3 (void);
extern void disable_usb3 (void);
static int old_state=0;
#endif
int plug=0, boot=1;
static void dock_switch_work(struct work_struct *work)
{
    int state;
    int count;
    struct dock_switch_data *pSwitch =
               container_of(work, struct dock_switch_data, work);

    state = gpio_get_value(pSwitch->gpio);
    printk(" dock_switch, gpio is %x\n", state);
#if 0
    if(state !=  old_state){
		old_state = state;
		if( state == 1)
			enable_usb3();
		else
			disable_usb3();
    }
#endif
	if(state == 0 && boot==0)
	{
		for(count =0; count<10;count++)
		{	
			if(plug)
				break;
			msleep(1000);
		}
		if(count==10)
			switch_set_state(&pSwitch->sdev, state);
    		
	}
	else
	{
		switch_set_state(&pSwitch->sdev, state);
		boot=0;
	}
}


static irqreturn_t dock_interrupt(int irq, void *dev_id)
{
    struct dock_switch_data *pSwitch = 
               (struct dock_switch_data *)dev_id;

    plug = gpio_get_value(pSwitch->gpio);
    
    schedule_work(&pSwitch->work);

    return IRQ_HANDLED;
}

static int dock_switch_probe(struct platform_device *pdev)
{
    struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;  
    struct dock_switch_data *switch_data;
    int ret = -EBUSY;

    if (!pdata)
        return -EBUSY;
	
    switch_data = kzalloc(sizeof(struct dock_switch_data), GFP_KERNEL);
    if (!switch_data)
        return -ENOMEM;

    switch_data->gpio = pdata->gpio;                  
    switch_data->irq = gpio_to_irq(pdata->gpio);       
    switch_data->sdev.print_state = switch_print_state;
    switch_data->sdev.name = DRIVER_NAME;
    switch_data->sdev.print_name = switch_print_name;
    switch_data->sdev.print_state = switch_print_state;
    ret = switch_dev_register(&switch_data->sdev);
    if (ret < 0)
        goto err_register_switch;
	
    INIT_WORK(&switch_data->work, dock_switch_work);

    tegra_gpio_enable(24);

    ret = request_irq(switch_data->irq, dock_interrupt, 
                  IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                  DRIVER_NAME, switch_data);
    if (ret) {
        printk("dock_switch request irq failed\n");
        goto err_request_irq;
	    }

    dock_switch_work(&switch_data->work);
    return 0;

err_request_irq:
    free_irq(switch_data->irq, switch_data);   
err_register_switch:
    kfree(switch_data);
    return ret;
}

static int __devexit dock_switch_remove(struct platform_device *pdev)
{
    struct dock_switch_data *switch_data = platform_get_drvdata(pdev);
	
    cancel_work_sync(&switch_data->work);
    switch_dev_unregister(&switch_data->sdev);
    kfree(switch_data);
    
    return 0;
}

static int dock_switch_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int dock_switch_resume(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver dock_switch_driver = {
    .probe      = dock_switch_probe,
    .remove     = __devexit_p(dock_switch_remove),
    .suspend    = dock_switch_suspend,
    .resume     = dock_switch_resume, 
    .driver     = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },
};

static int __init dock_switch_init(void)
{
    int err;

    err = platform_driver_register(&dock_switch_driver);
    if (err)
        goto err_exit;

    return 0;

err_exit:
    printk(KERN_INFO "Dock Switch register Failed! ----->>>\n");
    return err;
}

static void __exit dock_switch_exit(void)
{
    printk(KERN_INFO "Dock Switch driver unregister ----->>>\n");
    platform_driver_unregister(&dock_switch_driver);
}

module_init(dock_switch_init);
module_exit(dock_switch_exit);

MODULE_DESCRIPTION("Dock Switch Driver");
MODULE_LICENSE("GPL");
