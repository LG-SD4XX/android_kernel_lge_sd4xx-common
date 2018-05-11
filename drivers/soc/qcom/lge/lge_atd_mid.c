#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/printk.h>
#include <soc/qcom/lge/board_lge.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/sched.h>


static struct kobject *atd_kobject;
static int atd_status =0 ;


static ssize_t atd_status_show(struct kobject *kobj, struct kobj_attribute *attr,
                      char *buf)
{
        return sprintf(buf, "%d\n", atd_status);
}

static ssize_t atd_status_store(struct kobject *kobj, struct kobj_attribute *attr,
                      const char *buf, size_t count)
{
        sscanf(buf, "%du", &atd_status);
        return count;
}


static struct kobj_attribute atd_status_attribute =
	__ATTR(atd_status,  0660, atd_status_show,atd_status_store);



static struct wake_lock pwroff_lock;

static int lge_atd_status(void){
	return atd_status;
}
static int lge_atd_mid_poweroff_thread(void *data){
	int loopcnt = 50 ;
	if( lge_atd_status() && lge_get_factory_boot() ){
		while(loopcnt-- > 0) {
			pr_err("waiting for ATD power off thread, loopcnt : %d , ATD_STATUS : %d\n",
									loopcnt,lge_atd_status());
			msleep(100);
		}
		pr_err("timeout for ATD power off thread\n");
		kernel_power_off();
	}
	pr_err("exiting for ATD power off thread, loopcnt : %d , ATD_STATUS : %d \n",loopcnt,lge_atd_status());
	return 0;
}

void lge_atd_mid_poweroff(void){
	if( lge_atd_status() && lge_get_factory_boot() ){
		wake_lock(&pwroff_lock);
		kthread_run(lge_atd_mid_poweroff_thread,NULL,"ATD_PWROFF");
		pr_err("created waiting ATD power off thread, ATD_STATUS : %d \n",lge_atd_status());

	}

}



static int __init atd_mid_init (void)
{
        int error = 0;


        atd_kobject = kobject_create_and_add("atd_mid",
                                                 kernel_kobj);
        if(!atd_kobject){
                pr_err("failed to create the atd_mid kobject \n");
                return -ENOMEM;
		}
        error = sysfs_create_file(atd_kobject, &atd_status_attribute.attr);
        if (error) {
                pr_err("failed to create the atd_mid file in /sys/kernel/atd_mid/atd_status \n");
				kobject_put(atd_kobject);
        }
		wake_lock_init(&pwroff_lock, WAKE_LOCK_SUSPEND, "PWROFF");
        pr_info("Module initialized successfully \n");

        return error;
}

static void __exit atd_mid_exit (void)
{
        pr_info ("Module un initialized successfully \n");
        kobject_put(atd_kobject);
}

module_init(atd_mid_init);
module_exit(atd_mid_exit);
MODULE_DESCRIPTION("LGE ATD STATE driver");
MODULE_LICENSE("GPL v2");
