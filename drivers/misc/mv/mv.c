#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/proc_fs.h>
#include <uapi/linux/sched/types.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/kernel_stat.h>
#include <linux/cpu.h>
#include <linux/memblock.h>
#include <linux/byteorder/generic.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include "../hqsysfs/hqsys_misc.h"
#include "../../mmc/core/card.h"

#define PROC_MV_FILE "mv"
#define K(x) ((x) << (PAGE_SHIFT - 10))
extern  round_kbytes_to_readable_mbytes(unsigned int k);
extern	ssize_t hq_emmcinfo(char *buf);
extern char kernel_fwrew[10];
static struct proc_dir_entry *mv_proc;
char part_num[30];


static int mv_proc_show(struct seq_file *file, void*data)
{
	struct sysinfo i;
	si_meminfo(&i);
	int status;
	char RAM_size[8];
	char emmc_size[8];
	uint32_t manfidd;
	char manfid[32];
	char *product_version;

	status = hq_emmcinfo(emmc_size);

	if (round_kbytes_to_readable_mbytes(K(i.totalram)) >= 1024) {
		status = sprintf(RAM_size, "%dGB", round_kbytes_to_readable_mbytes(K(i.totalram))/1024);
	} else{
		status = sprintf(RAM_size, "%dMB", round_kbytes_to_readable_mbytes(K(i.totalram)));
	}

	manfidd = mmc_get_manfid();
	snprintf(manfid, 5,"0x%x", manfidd);

	seq_printf(file,"D: %s %s\n", manfid, RAM_size);
	seq_printf(file,"U: %s %s %s 0x%x\n", manfid, emmc_size, part_num, kernel_fwrew[0]);

	return 0;
}

static int __init part_num_setup(char *str)
{
	unsigned int i;
	sprintf(part_num,"%s",str);
	return 1;
}
__setup("part_num=", part_num_setup);


static int mv_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv_proc_show, NULL);
}

static const struct file_operations mv_proc_fops = {
	.open		= mv_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int proc_mv_init(void)
{
	mv_proc = proc_create(PROC_MV_FILE, 0644, NULL, &mv_proc_fops);
	if (mv_proc == NULL)
	{
		pr_info("create mv node failed:%d\n",mv_proc);
	}
	return 0;
}

void proc_mv_exit(void)
{
	 pr_info("exit mv !\n");
	 remove_proc_entry(PROC_MV_FILE,NULL);
}

MODULE_LICENSE("GPL");
module_init(proc_mv_init);
module_exit(proc_mv_exit);