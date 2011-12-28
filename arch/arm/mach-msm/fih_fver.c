#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>


//#include "../../../drivers/char/diag/diagfwd.h"
extern int read_efsfver(int efs_page, char *efs_buf); /* FIH, JiaHao, 2011/09/14 */

#define PROC_FVER "fver"

char *efs_buf;
int  efs_len;

/* This function is called at the beginning of a sequence. ie, when:
 * 1.the /proc file is read (first time)
 * 2.after the function stop (end of sequence) */
static void *efs_seq_start(struct seq_file *s, loff_t *pos)
{
	if (((*pos)*PAGE_SIZE) >= efs_len) return NULL;
	return (void *)((unsigned long) *pos+1);
}

/* This function is called after the beginning of a sequence.
 * It's called untill the return is NULL (this ends the sequence). */
static void *efs_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	++*pos;
	return efs_seq_start(s, pos);
}

/* This function is called at the end of a sequence */
static void efs_seq_stop(struct seq_file *s, void *v)
{
	/* nothing to do, we use a static value in start() */
}

/* This function is called for each "step" of a sequence */
static int efs_seq_show(struct seq_file *s, void *v)
{
	int n = (int)v-1;

	if (efs_len < (PAGE_SIZE*(n+1))) {
		seq_write(s, (efs_buf+(PAGE_SIZE*n)), (efs_len - (PAGE_SIZE*n)));
	} else {
		seq_write(s, (efs_buf+(PAGE_SIZE*n)), PAGE_SIZE);
	}

	return 0;
}

/* This structure gather "function" to manage the sequence */
static struct seq_operations efs_seq_ops = {
	.start = efs_seq_start,
	.next = efs_seq_next,
	.stop = efs_seq_stop,
	.show = efs_seq_show
};

/* This function is called when the /proc file is open. */
static int efs_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &efs_seq_ops);
};

/* This structure gather "function" that manage the /proc file */
static struct file_operations efs_file_ops = {
	.owner = THIS_MODULE,
	.open = efs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

/* This function is called when the module is loaded */
static int __init efs_fver_init(void)
{
	struct proc_dir_entry *entry;
	int i, r;

	/* get the total length of fver in byte */
	efs_len = read_efsfver(0, NULL); 
	if (0 >= efs_len) {
		efs_len = read_efsfver(0, NULL);
	}

	/* check fver */
	if (0 >= efs_len) {
		printk("fih_fver: [ERROR] efs_len = %d\n", efs_len);
		return (-1);
	}
	printk("fih_fver: [INFO] efs_len = %d\n", efs_len);

	/* alloc fver */
	i = (efs_len + PAGE_SIZE) - (efs_len & (PAGE_SIZE - 1));
	printk("fih_fver: [INFO] kzalloc = %d\n", i);
	efs_buf = kzalloc(i, GFP_KERNEL);
	if (NULL == efs_buf) {
		printk("fih_fver: [ERROR] efs_buf is NULL\n");
		return (-1);
	}

	/* read fver */
	for (i=0; i<=(efs_len/PAGE_SIZE); i++) {
		r = read_efsfver(i, (efs_buf+(PAGE_SIZE*i)));
		if (0 > r) {
			printk("fih_fver: [ERROR] r = %d\n", r);
			return (-1);
		}
	}

	/* create fver */
	entry = create_proc_entry(PROC_FVER, 0, NULL);
	if (entry) {
		entry->proc_fops = &efs_file_ops;
	}

	return (0);
}

/* This function is called when the module is unloaded. */
static void __exit efs_fver_exit(void)
{
	if (efs_buf) kfree(efs_buf);
	remove_proc_entry(PROC_FVER, NULL);
}

late_initcall(efs_fver_init);
module_exit(efs_fver_exit);
