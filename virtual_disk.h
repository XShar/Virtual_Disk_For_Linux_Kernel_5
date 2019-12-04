/*
 * virtual_disk.h
 *
 *  Created on: 2 дек. 2019 г.
 *      Author: oleg
 */

#ifndef VIRTUAL_DISK_VIRTUAL_DISK_H_
#define VIRTUAL_DISK_VIRTUAL_DISK_H_

static void blk_exit(void);
static int __init blk_init(void);
static int my_ioctl(struct block_device *bdev, fmode_t mode,
                     unsigned int cmd, unsigned long arg);
static int my_getgeo(struct block_device *bdev, struct hd_geometry *geo);
static blk_status_t _queue_rq(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data* bd);


#endif /* VIRTUAL_DISK_VIRTUAL_DISK_H_ */
