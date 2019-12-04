/*
 * Модуль создаёт виртуальный диск необходимого размера, адаптировано под kernel 5+.
 * Данный модуль можно использовать либо как каркас блочного утройства под новое ядро.
*/

#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/genhd.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/version.h>
#include <linux/blk_types.h>
#include <uapi/linux/hdreg.h>
#include <uapi/linux/cdrom.h>
#include "virtual_disk.h"

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_AUTHOR("X-Shar (ru-sfera.org)");

#define MY_DEVICE_NAME "xd"
#define DEV_MINORS  16
#define KERNEL_SECTOR_SIZE    512

/*
 * Параметры модуля можно ввести при загрузки модуля
 */

static int diskmb = 4;
module_param_named(size, diskmb, int, 0); // размер диска в Mb, по умолчанию - 4Mb

static int major = 0;
module_param(major, int, 0);
static int hardsect_size = KERNEL_SECTOR_SIZE;
module_param(hardsect_size, int, 0);
static int ndevices = 4;
module_param(ndevices, int, 0);

/*
 * Структура, которая описывает функции модуля.
 * Чтение/запись будет происходить из очереди (Так нужно в Линуксе для блочного устройства в отличие от например символьного устройства).
 * Некоторые программы такие, как например gparted требуют вывод геометрии диска, поэтому нужно также реализовать функции my_ioctl и my_getgeo.
 */
static struct block_device_operations mybdrv_fops = {
   .owner = THIS_MODULE,
   .ioctl = my_ioctl,
   .getgeo = my_getgeo
};

/*
 * Структура, которая описывает наш диск
*/
struct disk_dev {
   int size;                    // Размер устройства
   u8 *data;                    // Указатель на приватный буфер устройства (Это у нас будет выделенный участок памяти для работы в ОЗУ)
   spinlock_t lock;             // Указатель на спинлок */
   struct blk_mq_tag_set tag_set;
   struct request_queue *queue; // Указатель на очередь для устройства */
   struct gendisk *gd;          // endisk structure */
};

/*
 * Структура для работа с очередью.
 * Данная структура необходимо при создании очереди.
 * simple_request - Указатель на функцию обработки очереди.
 */
static struct blk_mq_ops _mq_ops = {
    .queue_rq =  _queue_rq,
};

static struct disk_dev *Devices = NULL;
static int nsectors;



/*
 * Инициализация модуля
 */
module_init(blk_init);
module_exit(blk_exit);

//Реализация функций основных функций, нужно реализовать функции, которые находятся в virtual_disk.h

/*
 * Получение геометрии диска
 */
static int my_getgeo(struct block_device *bdev, struct hd_geometry *geo) {

   unsigned long sectors = (diskmb * 1024) * 1024;
   printk( KERN_INFO "+++ getgeo\n" );
   geo->heads = 4;
   geo->sectors = 16;
   geo->cylinders = sectors / geo->heads / geo->sectors;
   geo->start = geo->sectors;
   return 0;
};

/*
 * Получение геометрии диска через ioctl
 */
static int my_ioctl( struct block_device *bdev, fmode_t mode,
                     unsigned int cmd, unsigned long arg ) {
	printk("+++ ioctl cmd=%d\n", cmd);
   switch( cmd ) {
      case HDIO_GETGEO: {
         struct hd_geometry geo;
         printk("+++ ioctk HDIO_GETGEO\n");
         my_getgeo( bdev, &geo );
         if( copy_to_user( (void __user *)arg, &geo, sizeof( geo ) ) )
            return -EFAULT;
         return 0;
      }
      default:
    	 printk("+++ ioctl unknown command\n");
         return -ENOTTY;
   }
}

// настройка внутреннего устройства
static void setup_device( struct disk_dev *dev, int which ) {

   memset(dev, 0, sizeof(struct disk_dev));
   dev->size = diskmb * 1024 * 1024;

   //Выделение памяти для работы в ОЗУ
   dev->data = vmalloc(dev->size);
   if(dev->data == NULL) {
      printk("+++ vmalloc failure.\n");
      return;
   }
   //Перед настройкой очереди, нужно инициировать spin_lock
   spin_lock_init(&dev->lock);
   
   //Под старое ядро
   //dev->queue = blk_mq_init_queue(simple_request, &dev->lock);
   
   //Под новое ядро
   dev->queue = blk_mq_init_sq_queue(&dev->tag_set, &_mq_ops, 128, BLK_MQ_F_SHOULD_MERGE);

   if( dev->queue == NULL ) goto out_vfree;

   // установка раздела аппаратного сектора
   blk_queue_logical_block_size( dev->queue, hardsect_size );
   dev->queue->queuedata = dev;
   dev->gd = alloc_disk( DEV_MINORS );
   if( ! dev->gd ) {
	  printk( "+++ alloc_disk failure\n" );
      goto out_vfree;
   }

   //Инициализация структуры устройства
   dev->gd->major = major;
   dev->gd->minors = DEV_MINORS;
   dev->gd->first_minor = which * DEV_MINORS;
   dev->gd->fops = &mybdrv_fops;
   dev->gd->queue = dev->queue;
   dev->gd->private_data = dev;
   snprintf( dev->gd->disk_name, 32, MY_DEVICE_NAME"%c", which + 'a' );
   set_capacity( dev->gd, nsectors * ( hardsect_size / KERNEL_SECTOR_SIZE ) );

   //Добавление устройства в систему, вообще опасная функция как понял из доков, т.к. если что-то неправильно сделали, может всё упасть, хе-хе.
   //Ну надеюсь всё будет пучком.)))
   add_disk( dev->gd );
   return;

out_vfree:
   if( dev->data ) vfree( dev->data );
}

//Интерфейсная функция, которую вызовет ядро в момент загрузки модуля
static int __init blk_init( void ) {
   int i;
   nsectors = diskmb * 1024 * 1024 / hardsect_size;
   major = register_blkdev( major, MY_DEVICE_NAME );
   if( major <= 0 ) {
      printk( "+++ unable to get major number\n" );
      return -EBUSY;
   }

   // выделение массива для хранения данных устройства
   Devices = kmalloc( ndevices * sizeof( struct disk_dev ), GFP_KERNEL );
   if( Devices == NULL ) goto out_unregister;
   for( i = 0; i < ndevices; i++ ) // инициализировать каждое устройство
      setup_device( Devices + i, i );
   return 0;

out_unregister:
   unregister_blkdev( major, MY_DEVICE_NAME );
   return -ENOMEM;
}

/*
 * Данную функцию вызовет ядро, в момент удаления модуля
 */
static void blk_exit( void ) {
   int i;
   for( i = 0; i < ndevices; i++ ) {
      struct disk_dev *dev = Devices + i;
      if( dev->gd ) {
         del_gendisk( dev->gd );
         put_disk(dev->gd);
      }
         blk_cleanup_queue( dev->queue );
      if( dev->data ) vfree( dev->data );
   }
   unregister_blkdev( major, MY_DEVICE_NAME );
   kfree( Devices );
}

static int do_simple_request(struct request *rq, unsigned int *nr_bytes)
{
    int ret = 0;
    struct bio_vec bvec;
    struct req_iterator iter;
    struct disk_dev *dev = rq->q->queuedata;

    loff_t pos = blk_rq_pos(rq) << SECTOR_SHIFT;
    loff_t dev_size = (loff_t)(dev->size << SECTOR_SHIFT);

    printk("+++ request start from sector %lld \n", blk_rq_pos(rq));

    rq_for_each_segment(bvec, rq, iter)
    {
        unsigned long b_len = bvec.bv_len;

        void* b_buf = page_address(bvec.bv_page) + bvec.bv_offset;

        if ((pos + b_len) > dev_size)
            b_len = (unsigned long)(dev_size - pos);

        if (rq_data_dir(rq))//WRITE
            memcpy(dev->data + pos, b_buf, b_len);
        else//READ
            memcpy(b_buf, dev->data + pos, b_len);

        pos += b_len;
        *nr_bytes += b_len;
    }

    return ret;
}

static blk_status_t _queue_rq(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data* bd)
{
    unsigned int nr_bytes = 0;
    blk_status_t status = BLK_STS_OK;
    struct request *rq = bd->rq;

    //Получение адреса очереди
    blk_mq_start_request(rq);

    if (do_simple_request(rq, &nr_bytes) != 0)
    {
        status = BLK_STS_IOERR;
    }

    printk("+++ request process %d bytes\n", nr_bytes);


    if (blk_update_request(rq, status, nr_bytes))
    {
        printk("+++ blk_update_request error");
    }

    __blk_mq_end_request(rq, status);

    return BLK_STS_OK;
}
