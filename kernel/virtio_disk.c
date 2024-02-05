//
// driver for qemu's virtio disk device.
// uses qemu's mmio interface to virtio.
//
// qemu ... -drive file=fs.img,if=none,format=raw,id=x0 -device virtio-blk-device,drive=x0,bus=virtio-mmio-bus.0
//

#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "buf.h"
#include "virtio.h"

// the address of virtio mmio register r.
#define R(offset,r) ((volatile uint32 *)(VIRTIO0 + VIRTIO_OFFSET * offset + (r)))

static struct disk {
  // Name of the disk to be used with panic and spinlock
  char *name;

  // a set (not a ring) of DMA descriptors, with which the
  // driver tells the device where to read and write individual
  // disk operations. there are NUM descriptors.
  // most commands consist of a "chain" (a linked list) of a couple of
  // these descriptors.
  struct virtq_desc *desc;

  // a ring in which the driver writes descriptor numbers
  // that the driver would like the device to process.  it only
  // includes the head descriptor of each chain. the ring has
  // NUM elements.
  struct virtq_avail *avail;

  // a ring in which the device writes descriptor numbers that
  // the device has finished processing (just the head of each chain).
  // there are NUM used ring entries.
  struct virtq_used *used;

  // our own book-keeping.
  char free[NUM];  // is a descriptor free?
  uint16 used_idx; // we've looked this far in used[2..NUM].

  // track info about in-flight operations,
  // for use when completion interrupt arrives.
  // indexed by first descriptor index of chain.
  struct {
    struct buf *b;
    char status;
  } info[NUM];

  // disk command headers.
  // one-for-one with descriptors, for convenience.
  struct virtio_blk_req ops[NUM];
  
  struct spinlock vdisk_lock;

  int bw_transfer;
  
} disk[2];

static struct buf* swap_buffer;

void
virtio_disk_init(int id, char * name)
{
  uint32 status = 0;

  initlock(&disk[id].vdisk_lock, name);
  disk[id].name = name;

  if(*R(id, VIRTIO_MMIO_MAGIC_VALUE) != 0x74726976 ||
     *R(id, VIRTIO_MMIO_VERSION) != 2 ||
     *R(id, VIRTIO_MMIO_DEVICE_ID) != 2 ||
     *R(id, VIRTIO_MMIO_VENDOR_ID) != 0x554d4551){
    panic_concat(2, "could not find virtio disk: ", name);
  }
  
  // reset device
  *R(id, VIRTIO_MMIO_STATUS) = status;

  // set ACKNOWLEDGE status bit
  status |= VIRTIO_CONFIG_S_ACKNOWLEDGE;
  *R(id, VIRTIO_MMIO_STATUS) = status;

  // set DRIVER status bit
  status |= VIRTIO_CONFIG_S_DRIVER;
  *R(id, VIRTIO_MMIO_STATUS) = status;

  // negotiate features
  uint64 features = *R(id, VIRTIO_MMIO_DEVICE_FEATURES);
  features &= ~(1 << VIRTIO_BLK_F_RO);
  features &= ~(1 << VIRTIO_BLK_F_SCSI);
  features &= ~(1 << VIRTIO_BLK_F_CONFIG_WCE);
  features &= ~(1 << VIRTIO_BLK_F_MQ);
  features &= ~(1 << VIRTIO_F_ANY_LAYOUT);
  features &= ~(1 << VIRTIO_RING_F_EVENT_IDX);
  features &= ~(1 << VIRTIO_RING_F_INDIRECT_DESC);
  *R(id, VIRTIO_MMIO_DRIVER_FEATURES) = features;

  // tell device that feature negotiation is complete.
  status |= VIRTIO_CONFIG_S_FEATURES_OK;
  *R(id, VIRTIO_MMIO_STATUS) = status;

  // re-read status to ensure FEATURES_OK is set.
  status = *R(id, VIRTIO_MMIO_STATUS);
  if(!(status & VIRTIO_CONFIG_S_FEATURES_OK))
      panic_concat(2, name, ": virtio disk FEATURES_OK unset");

  // initialize queue 0.
  *R(id, VIRTIO_MMIO_QUEUE_SEL) = 0;

  // ensure queue 0 is not in use.
  if(*R(id, VIRTIO_MMIO_QUEUE_READY))
      panic_concat(2, name, ": virtio disk should not be ready");

  // check maximum queue size.
  uint32 max = *R(id, VIRTIO_MMIO_QUEUE_NUM_MAX);
  if(max == 0)
      panic_concat(2, name, ": virtio disk has no queue 0");
  if(max < NUM)
      panic_concat(2, name, ": virtio disk max queue too short");

  // allocate and zero queue memory.
  disk[id].desc = kalloc(0,0,0);
  disk[id].avail = kalloc(0,0,0);
  disk[id].used = kalloc(0,0,0);
  if(!disk[id].desc || !disk[id].avail || !disk[id].used)
      panic_concat(2, name, ": virtio disk kalloc");
  memset(disk[id].desc, 0, PGSIZE);
  memset(disk[id].avail, 0, PGSIZE);
  memset(disk[id].used, 0, PGSIZE);

  // set queue size.
  *R(id, VIRTIO_MMIO_QUEUE_NUM) = NUM;

  // write physical addresses.
  *R(id, VIRTIO_MMIO_QUEUE_DESC_LOW) = (uint64)disk[id].desc;
  *R(id, VIRTIO_MMIO_QUEUE_DESC_HIGH) = (uint64)disk[id].desc >> 32;
  *R(id, VIRTIO_MMIO_DRIVER_DESC_LOW) = (uint64)disk[id].avail;
  *R(id, VIRTIO_MMIO_DRIVER_DESC_HIGH) = (uint64)disk[id].avail >> 32;
  *R(id, VIRTIO_MMIO_DEVICE_DESC_LOW) = (uint64)disk[id].used;
  *R(id, VIRTIO_MMIO_DEVICE_DESC_HIGH) = (uint64)disk[id].used >> 32;

  // queue is ready.
  *R(id, VIRTIO_MMIO_QUEUE_READY) = 0x1;

  // all NUM descriptors start out unused.
  for(int i = 0; i < NUM; i++)
    disk[id].free[i] = 1;

  // tell device we're completely ready.
  status |= VIRTIO_CONFIG_S_DRIVER_OK;
  *R(id, VIRTIO_MMIO_STATUS) = status;

  if (id == VIRTIO1_ID) {
    swap_buffer = kalloc(0,0,0);
    if (!swap_buffer) {
      panic("virtio_disk_init: kalloc of swap_buffer failed");
    }
    memset(swap_buffer, 0, BSIZE);
    initsleeplock(&swap_buffer->lock, "swap_buffer");
  }

  // plic.c and trap.c arrange for interrupts from VIRTIO0_IRQ and VIRTIO1_IRQ.
}

// find a free descriptor, mark it non-free, return its index.
static int
alloc_desc(int id)
{
  for(int i = 0; i < NUM; i++){
    if(disk[id].free[i]){
      disk[id].free[i] = 0;
      return i;
    }
  }
  return -1;
}

// mark a descriptor as free.
static void
free_desc(int id, int i)
{
  if(i >= NUM)
    panic_concat(2, disk[id].name, ": free_desc 1");
  if(disk[id].free[i])
      panic_concat(2, disk[id].name, ": free_desc 2");
  disk[id].desc[i].addr = 0;
  disk[id].desc[i].len = 0;
  disk[id].desc[i].flags = 0;
  disk[id].desc[i].next = 0;
  disk[id].free[i] = 1;
  if (!disk[id].bw_transfer) {
      wakeup(&disk[id].free[0]);
  }
}

// free a chain of descriptors.
static void
free_chain(int id, int i)
{
  while(1){
    int flag = disk[id].desc[i].flags;
    int nxt = disk[id].desc[i].next;
    free_desc(id, i);
    if(flag & VRING_DESC_F_NEXT)
      i = nxt;
    else
      break;
  }
}

// allocate three descriptors (they need not be contiguous).
// disk transfers always use three descriptors.
static int
alloc3_desc(int id, int *idx)
{
  for(int i = 0; i < 3; i++){
    idx[i] = alloc_desc(id);
    if(idx[i] < 0){
      for(int j = 0; j < i; j++)
        free_desc(id, idx[j]);
      return -1;
    }
  }
  return 0;
}

void
virtio_disk_rw(int id, struct buf *b, int write, int busy_wait)
{
  uint64 sector = b->blockno * (BSIZE / 512);

  acquire(&disk[id].vdisk_lock);
  disk[id].bw_transfer = busy_wait;

  // the spec's Section 5.2 says that legacy block operations use
  // three descriptors: one for type/reserved/sector, one for the
  // data, one for a 1-byte status result.

  // allocate the three descriptors.
  int idx[3];
  while(1){
    if(alloc3_desc(id, idx) == 0) {
      break;
    }
    if (!busy_wait) {
        sleep(&disk[id].free[0], &disk[id].vdisk_lock);
    } else {
        release(&disk[id].vdisk_lock);
        intr_on();
        while(alloc3_desc(id, idx) != 0);
        intr_off();
        acquire(&disk[id].vdisk_lock);
    }
  }

  // format the three descriptors.
  // qemu's virtio-blk.c reads them.

  struct virtio_blk_req *buf0 = &disk[id].ops[idx[0]];

  if(write)
    buf0->type = VIRTIO_BLK_T_OUT; // write the disk
  else
    buf0->type = VIRTIO_BLK_T_IN; // read the disk
  buf0->reserved = 0;
  buf0->sector = sector;

  disk[id].desc[idx[0]].addr = (uint64) buf0;
  disk[id].desc[idx[0]].len = sizeof(struct virtio_blk_req);
  disk[id].desc[idx[0]].flags = VRING_DESC_F_NEXT;
  disk[id].desc[idx[0]].next = idx[1];

  disk[id].desc[idx[1]].addr = (uint64) b->data;
  disk[id].desc[idx[1]].len = BSIZE;
  if(write)
    disk[id].desc[idx[1]].flags = 0; // device reads b->data
  else
    disk[id].desc[idx[1]].flags = VRING_DESC_F_WRITE; // device writes b->data
  disk[id].desc[idx[1]].flags |= VRING_DESC_F_NEXT;
  disk[id].desc[idx[1]].next = idx[2];

  disk[id].info[idx[0]].status = 0xff; // device writes 0 on success
  disk[id].desc[idx[2]].addr = (uint64) &disk[id].info[idx[0]].status;
  disk[id].desc[idx[2]].len = 1;
  disk[id].desc[idx[2]].flags = VRING_DESC_F_WRITE; // device writes the status
  disk[id].desc[idx[2]].next = 0;

  // record struct buf for virtio_disk_intr().
  b->disk = 1;
  disk[id].info[idx[0]].b = b;

  // tell the device the first index in our chain of descriptors.
  disk[id].avail->ring[disk[id].avail->idx % NUM] = idx[0];

  __sync_synchronize();

  // tell the device another avail ring entry is available.
  disk[id].avail->idx += 1; // not % NUM ...

  __sync_synchronize();

  *R(id, VIRTIO_MMIO_QUEUE_NOTIFY) = 0; // value is queue number

  // Wait for virtio_disk_intr() to say request has finished.
  while(b->disk == 1) {
    if (!busy_wait) {
        sleep(b, &disk[id].vdisk_lock);
    } else {
        release(&disk[id].vdisk_lock);
        intr_on();
        while(b->disk == 1);
        intr_off();
        acquire(&disk[id].vdisk_lock);
    }
  }

  disk[id].info[idx[0]].b = 0;
  free_chain(id, idx[0]);

  release(&disk[id].vdisk_lock);
}

void write_block(int blockno, uchar data[BSIZE], int busy_wait) {
    struct buf *b = swap_buffer;
    b->blockno = blockno;
    memmove(b->data, data, BSIZE);

    virtio_disk_rw(VIRTIO1_ID, b, 1, busy_wait);
}

void read_block(int blockno, uchar data[BSIZE], int busy_wait) {
    struct buf *b = swap_buffer;
    b->blockno = blockno;

    virtio_disk_rw(VIRTIO1_ID, b, 0, busy_wait);
    memmove(data, b->data, BSIZE);
}

void
virtio_disk_intr(int id)
{
  acquire(&disk[id].vdisk_lock);

  // the device won't raise another interrupt until we tell it
  // we've seen this interrupt, which the following line does.
  // this may race with the device writing new entries to
  // the "used" ring, in which case we may process the new
  // completion entries in this interrupt, and have nothing to do
  // in the next interrupt, which is harmless.
  *R(id, VIRTIO_MMIO_INTERRUPT_ACK) = *R(id, VIRTIO_MMIO_INTERRUPT_STATUS) & 0x3;

  __sync_synchronize();

  // the device increments disk.used->idx when it
  // adds an entry to the used ring.

  while(disk[id].used_idx != disk[id].used->idx){
    __sync_synchronize();
    int idx = disk[id].used->ring[disk[id].used_idx % NUM].id;

    if(disk[id].info[idx].status != 0)
      panic_concat(2, disk[id].name, ": virtio_disk_intr status");

    struct buf *b = disk[id].info[idx].b;
    b->disk = 0;   // disk is done with buf
    if (!disk[id].bw_transfer) {
        wakeup(b);
    }

    disk[id].used_idx += 1;
  }

  release(&disk[id].vdisk_lock);
}


// right now whole bit vector can fit inside one page
// but, if the size of swap disk was to increase, id have to store the inside of multiple pages
// that is why array of pointers to pages with bits is stored inside of swap disk structure

static struct bit_vector {
    struct spinlock bitvector_lock;
    uint8* bits[BITVECSIZEPAGES];
} bit_vector;

// creates a structure used to track pages on swap disk
// for now, the structure is a bit vector, where 1 indicates that the corresponding block on swap disk is free
void
init_swap_disk() {

    initlock(&bit_vector.bitvector_lock, "bit_vector");

    // allocate pages for bit vector, and fill them with 1's
    for ( int i = 0; i  < BITVECSIZEPAGES ; i++) {
        bit_vector.bits[i] = kalloc(0,0,0);
        memset(bit_vector.bits[i], -1, PGSIZE);
    }

}

uint32
allocate_swap_block() {

    acquire(&bit_vector.bitvector_lock);


    for ( int i = 0; i < (BITVECSIZEPAGES - 1); i++) {  // warning because currently whole bit vector can fit inside 1 page
        for ( uint32 j = 0; j < PGSIZE; j++) {
            if ( bit_vector.bits[i][j] == 0 ) continue;

            // look for the set bit inside of this byte
            for (int k = 0; k < 8; k++) {
                if ((bit_vector.bits[i][j] & (1 << k)) != 0) {
                    // set the bit to 0, and return the block number
                    bit_vector.bits[i][j] |= (1 << k);
                    release(&bit_vector.bitvector_lock);
                    return (uint32)i * PGSIZE * 8 + j * 8 + k;
                }
            }
        }
    }

    //check for the last page, separately, because it might not store bits of bit vector in its whole capacity
    for ( uint32 i = 0; i < BITVECSIZEBYTES % PGSIZE; i++) {
        if ( bit_vector.bits[BITVECSIZEPAGES - 1][i] == 0 ) continue;

        // look for the set bit inside of this byte
        for (int j = 0; j < 8; j++) {
            if ((bit_vector.bits[BITVECSIZEPAGES - 1][i] & (1 << j)) != 0) {
                // set the bit to 0, and return the block number
                bit_vector.bits[BITVECSIZEPAGES - 1][i] &= ~(1 << j);
                release(&bit_vector.bitvector_lock);
                return (uint32)(BITVECSIZEPAGES - 1) * PGSIZE * 8 + i * 8 + j;
            }
        }
    }

    // Error, no free space on swap disk

    release(&bit_vector.bitvector_lock);

    return -1;
}

void
free_swap_block(uint32 blockNo) {

    acquire(&bit_vector.bitvector_lock);

    uint32 page = blockNo / (PGSIZE * 8);
    uint32 byte = (blockNo / 8 ) % PGSIZE;
    uint32 bit = blockNo % 8;

    bit_vector.bits[page][byte] |= (1 << bit);

    release(&bit_vector.bitvector_lock);

}

int swap_flag = 0;

// Writes a page from physical memory on to the swap disk
// pa - physical address of the page to be written on to the swap disk
// returns the number of the block on swap disk, in which the page was written
uint32
write_on_swap(uint64 pa) {
    // allocate a block on swap disk
    // lock?
    // write the page which in 4 parts since disk block size is equal to PGSIZE/4
    // unlock?


    uint32 blockNo = allocate_swap_block();
    if ( blockNo == -1 ) {
        // no free space left, exit the program
        //panic("swap disk full");
        return -1;
    }

    swap_flag = 1;

    // lock
    uchar* addr = (uchar*)pa;
    int blk = blockNo * 4; // 4 swap disk blocks are needed to store one page
    for ( int i = 0; i < 4; i++ ) {

        write_block(blk, addr, 1);

        blk++;
        addr += PGSIZE / 4;
        // TODO - document how swap disk blocks are being tracked
    }

    swap_flag = 0;

    // quarters of a page are stored on swap disk in little endian
    // swap disk block with lower number stores lower part of the page

    // unlock
    return blockNo;
}

// Reads a page from swap disk and writes it in to the memory
// blockNo - the number swap disk block that contains the page to be read
// pa - starting address of the memory block in which the read page will be written
void
read_from_swap(uint32 blockNo, uint64 pa) {
    // lock?
    // read page from disk while writing it into memory
    // unlock?
    // free the swap disk block

    swap_flag = 1;

    uchar* addr = (uchar*)pa;
    int blk = blockNo * 4;
    for ( int i = 0; i < 4; i++ ) {

        read_block(blk, addr, 1);

        blk++;
        addr += PGSIZE / 4;
    }

    swap_flag = 0;

    free_swap_block(blockNo);


}


