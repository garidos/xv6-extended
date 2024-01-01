// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"
#include "proc.h"

#define MAXTABLESIZE ((((PHYSTOP - KERNBASE) / PGSIZE) * 4 + PGSIZE + 1) / PGSIZE)

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;

// structure used for storing all the necessary information for page swapping
// it has bound pages stored in page_info array which store information on page_count pages
struct page_info {
    // TODO - use lock
    struct spinlock lock;
    int bound;
    int page_count;
    uint32* page_info[MAXTABLESIZE];
} page_table;

/*
 * blk_num   ref_cnt      OD     S
 * 31..10     9..2        1      0
 * S - bit that indicates if the page is swappable : 1 - can be swapped; 0 - can not be swapped; initial value = 1;
 * OD - bit that indicates if the page is currently on disk : 1 - page is on swap disk; 0 - page is in memory; initial value = 0;
 * ref_cnt - reference counter used by page swapping algorithm;
 * blk_num - number of block on swap disk which stores this page, valid only if OD = 1;
 */

void
kinit()
{
  initlock(&kmem.lock, "kmem");
  freerange(end, (void*)PHYSTOP);
  init_page_table();
}

void
init_page_table(void) {

    initlock(&page_table.lock, "page_table");

    page_table.page_count = ((char*)PHYSTOP - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
    page_table.bound = ((page_table.page_count * 4) + PGSIZE - 1 ) / PGSIZE;
    for ( int i = 0; i < page_table.bound; i++ ) {
        page_table.page_info[i] = (uint32*)help_kalloc();
        // fill with 0's
        memset((char *) page_table.page_info[i], 0, PGSIZE);
    }
    // since freerange() initiates free space by calling free() starting from start_adr(which is end), and in free, page being freed
    // is added to the front of the list of free pages, that means that first pages to be allocated will be the ones with the highest address
    // so after we've taken pages we need for the page_info table, we have to mark them in that same table

    // now we have to mark the pages used by page_info table as non-swappable
    // this will be done by the kalloc in the future, but since the table didn't exist when we allocated pages needed
    // we have to do it by ourselves
    int pg = page_table.page_count - 1;
    for ( int i = 0; i < page_table.bound; i++ ) {
        page_set_s(pg, 0);
        pg--;
    }
}

void
page_set_s(int page_num, int swappable) {

    int page = page_num / (PGSIZE / 4); // find the page of page_info table which stores the entry for this page
    int entry = page_num % (PGSIZE / 4);    // find the entry in that page

    // since this is done when the page is allocated, that means we have to reset all the information on it in the page_info table
    page_table.page_info[page][entry] = 0;
    // this will set the OD bit to 0
    // TODO - maybe the ref counter should be set to 0x80 when the page is first allocated

    if ( swappable == 0 ) {
        // clear the S bit - page is not swappable
        page_table.page_info[page][entry] &= ~((uint32)1);
    } else {
        // set the S bit - page is swappable
        page_table.page_info[page][entry] |= (uint32)1;
    }
}

void
page_add_ref(int page_num) {
    int page = page_num / (PGSIZE / 4); // find the page of page_info table which stores the entry for this page
    int entry = page_num % (PGSIZE / 4);    // find the entry in that page

    if ( page_table.page_info[page][entry] & (uint32)1 ) {  //only if swappable
        page_table.page_info[page][entry] |= ((uint32) 1 << 9);   // set the highest bit in ref_cnt
    }
}

void *
help_kalloc(void)
{
    struct run *r;

    acquire(&kmem.lock);
    r = kmem.freelist;

    if(r)
        kmem.freelist = r->next;
    release(&kmem.lock);

    if(r)
        memset((char *) r, 5, PGSIZE); // fill with junk

    return (void*)r;
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.

// parameter swappable indicates if page is swappable, or not, in which case it has to be in memory at all times
// kernel pages and some additional pages used by the system are not swappable
// kalloc uses this parameter to set the S bit in that page's entry in page_info table
void* kalloc(int swappable) {
    struct run *r;

    acquire(&kmem.lock);

    if ( kmem.freelist == 0 ) {
        // TODO - swap
    }

    r = kmem.freelist;

    if(r)
        kmem.freelist = r->next;
    release(&kmem.lock);

    if(r)
        memset((char *) r, 5, PGSIZE); // fill with junk

    if (r) {
        // calculate the page number
        int page_num = ((char*)r - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
        page_set_s(page_num, swappable);
    }

    return (void*)r;
}

void
freerange(void *pa_start, void *pa_end)
{
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start);
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE)
    help_kfree(p);
}

// Free the page of physical memory pointed at by pa,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(void *pa)
{
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  // reset the info on this page
  // TODO - dont do this on pages that are freed after swapping
  int page_num = ((char*)pa - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
  // it doesn't matter what I pass as swappable argument, i just want it to reset the entry for this page
  page_set_s(page_num, 0);
  // swapping will be done when the memory is full, when every entry in the page_info table is valid,
  // so it doesn't really matter what's stored in the entry when the page is free
  // ref_cnt will still get updated, but that doesn't matter either

  r = (struct run*)pa;

  acquire(&kmem.lock);
  r->next = kmem.freelist;
  kmem.freelist = r;
  release(&kmem.lock);
}

void
help_kfree(void *pa)
{
    struct run *r;

    if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
        panic("kfree");

    // Fill with junk to catch dangling refs.
    memset(pa, 1, PGSIZE);

    r = (struct run*)pa;

    acquire(&kmem.lock);
    r->next = kmem.freelist;
    kmem.freelist = r;
    release(&kmem.lock);
}

int first = 0;

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
/*
void *
kalloc(void)
{
  struct run *r;

  acquire(&kmem.lock);
  r = kmem.freelist;
  // TODO - If r is null, that means that there is no space left, so we have to do page swapping
  if(r)
    kmem.freelist = r->next;
  release(&kmem.lock);

  if(r)
      memset((char *) r, 5, PGSIZE); // fill with junk


  return (void*)r;
}*/


#define NUMOFENTRIES (1 << 9)

void
update_ref_cnts(void) {

    for ( int i = 0; i < page_table.bound - 1; i++ ) {
        for ( int j = 0; j < PGSIZE / 4; j++ ) {
            if ( (page_table.page_info[i][j] & (uint32)1) == 0 ) continue;    // either non allocated or non-swappable page
            uint32 entry = page_table.page_info[i][j];
            uint8 ref_cnt = entry >> 2;
            ref_cnt = ref_cnt >> 1; // shift the counter to the right
            entry &= 0xfffffc03;   // ref_cnt = 0
            entry |= ((uint32)ref_cnt << 2);
            page_table.page_info[i][j] = entry;   // ref_cnt = updated ref_cnt
        }
    }

    int i = page_table.bound - 1;
    for ( int j = 0; j < page_table.page_count % (PGSIZE / 4); j++ ) {
        if ( (page_table.page_info[i][j] & (uint32)1) == 0 ) continue;    // either non allocated or non-swappable page
        uint32 entry = page_table.page_info[i][j];
        uint8 ref_cnt = entry >> 2;
        ref_cnt = ref_cnt >> 1; // shift the counter to the right
        entry &= 0xfffffc03;   // ref_cnt = 0
        entry |= ((uint32)ref_cnt << 2);
        page_table.page_info[i][j] = entry;   // ref_cnt = updated ref_cnt
    }

    // go through the array of processes and find the running one
    // using the pagetable field from that processes struct update its reference counters

    struct proc *p = myproc();

    pagetable_t pt = p->pagetable;

    for ( int e = 0; e < NUMOFENTRIES; e++) {
        if ( !(pt[e] & PTE_V) ) continue;
        pagetable_t pt1 = (pagetable_t) PTE2PA(pt[e]);
        for ( int k = 0; k < NUMOFENTRIES; k++) {
            if ( !(pt1[k] & PTE_V) ) continue;
            pagetable_t pt2 = (pagetable_t) PTE2PA(pt1[k]);
            for ( int j = 0; j < NUMOFENTRIES; j++) {
                if ( pt2[j] & PTE_V && pt2[j] & PTE_A ) {
                    pt2[j] &= ~PTE_A;   // clear Access bit
                    char* pa = (char*)PTE2PA(pt2[j]);
                    int page_num = ((char*)pa - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
                    page_add_ref(page_num);
                }
            }
        }
    }

}

// finds the victim page with lowest ref_cnt
// returns the number of that page
int
find_victim(void) {
    uint8 min = 0xff;
    int page_num = -1;

    for ( int i = 0; i < page_table.bound - 1; i++ ) {
        for ( int j = 0; j < PGSIZE / 4; j++ ) {
            if ( (page_table.page_info[i][j] & (uint32)1) == 0 ) continue;    // either non allocated or non-swappable page
            if ( (page_table.page_info[i][j] & (uint32)2) ) continue; // page is already on disk
            uint8 ref_cnt = page_table.page_info[i][j] >> 2;
            ref_cnt = ref_cnt >> 1; // shift the counter to the right
            if ( ref_cnt < min) {
                min = ref_cnt;
                page_num = i*(PGSIZE / 4) + j;
            }
        }
    }

    int i = page_table.bound - 1;
    for ( int j = 0; j < page_table.page_count % (PGSIZE / 4); j++ ) {
        if ( (page_table.page_info[i][j] & (uint32)1) == 0 ) continue;    // either non allocated or non-swappable page
        if ( (page_table.page_info[i][j] & (uint32)2) ) continue; // page is already on disk
        uint8 ref_cnt = page_table.page_info[i][j] >> 2;
        ref_cnt = ref_cnt >> 1; // shift the counter to the right
        if ( ref_cnt < min) {
            min = ref_cnt;
            page_num = i*(PGSIZE / 4) + j;
        }
    }

    return page_num;
}