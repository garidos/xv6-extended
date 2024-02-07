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


void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

// threshold which changes dynamically, used by thrashing prevention algorithm
// represents the amount of pages that are free or allocated as swappable ( total amount pages in memory minus number of currently allocated non swappable pages )
int ws_threshold;

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;


void
kinit()
{
  initlock(&kmem.lock, "kmem");
  freerange(end, (void*)PHYSTOP);
  init_page_table();
}

// help_kalloc and help_kfree are used instead of kalloc and kfree before page_table is initialized
void *
help_kalloc(void)
{
    struct run *r;

    acquire(&kmem.lock);
    r = kmem.freelist;

    if(r) {
        kmem.freelist = r->next;
        ws_threshold--;
    }

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
// kalloc uses this parameter to set the SWP bit in that page's entry in page_info table
void* kalloc(int swappable, int pid, uint64 va) {
    struct run *r;

    acquire(&kmem.lock);

    // while loop used to avoid race condition, since allocation is allowed while swapping
    while ( kmem.freelist == 0 ) {
        release(&kmem.lock);
        int status = swap();     // try to free a single page by swapping it to swap disk
        if ( status < 0 ) {
            r = 0;
            return r;   // no space left
        }
        acquire(&kmem.lock);
    }

    r = kmem.freelist;

    if(r) {
        kmem.freelist = r->next;
        if ( swappable == 0 ) ws_threshold--;
    }

    release(&kmem.lock);

    if(r)
        memset((char *) r, 5, PGSIZE); // fill with junk

    if (r) {
        // calculate the page number and set its info
        int page_num = ((char*)r - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
        page_set(page_num, swappable, pid, va);
    }

    return (void*)r;
}

void
freerange(void *pa_start, void *pa_end)
{
  char *p;
  ws_threshold = 0;
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
  int page_num = ((char*)pa - (char*)PGROUNDUP((uint64)end)) / PGSIZE;

  // if the page was non-swappable increase the counter(which represents thrashing threshold)
  update_cnt(page_num);
  // it doesn't matter what I pass as swappable argument, I just want it to reset the entry for this page
  page_set(page_num, 0, 0, 0);
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
    ws_threshold++;
    release(&kmem.lock);
}