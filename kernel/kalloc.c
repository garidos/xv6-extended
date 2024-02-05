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

#define MAXTABLESIZE ((((PHYSTOP - KERNBASE) / PGSIZE) * 8 + PGSIZE + 1) / PGSIZE)

extern struct proc proc[NPROC];

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

// structure used for storing all the necessary information for page swapping
// it has bound pages stored in page_info array which store information on page_count pages
struct page_info {
    // TODO - use lock
    struct spinlock lock;
    int bound;
    int page_count;
    uint64* page_info[MAXTABLESIZE];
} page_table;

/*
 *  pid      page    ref_cnt    S
 * 63..38   37..9     8..1      0
 * S - bit that indicates if the page is swappable : 1 - can be swapped; 0 - can not be swapped; initial value = 1;
 * ref_cnt - reference counter used by page swapping algorithm;
 * page - number of page in virtual memory
 * pid - process id
 * page + pid - unique identifier of page stored in this memory block (tag)
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
    page_table.bound = ((page_table.page_count * 8) + PGSIZE - 1 ) / PGSIZE;
    for ( int i = 0; i < page_table.bound; i++ ) {
        page_table.page_info[i] = (uint64*)help_kalloc();
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
        page_set(pg, 0, 0, 0);
        pg--;
    }
}

// sets the info for page_num page
// swappable - if the page is swappable or not
// pid - pid of the process that this page belongs to
// va - virtual address used to extract the number of the page in processes virtual memory
// if swappable = 0, pid and va are not used
void
page_set(int page_num, int swappable, int pid, uint64 va) {

    int page = page_num / (PGSIZE / 8); // find the page of page_info table which stores the entry for this page
    int entry = page_num % (PGSIZE / 8);    // find the entry in that page

    // since this is done when the page is allocated, that means we have to reset all the information on it in the page_info table
    page_table.page_info[page][entry] = 0;
    // this will set the OD bit to 0



    if ( swappable == 0 || pid == 1) {  // pages belonging to the first process should never be swapped
        // clear the S bit - page is not swappable
        page_table.page_info[page][entry] &= ~((uint64)1);
        // if not swappable pid and page fields are never used, so we don't set them
    } else {
        // ref counter set to 0x80, not needed since it will be updated before context switch, but just in case
        page_table.page_info[page][entry] |= ((uint64) 1 << 8);
        // set the S bit - page is swappable
        page_table.page_info[page][entry] |= (uint64)1;
        uint64 pageid = (va >> 12) & ~(~(uint64)0 << 27);   // extract 27 page bits from virtual address
        page_table.page_info[page][entry] |= pageid << 9;
        page_table.page_info[page][entry] |= (uint64)pid << 38;
    }
}

// called by uvmpcopy to check if the given page is swappable or not
int
check_swappable(void* pa) {
    int page_num = ((char*)pa - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
    int page = page_num / (PGSIZE / 8); // find the page of page_info table which stores the entry for this page
    int entry = page_num % (PGSIZE / 8);    // find the entry in that page

    if ( page_table.page_info[page][entry] & (uint64)1 ) return 1;
    return 0;
}

// called by kfree
// increments the working set threshold if the page being freed was not swappable
void
update_cnt(int page_num) {

    int page = page_num / (PGSIZE / 8); // find the page of page_info table which stores the entry for this page
    int entry = page_num % (PGSIZE / 8);    // find the entry in that page

    if ( (page_table.page_info[page][entry] & (uint64)1) == 0 ) {   // if this block belonged to non-swappable page, increment the counter
        ws_threshold++;
    }

}

void
page_add_ref(int page_num) {
    int page = page_num / (PGSIZE / 8); // find the page of page_info table which stores the entry for this page
    int entry = page_num % (PGSIZE / 8);    // find the entry in that page

    if ( page_table.page_info[page][entry] & (uint64)1 ) {  //only if swappable
        page_table.page_info[page][entry] |= ((uint64) 1 << 8);   // set the highest bit in ref_cnt
    }
}

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
// kalloc uses this parameter to set the S bit in that page's entry in page_info table
void* kalloc(int swappable, int pid, uint64 va) {
    struct run *r;

    acquire(&kmem.lock);

    // while loop used to avoid race condition, since allocation is allowed while swapping
    while ( kmem.freelist == 0 ) {  // TODO - maybe dont allow allocation until swap is done, so i dont have to use while here
        release(&kmem.lock);
        int status = swap();     // try to free a single page by swapping it to swap disk
        if ( status ) {
            r = 0;
            return r;
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
        // calculate the page number
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

int first = 0;


#define NUMOFENTRIES (1 << 9)

void
update_ref_cnts(void) {

    for ( int i = 0; i < page_table.bound - 1; i++ ) {
        for ( int j = 0; j < PGSIZE / 8; j++ ) {
            if ( (page_table.page_info[i][j] & (uint64)1) == 0 ) continue;    // either non allocated or non-swappable page
            uint64 entry = page_table.page_info[i][j];
            uint8 ref_cnt = entry >> 1;
            ref_cnt = ref_cnt >> 1; // shift the counter to the right
            entry &= 0xfffffffffffffe01;   // ref_cnt = 0
            entry |= ((uint64)ref_cnt << 1);
            page_table.page_info[i][j] = entry;   // ref_cnt = updated ref_cnt
        }
    }

    int i = page_table.bound - 1;
    for ( int j = 0; j < page_table.page_count % (PGSIZE / 8); j++ ) {
        if ( (page_table.page_info[i][j] & (uint64)1) == 0 ) continue;    // either non allocated or non-swappable page
        uint64 entry = page_table.page_info[i][j];
        uint8 ref_cnt = entry >> 1;
        ref_cnt = ref_cnt >> 1; // shift the counter to the right
        entry &= 0xfffffffffffffe01;   // ref_cnt = 0
        entry |= ((uint32)ref_cnt << 1);
        page_table.page_info[i][j] = entry;   // ref_cnt = updated ref_cnt
    }

    // go through the array of processes and find the running one
    // using the pagetable field from that processes struct update its reference counters

    struct proc *p = myproc();

    pagetable_t pt = p->pagetable;

    int w_set = 0;

    for ( int e = 0; e < NUMOFENTRIES; e++) {
        if ( !(pt[e] & PTE_V) ) continue;
        pagetable_t pt1 = (pagetable_t) PTE2PA(pt[e]);
        for ( int k = 0; k < NUMOFENTRIES; k++) {
            if ( !(pt1[k] & PTE_V) ) continue;
            pagetable_t pt2 = (pagetable_t) PTE2PA(pt1[k]);
            for ( int j = 0; j < NUMOFENTRIES; j++) {
                if ( pt2[j] & PTE_V && pt2[j] & PTE_A && pt2[j] & PTE_U ) {     // TODO - reminder: I added a check for PTE_U bit, only user pages can be swapped
                    pt2[j] &= ~PTE_A;   // clear Access bit
                    char* pa = (char*)PTE2PA(pt2[j]);
                    int page_num = ((char*)pa - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
                    page_add_ref(page_num);
                    w_set++;
                }
            }
        }
    }

    p->working_set = w_set;

}

// finds the victim page with lowest ref_cnt
// returns the number of that page
int
find_victim(void) {
    uint8 min = 0xff;
    int page_num = -1;

    for ( int i = 0; i < page_table.bound - 1; i++ ) {
        for ( int j = 0; j < PGSIZE / 8; j++ ) {
            if ( (page_table.page_info[i][j] & (uint64)1) == 0 ) continue;    // either non allocated or non-swappable page
            uint8 ref_cnt = page_table.page_info[i][j] >> 1;
            if ( ref_cnt < min) {
                min = ref_cnt;
                page_num = i*(PGSIZE / 8) + j;
            }
        }
    }

    int i = page_table.bound - 1;
    for ( int j = 0; j < page_table.page_count % (PGSIZE / 8); j++ ) {
        if ( (page_table.page_info[i][j] & (uint64)1) == 0 ) continue;    // either non allocated or non-swappable page
        uint8 ref_cnt = page_table.page_info[i][j] >> 1;
        if ( ref_cnt < min) {
            min = ref_cnt;
            page_num = i*(PGSIZE / 8) + j;
        }
    }

    return page_num;
}


int
swap(void) {
    //try to free memory by page swapping

    // find a victim page
    // write a victim page to swap disk
    // update its pte
    // free the page

    int victim_page = find_victim();
    if ( victim_page == -1 ) return 1;


    int page = victim_page / (PGSIZE / 8); // find the page of page_info table which stores the entry for this page
    int entry = victim_page % (PGSIZE / 8);    // find the entry in that page

    uint32 victim_pid = page_table.page_info[page][entry] >> 38;
    uint64 victim_va = ((page_table.page_info[page][entry] >> 9) & ~(~(uint64)0 << 27 )) << 12;

    // set the page as non-swappable, so it can not be chosen as swap page again while its being written on to swap disk
    page_set(victim_page, 0,0,0);


    struct proc* p;

    for(p = proc; p < &proc[NPROC]; p++) {
        if ( p->pid == victim_pid ) break;
    }

    pte_t * victim_pte = walk(p->pagetable, victim_va, 0);

    /*
     * information on if the page is on swapped disk and if it is on which block of swap disk will be stored in that pages pte
     * first RSW bit will be used as a flag to indicate if the page is on swap disk ( we'll call it D bit )
     * part of the pte that usually stores the number of block in physical memory will be used to store block number
     * also, V bit has to be set to 0
     */

    char* victim_addr = (char*)PGROUNDUP((uint64)end) + PGSIZE * victim_page;



    uint32 blockNo = write_on_swap((uint64)victim_addr);
    if ( blockNo == -1 ) {
        page_set(victim_page,1,victim_pid,victim_va);   // return the old info just in case
        return 1;
    }

    // clear V bit
    *victim_pte &= ~PTE_V;
    // set D bit
    *victim_pte |= PTE_D;

    // clear the previous value stored in rest of the pte
    *victim_pte &= ~(~(uint64)0 << 10);
    // store the number of swap disk block
    *victim_pte |= (uint64)blockNo << 10;

    // return the old info so that the counter will be updated accordingly
    page_set(victim_page,1,victim_pid,victim_va);
    // free victim page
    kfree(victim_addr);

    return 0;
}

