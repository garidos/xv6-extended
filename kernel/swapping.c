// functions related to page swapping and thrashing
// information on every page in memory

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"
#include "proc.h"

#define MAXTABLESIZE ((((PHYSTOP - KERNBASE) / PGSIZE) * 8 + PGSIZE + 1) / PGSIZE)
#define NUMOFENTRIES (1 << 9)

// find the number of page in page_info which stores entry for page with page_number
#define PN2PTP(page_num) (page_num / (PGSIZE / 8))
// find the entry inside of page_info page that corresponds to page_number
#define PN2PTPE(page_num) (page_num % (PGSIZE / 8))

// #define PA2PTE(pa) ((((uint64)pa) >> 12) << 10)

extern struct proc proc[NPROC];

extern char end[]; // first address after kernel.
// defined by kernel.ld.

extern int ws_threshold;

// structure used for storing all the necessary information for page swapping
// it has bound pages stored in page_info array which store information on page_count pages
struct page_info {
    struct spinlock lock;
    int bound;
    int page_count;
    uint64* page_info[MAXTABLESIZE];
} page_table;

/*
 *  pid      page    ref_cnt   SWP
 * 63..38   37..9     8..1      0
 * SWP - bit that indicates if the page is swappable : 1 - can be swapped; 0 - can not be swapped; initial value = 1;
 * ref_cnt - reference counter used by page swapping algorithm;
 * page - number of page in virtual memory
 * pid - process id
 * page + pid - unique identifier of page stored in this memory block (tag)
 */


// initiates a page_table by calculating the number of pages that can fit in memory and allocating pages for page_info array
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
// if swappable == 0, pid and va are not used
void
page_set(int page_num, int swappable, int pid, uint64 va) {

    int page = PN2PTP(page_num);
    int entry = PN2PTPE(page_num);

    acquire(&page_table.lock);

    // since this is done when the page is allocated, that means we have to reset all the information on it in the page_info table
    page_table.page_info[page][entry] = 0;


    if ( swappable == 0 || pid == 1) {  // pages belonging to the first process should never be swapped
        // clear the SWP bit - page is not swappable
        page_table.page_info[page][entry] &= ~((uint64)1);
        // if not swappable pid and page fields are never used, so we don't set them
    } else {
        // ref counter set to 0x80, not needed since it will be updated before context switch, but just in case
        page_table.page_info[page][entry] |= ((uint64) 1 << 8);
        // set the SWP bit - page is swappable
        page_table.page_info[page][entry] |= (uint64)1;
        uint64 pageid = (va >> 12) & ~(~(uint64)0 << 27);   // extract 27 page bits from virtual address
        page_table.page_info[page][entry] |= pageid << 9;
        page_table.page_info[page][entry] |= (uint64)pid << 38;
    }

    release(&page_table.lock);

}

// called by uvmpcopy to check if the given page is swappable or not
// pa - physical address of page to be checked
// returns 1 if swappable, 0 otherwise
int
check_swappable(void* pa) {
    int page_num = ((char*)pa - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
    int page = PN2PTP(page_num);
    int entry = PN2PTPE(page_num);
    int ret = 0;

    acquire(&page_table.lock);

    if ( page_table.page_info[page][entry] & (uint64)1 ) ret = 1;

    release(&page_table.lock);

    return ret;
}

// called by kfree
// increments the working set threshold if the page being freed was not swappable
// page_num - number of page (in memory) that's being freed
void
update_cnt(int page_num) {

    int page = PN2PTP(page_num);
    int entry = PN2PTPE(page_num);

    acquire(&page_table.lock);

    if ( (page_table.page_info[page][entry] & (uint64)1) == 0 ) {   // if this block belonged to non-swappable page, increment the counter
        ws_threshold++;
    }

    release(&page_table.lock);
}

// adds a reference ( sets the highest bit ) to the reference counter belonging to the page_num page
// page_num - page number in memory
void
page_add_ref(int page_num) {
    int page = PN2PTP(page_num);
    int entry = PN2PTPE(page_num);

    acquire(&page_table.lock);

    if ( page_table.page_info[page][entry] & (uint64)1 ) {  //only if swappable
        page_table.page_info[page][entry] |= ((uint64) 1 << 8);   // set the highest bit in ref_cnt
    }

    release(&page_table.lock);
}

// called on every timer tick, shifts every reference counter in page_table,
// updates reference counters of pages belonging to the current process and updates its working set
void
update_ref_cnts(void) {

    acquire(&page_table.lock);

    // goes through every entry in page_table and shifts reference counters by 1
    for ( int i = 0; i < page_table.bound; i++ ) {
        int bound = (i == (page_table.bound - 1)?page_table.page_count % (PGSIZE / 8):PGSIZE / 8);
        for ( int j = 0; j < bound; j++ ) {
            if ( (page_table.page_info[i][j] & (uint64)1) == 0 ) continue;    // either non allocated or non-swappable page
            uint64 entry = page_table.page_info[i][j];
            uint8 ref_cnt = entry >> 1;
            ref_cnt = ref_cnt >> 1; // shift the counter to the right
            entry &= 0xfffffffffffffe01;   // ref_cnt = 0
            entry |= ((uint64)ref_cnt << 1);
            page_table.page_info[i][j] = entry;   // ref_cnt = updated ref_cnt
        }
    }

    release(&page_table.lock);


    // using the pagetable field from running process struct update its reference counters and calculate the working set

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
                if ( pt2[j] & PTE_V && pt2[j] & PTE_A && pt2[j] & PTE_U ) {
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

    acquire(&page_table.lock);

    uint8 min = 0xff;
    int page_num = -1;

    for ( int i = 0; i < page_table.bound; i++ ) {
        int bound = (i == (page_table.bound - 1)?page_table.page_count % (PGSIZE / 8):PGSIZE / 8);
        for ( int j = 0; j < bound; j++ ) {
            if ( (page_table.page_info[i][j] & (uint64)1) == 0 ) continue;    // either non allocated or non-swappable page
            uint8 ref_cnt = page_table.page_info[i][j] >> 1;
            if ( ref_cnt < min) {
                min = ref_cnt;
                page_num = i*(PGSIZE / 8) + j;
            }
        }
    }

    release(&page_table.lock);

    return page_num;
}

// function that tries to free memory by swapping pages to disk
// called by kalloc when memory is full
// returns 0 on success, 1 on failure
int
swap(void) {
    // find a victim page; write a victim page to swap disk; update its pte; free the page

    int victim_page = find_victim();
    if ( victim_page == -1 ) return 1;

    int page = PN2PTP(victim_page);
    int entry = PN2PTPE(victim_page);

    acquire(&page_table.lock);

    uint32 victim_pid = page_table.page_info[page][entry] >> 38;
    uint64 victim_va = ((page_table.page_info[page][entry] >> 9) & ~(~(uint64)0 << 27 )) << 12;

    release(&page_table.lock);

    // set the page as non-swappable, so it can not be chosen as swap page again while its being written on to swap disk
    page_set(victim_page, 0,0,0);


    struct proc* p;

    for(p = proc; p < &proc[NPROC]; p++) {
        if ( p->pid == victim_pid ) break;
    }

    pte_t * victim_pte = walk(p->pagetable, victim_va, 0);

    /*
     * information on if the page is on swapped disk and if it is, on which block of swap disk will be stored in that pages pte
     * first RSW bit will be used as a flag to indicate if the page is on swap disk ( we'll call it S bit )
     * part of the pte that stores the number of block in physical memory when page is in memory will now be used to store block number on swap disk
     * also, V bit has to be set to 0
     */

    char* victim_addr = (char*)PGROUNDUP((uint64)end) + PGSIZE * victim_page;

    uint32 blockNo = write_on_swap((uint64)victim_addr);
    if ( blockNo == -1 ) {
        page_set(victim_page,1,victim_pid,victim_va);   // return the old info just in case
        return -1;
    }

    // clear V bit
    *victim_pte &= ~PTE_V;
    // set S bit
    *victim_pte |= PTE_S;

    // clear the previous value stored in rest of the pte
    *victim_pte &= ~(~(uint64)0 << 10);
    // store the number of block on swap disk
    *victim_pte |= (uint64)blockNo << 10;

    // return the old info so that the counter will be updated accordingly
    page_set(victim_page,1,victim_pid,victim_va);
    // free victim page
    kfree(victim_addr);

    return 0;
}


// tries to load a page from swap disk
// returns 0 on success, negative value otherwise
int
load_page(uint64 va, pagetable_t pg_table, int pid) {

    // get the pte corresponding to the virtual address that caused PF; check if the page was swapped; if not, return 1
    // get the number of block on swap disk that stores this page; allocate the block in memory to store the page
    // load the page from disk in to the allocated block; update pte ( set valid, reset D bit, insert physical block number  )

    pte_t * pte = walk(pg_table, va, 0);
    if ( pte == 0 ) return -1;

    if ( (*pte & PTE_S) == 0 ) return -1;

    uint32 blockNo = *pte >> 10;
    // alloc as non-swappable page, to make sure that the page doesn't get swapped while its being read from swap disk
    char* free_page = (char*)kalloc(0, 0, 0);
    if ( free_page == 0 ) return -1;

    read_from_swap(blockNo, (uint64)free_page);

    // reading is finished, so we can set the page to swappable and add the tag in to page_info table entry
    int page_num = ((char*)free_page - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
    page_set(page_num, 1, pid, va);

    // set V bit
    *pte |= PTE_V;
    // reset S bit
    *pte &= ~PTE_S;
    // add physical block number
    // clear old value first
    *pte &= ~(~(uint64)0 << 10);
    *pte |= PA2PTE((uint64)free_page);

    return 0;
}

// checks if thrashing is occurring based on the sum of working sets of every active process and working set threshold
// if it has, tries to resolve it by suspending the process with biggest working set
// this function should be called periodically ( period should be greater than one used to change context )
void
thrashing_check(void) {

    // go through every active process and calculate sum of their working sets, while noting the process with the biggest working set
    // check if the sum exceeds allowed amount; if it does, take the process with the biggest working set and suspend it
    // if it does not, go through all the suspended processes and check if adding their working set to the current sum would exceed the allowed amount
    // if not, bring the process back from suspension by marking it as runnable ( use the process lock )

    int sum = 0;
    struct proc* max_ws_proc = 0;
    int proc_cnt = 0;

    for(struct proc* p = proc; p < &proc[NPROC]; p++) {
        acquire(&p->lock);  // lock is needed for process' state
        if ( p->state == RUNNABLE || p->state == RUNNING ) {
            sum += p->working_set;
            if ( max_ws_proc == 0 || p->working_set > max_ws_proc->working_set ) max_ws_proc = p;   // lock is not needed for process' working_set
            proc_cnt++;
        }
        release(&p->lock);
    }

    if ( max_ws_proc == 0 || proc_cnt == 1 ) return;    // if there's only one active process there's no point in suspending it


    if ( sum > ws_threshold ) {

        // suspend process with the biggest working set

        //printf("Process %d suspended!\n", max_ws_proc->pid);

        acquire(&max_ws_proc->lock);
        // ws stays unchanged, so later on, we have a way to check if the process can be brought back
        max_ws_proc->state = SUSPENDED; // yield will be called after this function anyway, so the state it was in doesn't matter
        // since the process is not RUNNING or RUNNABLE it won't be picked by scheduler, hence it's suspended

        release(&max_ws_proc->lock);

    } else {

        // try to wake up suspended processes

        for(struct proc* p = proc; p < &proc[NPROC]; p++) {
            acquire(&p->lock);
            if ( p->state == SUSPENDED && sum + p->working_set <= ws_threshold ) {
                p->state = RUNNABLE;    // so it can get scheduled
                // its working set won't be reset
                sum += p->working_set;
                //printf("Process %d woken up!\n", p->pid);
            }
            release(&p->lock);
        }

    }

}