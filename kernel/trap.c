#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

struct spinlock tickslock;
uint ticks;

extern char trampoline[], uservec[], userret[];

extern struct proc proc[NPROC];

extern int swap_flag;

// in kernelvec.S, calls kerneltrap().
void kernelvec();

extern int devintr();

int timer_tick;
int thrashing_tick;

extern char end[];

extern int ws_threshold;

void
trapinit(void)
{
  initlock(&tickslock, "time");
  timer_tick = 0;
  thrashing_tick = 0;
}

// set up to take exceptions and traps while in the kernel.
void
trapinithart(void)
{
  w_stvec((uint64)kernelvec);
}

//
// handle an interrupt, exception, or system call from user space.
// called from trampoline.S
//
void
usertrap(void)
{
  int which_dev = 0;

  if((r_sstatus() & SSTATUS_SPP) != 0)
    panic("usertrap: not from user mode");

  // send interrupts and exceptions to kerneltrap(),
  // since we're now in the kernel.
  w_stvec((uint64)kernelvec);

  struct proc *p = myproc();
  
  // save user program counter.
  p->trapframe->epc = r_sepc();

  // here, PF is raised only when a page was swapped and is not present in memory, so, the handling of the PF will consist
  // of finding out which page was it that caused the PF, finding it on swap disk, and loading it back in memory
  // if memory is full, before loading the page back in, we will have to make space for it, and swap some other page on to swap disk
  // this last part should be handled by kalloc?

  if(r_scause() == 8) {
      // system call

      if (killed(p))
          exit(-1);

      // sepc points to the ecall instruction,
      // but we want to return to the next instruction.
      p->trapframe->epc += 4;

      // an interrupt will change sepc, scause, and sstatus,
      // so enable only now that we're done with those registers.
      intr_on();

      syscall();
  } else if (r_scause() == 0xf || r_scause() == 0xc || r_scause() == 0xd) { // page faults
      // It's not important which type of PF it is?
      // stval holds the virtual address that caused PF
      if ( handle_page_fault() ) {
          /*
          printf("usertrap(): unresolved page fault scause=%p pid=%d\n", r_scause(), p->pid);
          printf("            sepc=%p stval=%p\n", r_sepc(), r_stval());*/
          setkilled(p);
      }
  } else if((which_dev = devintr()) != 0){
    // ok
  } else {
    printf("usertrap(): unexpected scause %p pid=%d\n", r_scause(), p->pid);
    printf("            sepc=%p stval=%p\n", r_sepc(), r_stval());
    setkilled(p);
  }

  if(killed(p))
    exit(-1);

  // give up the CPU if this is a timer interrupt.
  if(which_dev == 2) { // if(which_dev == 2 && timer_tick++ == 2) {
      // time_tick = 0;
      update_ref_cnts();

      if ( swap_flag == 0 ) thrashing_tick++; // thrashing_tick += 2;
      if ( thrashing_tick == 30 && swap_flag == 0) {
          thrashing_check();
          thrashing_tick = 0;
      }

      yield();
  }

  usertrapret();
}

//
// return to user space
//
void
usertrapret(void)
{
  struct proc *p = myproc();

  // we're about to switch the destination of traps from
  // kerneltrap() to usertrap(), so turn off interrupts until
  // we're back in user space, where usertrap() is correct.
  intr_off();

  // send syscalls, interrupts, and exceptions to uservec in trampoline.S
  uint64 trampoline_uservec = TRAMPOLINE + (uservec - trampoline);
  w_stvec(trampoline_uservec);

  // set up trapframe values that uservec will need when
  // the process next traps into the kernel.
  p->trapframe->kernel_satp = r_satp();         // kernel page table
  p->trapframe->kernel_sp = p->kstack + PGSIZE; // process's kernel stack
  p->trapframe->kernel_trap = (uint64)usertrap;
  p->trapframe->kernel_hartid = r_tp();         // hartid for cpuid()

  // set up the registers that trampoline.S's sret will use
  // to get to user space.
  
  // set S Previous Privilege mode to User.
  unsigned long x = r_sstatus();
  x &= ~SSTATUS_SPP; // clear SPP to 0 for user mode
  x |= SSTATUS_SPIE; // enable interrupts in user mode
  w_sstatus(x);

  // set S Exception Program Counter to the saved user pc.
  w_sepc(p->trapframe->epc);

  // tell trampoline.S the user page table to switch to.
  uint64 satp = MAKE_SATP(p->pagetable);

  // jump to userret in trampoline.S at the top of memory, which 
  // switches to the user page table, restores user registers,
  // and switches to user mode with sret.
  uint64 trampoline_userret = TRAMPOLINE + (userret - trampoline);
  ((void (*)(uint64))trampoline_userret)(satp);
}

// interrupts and exceptions from kernel code go here via kernelvec,
// on whatever the current kernel stack is.
void 
kerneltrap()
{
  int which_dev = 0;
  uint64 sepc = r_sepc();
  uint64 sstatus = r_sstatus();
  uint64 scause = r_scause();
  
  if((sstatus & SSTATUS_SPP) == 0)
    panic("kerneltrap: not from supervisor mode");
  if(intr_get() != 0)
    panic("kerneltrap: interrupts enabled");

  if((which_dev = devintr()) == 0){
      // PF in kernel should never happen
    printf("scause %p\n", scause);
    printf("sepc=%p stval=%p\n", r_sepc(), r_stval());
    panic("kerneltrap");
  }

  // give up the CPU if this is a timer interrupt.
  // if(which_dev == 2 && myproc() != 0 && myproc()->state == RUNNING && timer_tick++ == 2)
  if(which_dev == 2 && myproc() != 0 && myproc()->state == RUNNING ) {
      // timer_tick = 0;
      update_ref_cnts();

      if ( swap_flag == 0 ) thrashing_tick++;   // thrashing_tick += 2;
      if ( thrashing_tick == 30 && swap_flag == 0) {
          thrashing_check();
          thrashing_tick = 0;
      }

      yield();
  }

  // the yield() may have caused some traps to occur,
  // so restore trap registers for use by kernelvec.S's sepc instruction.
  w_sepc(sepc);
  w_sstatus(sstatus);
}

void
clockintr()
{
  acquire(&tickslock);
  ticks++;
  wakeup(&ticks);
  release(&tickslock);
}

// check if it's an external interrupt or software interrupt,
// and handle it.
// returns 2 if timer interrupt,
// 1 if other device,
// 0 if not recognized.
int
devintr()
{
  uint64 scause = r_scause();

  if((scause & 0x8000000000000000L) &&
     (scause & 0xff) == 9){
    // this is a supervisor external interrupt, via PLIC.

    // irq indicates which device interrupted.
    int irq = plic_claim();

    if(irq == UART0_IRQ){
      uartintr();
    } else if(irq == VIRTIO0_IRQ){
      virtio_disk_intr(VIRTIO0_ID);
    } else if(irq == VIRTIO1_IRQ){
      virtio_disk_intr(VIRTIO1_ID);
    } else if(irq){
      printf("unexpected interrupt irq=%d\n", irq);
    }

    // the PLIC allows each device to raise at most one
    // interrupt at a time; tell the PLIC the device is
    // now allowed to interrupt again.
    if(irq)
      plic_complete(irq);

    return 1;
  } else if(scause == 0x8000000000000001L){
    // software interrupt from a machine-mode timer interrupt,
    // forwarded by timervec in kernelvec.S.

    if(cpuid() == 0){
      if ( swap_flag == 0 )clockintr();
    }
    
    // acknowledge the software interrupt by clearing
    // the SSIP bit in sip.
    w_sip(r_sip() & ~2);

    return 2;
  } else {
    return 0;
  }
}

// returns 0 if PF was resolved, 1 otherwise
int
handle_page_fault() {

    uint64 va = r_stval();
    struct proc* p = myproc();

    return load_page(va, p->pagetable, p->pid);
}

int
load_page(uint64 va, pagetable_t pg_table, int pid) {

    // get the pte corresponding to the virtual address that caused PF
    // check if the page was swapped
    // if not, return 1
    // get the number of block on swap disk that stores this page
    // allocate the block in memory to store the page
    // load the page from disk in to the allocated block
    // update pte ( set valid, reset D bit, insert physical block number  )

    pte_t * pte = walk(pg_table, va, 0);
    if ( pte == 0 ) return 1;

    if ( (*pte & PTE_D) == 0 ) return 1;

    uint32 blockNo = *pte >> 10;
    // alloc as non-swappable page, to make sure that the page doesn't get swapped while its being read from swap disk
    char* free_page = (char*)kalloc(0, 0, 0);
    if ( free_page == 0 ) return 1;

    read_from_swap(blockNo, (uint64)free_page);

    // reading is finished, so we can set the page to swappable and add the tag in to page_info table entry
    int page_num = ((char*)free_page - (char*)PGROUNDUP((uint64)end)) / PGSIZE;
    page_set(page_num, 1, pid, va);

    // set V bit
    *pte |= PTE_V;
    // reset D bit
    *pte &= ~PTE_D;
    // add physical block number
    // clear old value first
    *pte &= ~(~(uint64)0 << 10);
    *pte |= PA2PTE((uint64)free_page);

    return 0;
}


// this function should be called periodically ( period should be greater than one used to change context )
void
thrashing_check(void) {

    // go through every active process and calculate sum of their working sets, while noting the process with the biggest working set
    // check if the sum exceeds allowed amount
    // if it does, take the process with the biggest working set and suspend it
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