#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {


  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);

    //a2
  //release waiting parent

  p->exitcode = _MKWAIT_EXIT(exitcode);
  p->state = 0;
  lock_acquire(p->waitPidLock);
      cv_broadcast(p->waitPidCV, p->waitPidLock);
  lock_release(p->waitPidLock);




  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  //a2
  KASSERT(curproc != NULL);
  *retval = curproc->pid;
  return(0);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */

  if (options != 0) {
    return(EINVAL);
  }

  //a2
  //check pid is valid child
  //prob2
  int flag = 0;
  proc *cd;
  for (unsigned i = 0; i< array_num(&curproc->children); i++) {
    cd = array_get(&curproc->children,i);
    if ( cd->pid == pid){
      flag = 1;
      break;
    }
  }
  if(flag == 0){
    return(ECHILD);
  }
  //check the child is running
  lock_acquire(cd->waitPidLock);
  while(cd->state == 1){
    cv_wait(cd->waitPidCV, cd->waitPidLock);
  }
  lock_release(cd->waitPidLock);



  /* for now, just pretend the exitstatus is 0 */
  exitstatus = cd->exitcode;
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}

int sys_fork(struct trapframe *tf, pid_t *retval) {
  //build new proc
  KASSERT(curproc != NULL);
  struct proc *childProc = proc_create_runprogram(curproc->p_name);
  if(childProc == NULL){
    return EMPROC;
  }

  //build new addspace
  int cperr = as_copy(curproc_getas(), &(childProc->p_addrspace));
  if(cperr) {
    proc_destroy(childProc);
    return cperr;
  }

  //build new trapframe
  struct trapframe *ctf = kmalloc(sizeof(struct trapframe));
  if(ctf == NULL) {
    proc_destroy(childProc);
    return ENOMEM;
  }
  memcpy(ctf,tf,sizeof(struct trapframe));


  //assignpid (done in proc.c) and creat relationship
  childProc->parent = curproc;
  array_add(&curproc->children, childProc, NULL);  



  //transfer thread
  int therr = thread_fork(curthread->t_name, childProc, &enter_forked_process, ctf, 0);
  if(therr) {
    proc_destroy(childProc);
    kfree(ntf);
    ctf = NULL;
    return therr;
  }

  *retval = childProc->pid;
  return 0;



  
    
  };



