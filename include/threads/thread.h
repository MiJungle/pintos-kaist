#ifndef THREADS_THREAD_H
#define THREADS_THREAD_H
#define USERPROG

#include <debug.h>
#include <list.h>
#include <stdint.h>
#include "threads/interrupt.h"
#include "threads/synch.h"
#ifdef VM
#include "vm/vm.h"
#endif


/* States in a thread's life cycle. */
enum thread_status {
	THREAD_RUNNING,     /* Running thread. */
	THREAD_READY,       /* Not running but ready to run. */
	THREAD_BLOCKED,     /* Waiting for an event to trigger. */
	THREAD_DYING        /* About to be destroyed. */
};

/* Thread identifier type.
   You can redefine this to whatever type you like. */
typedef int tid_t;
#define TID_ERROR ((tid_t) -1)          /* Error value for tid_t. */

/* Thread priorities. */
#define PRI_MIN 0                       /* Lowest priority. */
#define PRI_DEFAULT 31                  /* Default priority. */
#define PRI_MAX 63                      /* Highest priority. */

/* ------------------ project2 -------------------- */
#define FDT_PAGES 3		/* pages to allocate for file descriptor tables (thread_create, process_exit) */
#define FDCOUNT_LIMIT FDT_PAGES *(1 << 9)		/* limit fd_idx */
/* ------------------------------------------------ */

/* A kernel thread or user process.
 *
 * Each thread structure is stored in its own 4 kB page.  The
 * thread structure itself sits at the very bottom of the page
 * (at offset 0).  The rest of the page is reserved for the
 * thread's kernel stack, which grows downward from the top of
 * the page (at offset 4 kB).  Here's an illustration:
 * 
 * 커널 스레드 또는 사용자 프로세스입니다.
 * 각 스레드 구조는 자체 4kB 페이지에 저장됩니다. 
 * 스레드 구조 자체는 페이지의 맨 아래(오프셋 0)에 위치합니다. 
 * 페이지의 나머지 부분은 스레드의 커널 스택을 위해 예약되어 있으며, 
 * 스레드의 커널 스택은 페이지의 위쪽에서 아래쪽으로 확장된다(오프셋 4kB).
 *
 *
 *      4 kB +---------------------------------+
 *           |          kernel stack           |
 *           |                |                |
 *           |                |                |
 *           |                V                |
 *           |         grows downward          |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           +---------------------------------+
 *           |              magic              |
 *           |            intr_frame           |
 *           |                :                |
 *           |                :                |
 *           |               name              |
 *           |              status             |
 *      0 kB +---------------------------------+
 *
 * The upshot of this is twofold:
 *
 *    1. First, `struct thread' must not be allowed to grow too
 *       big.  If it does, then there will not be enough room for
 *       the kernel stack.  Our base `struct thread' is only a
 *       few bytes in size.  It probably should stay well under 1
 *       kB.
 * 
 * 			 첫째, 'struct thread'가 너무 커서는 안 된다. 만약 그렇다면, 
 * 			 커널 스택을 위한 충분한 공간이 없을 것이다. 
 * 			 우리의 기본 'struct thread'는 크기가 몇 바이트에 불과하다. 
 * 			 그것은 아마도 1kB 이하로 잘 유지될 것이다.
 *
 *    2. Second, kernel stacks must not be allowed to grow too
 *       large.  If a stack overflows, it will corrupt the thread
 *       state.  Thus, kernel functions should not allocate large
 *       structures or arrays as non-static local variables.  Use
 *       dynamic allocation with malloc() or palloc_get_page()
 *       instead.
 * 
 * 			 둘째, 커널 스택이 너무 커서는 안 됩니다.
 * 			 스택이 오버플로되면 스레드 상태가 손상됩니다.
 * 			 그러므로 커널 함수는 큰 구조나 배열을 정적이 아닌 로컬 변수로 할당해서는 안 된다. 
 * 			 대신 malloc() 또는 palloc_get_page()와 함께 동적 할당을 사용합니다.
 *
 * The first symptom of either of these problems will probably be
 * an assertion failure in thread_current(), which checks that
 * the `magic' member of the running thread's `struct thread' is
 * set to THREAD_MAGIC.  Stack overflow will normally change this
 * value, triggering the assertion. 
 * 
 * 이러한 문제의 첫 번째 증상은 실행 중인 스레드의 
 * 'struct thread'의 'magic' 멤버가 TRADE_MAGIC으로 설정되어 있는지 
 * 확인하는 thread_current()의 어설션 실패일 수 있습니다.
 * 스택 오버플로는 일반적으로 이 값을 변경하여 어설션을 트리거합니다.
 * */

/* The `elem' member has a dual purpose.  It can be an element in
 * the run queue (thread.c), or it can be an element in a
 * semaphore wait list (synch.c).  It can be used these two ways
 * only because they are mutually exclusive: only a thread in the
 * ready state is on the run queue, whereas only a thread in the
 * blocked state is on a semaphore wait list. 
 * 
 * "elem" 멤버는 이중의 목적을 가지고 있다.
 * 실행 대기열의 요소(thread.c)이거나 세마포어 대기 목록(synch.c)의 요소일 수 있습니다. 
 * 이 두 가지 방법은 서로 배타적이기 때문에 사용할 수 있다: 
 * 실행 대기열에 준비된 상태의 스레드만 있는 반면 차단된 상태의 스레드만 세마포어 대기 목록에 있다
 * 
 * 
 * */
struct thread {
	/* Owned by thread.c. */
	tid_t tid;                          /* Thread identifier. */
	enum thread_status status;          /* Thread state. */
	char name[16];                      /* Name (for debugging purposes). */
	int priority;                       /* Priority. */

	/* Shared between thread.c and synch.c. */
	struct list_elem elem;              /* List element. */

	/* ----- PROJECT 1 --------- */
	int64_t wake_up_tick; /* thread's wakeup_time */
	int initial_priority; /* thread's initial priority */
	// 깨어나야할 tick 저장 (Alarm Clock - wakeup_tick)
	struct lock *wait_on_lock; /* which lock thread is waiting for  */
	// 자신에게 priority를 donate한 스레드의 리스트
	struct list donation_list; /* list of threads that donate priority to **this thread** */
	// priority를 donate한 스레드들의 리스트를 관리하기 위한 element.
	// 이 element를 통해 자신이 우선 순위를 donate한 스레드의 donations 리스트에 연결된다.
	struct list_elem donation_elem; /* prev and next pointer of donation_list where **this thread donate** */
	/* ------------------------- */

	/* ---------- Project 2 ---------- */
	int exit_status;	 	/* to give child exit_status to parent */
	int fd_idx;					// fd table에 open spot의 index
	struct file **fd_table;   // file descriptor table의 시작주소 가리키게 초기화
	struct file *running;			// 현재 스레드가 사용 중인 파일(load하고 있는 파일)

	struct intr_frame parent_if;	/* Information of parent's frame */
	struct list child_list; 		 /* list of threads that are made by this thread */
	struct list_elem child_elem; /* elem for this thread's parent's child_list */
	struct semaphore fork_sema;  /* parent thread should wait while child thread copy parent */
	struct semaphore wait_sema;
	struct semaphore exit_sema;
	/* ------------------------------- */
	
#ifdef USERPROG
	/* Owned by userprog/process.c. */
	uint64_t *pml4;                     /* Page map level 4 */
#endif
#ifdef VM
	/* Table for whole virtual memory owned by thread. */
	struct supplemental_page_table spt;
#endif

	/* Owned by thread.c. */
	struct intr_frame tf;               /* Information for switching */
	// 스레드의 스택 맨 끝 값을 의미?
	unsigned magic;                     /* Detects stack overflow. */
};

/* If false (default), use round-robin scheduler.
   If true, use multi-level feedback queue scheduler.
   Controlled by kernel command-line option "-o mlfqs". */
extern bool thread_mlfqs;

void thread_init (void);
void thread_start (void);

void thread_tick (void);
void thread_print_stats (void);

typedef void thread_func (void *aux);
tid_t thread_create (const char *name, int priority, thread_func *, void *);

void thread_block (void);
void thread_unblock (struct thread *);

struct thread *thread_current (void);
tid_t thread_tid (void);
const char *thread_name (void);

void thread_exit (void) NO_RETURN;
void thread_yield (void);

int thread_get_priority (void);
void thread_set_priority (int);

int thread_get_nice (void);
void thread_set_nice (int);
int thread_get_recent_cpu (void);
int thread_get_load_avg (void);

void do_iret (struct intr_frame *tf);

/* ------------- project 1 ------------ */
// 구현할 함수 선언(Alarm Clock - sleep_list 초기화)
// 실행 중인 쓰레드를 슬립으로 만든다
void thread_sleep(int64_t ticks);
// 슬립큐에서 깨워야할 스레드를 깨움
void thread_awake(int64_t ticks);
// next_tick_to_awake 최소값 갱신?
int64_t get_next_tick_to_awake(void);

bool cmp_priority(struct list_elem *element1, struct list_elem *element2, void *aux);
bool preempt_by_priority(void);
bool thread_donate_priority_compare (struct list_elem *element1, struct list_elem *element2, void *aux);
/* ------------------------------------- */
/* ------------------- project 2 -------------------- */
struct thread* get_child_by_tid(tid_t tid);
/* -------------------------------------------------- */
#endif /* threads/thread.h */
