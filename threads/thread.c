#include "threads/thread.h"
#include <debug.h>
#include <stddef.h>
#include <random.h>
#include <stdio.h>
#include <string.h>
#include "threads/flags.h"
#include "threads/interrupt.h"
#include "threads/intr-stubs.h"
#include "threads/palloc.h"
#include "threads/synch.h"
#include "threads/vaddr.h"
#include "intrinsic.h"
#ifdef USERPROG
#include "userprog/process.h"
#endif

/* Random value for struct thread's `magic' member.
	 Used to detect stack overflow.  See the big comment at the top
	 of thread.h for details. */
#define THREAD_MAGIC 0xcd6abf4b

/* Random value for basic thread
	 Do not modify this value. */
#define THREAD_BASIC 0xd42df210

/* List of processes in THREAD_READY state, that is, processes
	 that are ready to run but not actually running. */
static struct list ready_list;

/* ----- project 1 ------------ */
// THREAD_BLOCKED 상태의 스레드를 관리하기 위한 리스트 자료구조 추가 (Alarm Clock - sleep_list)
static struct list sleep_list; /* sleep list for blocked threads */
/* --------------------------- */

/* Idle thread. */
static struct thread *idle_thread;

/* Initial thread, the thread running init.c:main(). */
static struct thread *initial_thread;

/* Lock used by allocate_tid(). */
static struct lock tid_lock;

/* Thread destruction requests */
static struct list destruction_req;

/* Statistics. */
static long long idle_ticks;	 /* # of timer ticks spent idle. */
static long long kernel_ticks; /* # of timer ticks in kernel threads. */
static long long user_ticks;	 /* # of timer ticks in user programs. */

/* Scheduling. */
#define TIME_SLICE 4					/* # of timer ticks to give each thread. */
static unsigned thread_ticks; /* # of timer ticks since last yield. */

/* ---------- project 1 ------------ */
static int64_t next_tick_to_awake; /* the earliest awake time in sleep list */
/* --------------------------------- */

/* If false (default), use round-robin scheduler.
	 If true, use multi-level feedback queue scheduler.
	 Controlled by kernel command-line option "-o mlfqs". */
bool thread_mlfqs;

static void kernel_thread(thread_func *, void *aux);

static void idle(void *aux UNUSED);
static struct thread *next_thread_to_run(void);
static void init_thread(struct thread *, const char *name, int priority);
static void do_schedule(int status);
static void schedule(void);
static tid_t allocate_tid(void);

/* ------------------- project 1 -------------------- */
void thread_sleep(int64_t ticks);
void thread_awake(int64_t ticks);
int64_t get_next_tick_to_awake(void);
bool cmp_priority(struct list_elem *element1, struct list_elem *element2, void *aux UNUSED);
bool preempt_by_priority(void);
bool thread_donate_priority_compare(struct list_elem *element1, struct list_elem *element2, void *aux UNUSED);
/* -------------------------------------------------- */
/* ------------------- project 2 -------------------- */
struct thread *get_child_by_tid(tid_t tid);
/* -------------------------------------------------- */

/* Returns true if T appears to point to a valid thread. */
#define is_thread(t) ((t) != NULL && (t)->magic == THREAD_MAGIC)

/* Returns the running thread.
 * Read the CPU's stack pointer `rsp', and then round that
 * down to the start of a page.  Since `struct thread' is
 * always at the beginning of a page and the stack pointer is
 * somewhere in the middle, this locates the curent thread.
 *
 * 실행 중인 스레드를 반환합니다.
 * CPU의 스택 포인터 'rsp'를 읽은 다음 페이지 시작 부분으로 반올림하십시오.
 * 'struct thread'는 항상 페이지의 시작 부분에 있고
 * 스택 포인터가 중간 어딘가에 있기 때문에, 이것은 현재 스레드를 찾는다.
 *
 * */
#define running_thread() ((struct thread *)(pg_round_down(rrsp())))

// Global descriptor table for the thread_start.
// Because the gdt will be setup after the thread_init, we should
// setup temporal gdt first.

// thread_start의 글로벌 설명자 테이블입니다.
// gdt는 이후에 설정될 것이기 때문에
// 우리는 먼저 시간 gdt를 설정해야 합니다.
static uint64_t gdt[3] = {0, 0x00af9a000000ffff, 0x00cf92000000ffff};

/*
	 Initializes the threading system by transforming the code
	 that's currently running into a thread.  This can't work in
	 general and it is possible in this case only because loader.S
	 was careful to put the bottom of the stack at a page boundary.

	 Also initializes the run queue and the tid lock.

	 After calling this function, be sure to initialize the page
	 allocator before trying to create any threads with
	 thread_create().

	 It is not safe to call thread_current() until this function
	 finishes.

	현재 실행 중인 코드를 스레드로 변환하여 스레딩 시스템을 초기화합니다.
	이것은 일반적으로 동작할 수 없으며 이 경우 로더 때문에 가능하다.
	S는 스택의 하단을 페이지 경계에 놓기 위해 조심했다.
	또한 실행 대기열 및 조석 잠금도 초기화합니다.
	이 함수를 호출한 후 thread_create()로 스레드를 만들기 전에
	페이지 할당자를 초기화해야 합니다.
	이 함수가 완료될 때까지 thread_current()를 호출하는 것은 안전하지 않습니다
*/

void thread_init(void)
{
	ASSERT(intr_get_level() == INTR_OFF);

	/* Reload the temporal gdt for the kernel
	 * This gdt does not include the user context.
	 * The kernel will rebuild the gdt with user context, in gdt_init ().
	 *
	 * 커널에 대한 시간 gdt 다시 로드 이 gdt에는 사용자 컨텍스트가 포함되지 않습니다.
	 * 커널은 사용자 컨텍스트 indt_init()를 사용하여 gdt를 재구성합니다.
	 *
	 * */
	struct desc_ptr gdt_ds = {
			.size = sizeof(gdt) - 1,
			.address = (uint64_t)gdt};
	lgdt(&gdt_ds);

	/* Init the globla thread context */
	lock_init(&tid_lock);
	list_init(&ready_list);
	list_init(&destruction_req);

	/* ------------- project 1 ---------------- */
	// sleep_list 초기화
	list_init(&sleep_list);
	// next_tick_to_awake 초기화
	next_tick_to_awake = INT64_MAX;
	/* ---------------------------------------- */

	/* Set up a thread structure for the running thread. */
	initial_thread = running_thread();
	init_thread(initial_thread, "main", PRI_DEFAULT);
	initial_thread->status = THREAD_RUNNING;
	initial_thread->tid = allocate_tid();
}

/* Starts preemptive thread scheduling by enabling interrupts.
	 Also creates the idle thread. */
void thread_start(void)
{
	/* Create the idle thread. */
	struct semaphore idle_started;
	sema_init(&idle_started, 0);

	// idle 스레드를 만들고 맨 처음 ready queue에 들어간다.
	// semaphore를 1로 UP 시켜 공유 자원의 접근을 가능하게 한 다음 바로 BLOCK된다.
	thread_create("idle", PRI_MIN, idle, &idle_started);

	/* Start preemptive thread scheduling. */
	intr_enable();

	/* Wait for the idle thread to initialize idle_thread. */
	sema_down(&idle_started);
}

/* Called by the timer interrupt handler at each timer tick.
	 Thus, this function runs in an external interrupt context.

	 각 타이머 체크에서 타이머 인터럽트 핸들러가 호출합니다.
	 따라서 이 함수는 외부 인터럽트 컨텍스트에서 실행됩니다.
*/
void thread_tick(void)
{
	struct thread *t = thread_current();

	/* Update statistics. */
	if (t == idle_thread)
		idle_ticks++;
#ifdef USERPROG
	else if (t->pml4 != NULL)
		user_ticks++;
#endif
	else
		kernel_ticks++;

	/* Enforce preemption. */
	if (++thread_ticks >= TIME_SLICE)
		intr_yield_on_return();
}

/* Prints thread statistics. */
void thread_print_stats(void)
{
	printf("Thread: %lld idle ticks, %lld kernel ticks, %lld user ticks\n",
				 idle_ticks, kernel_ticks, user_ticks);
}

/* Creates a new kernel thread named NAME with the given initial
	 PRIORITY, which executes FUNCTION passing AUX as the argument,
	 and adds it to the ready queue.  Returns the thread identifier
	 for the new thread, or TID_ERROR if creation fails.

	 If thread_start() has been called, then the new thread may be
	 scheduled before thread_create() returns.  It could even exit
	 before thread_create() returns.  Contrariwise, the original
	 thread may run for any amount of time before the new thread is
	 scheduled.  Use a semaphore or some other form of
	 synchronization if you need to ensure ordering.

	 The code provided sets the new thread's `priority' member to
	 PRIORITY, but no actual priority scheduling is implemented.
	 Priority scheduling is the goal of Problem 1-3.

	 지정된 초기 우선 순위를 사용하여 NAME이라는 이름의 새 커널 스레드를 만듭니다.
	 이 스레드는 인수로 AUX를 전달하는 함수를 실행하고 준비 대기열에 추가합니다.
	 새 스레드의 스레드 식별자를 반환하거나, 생성에 실패하면 TID_ERROR를 반환합니다.
	 thread_start()가 호출된 경우, thread_create()가 반환되기 전에 새 스레드를 예약할 수 있습니다.
	 thread_create()가 반환되기 전에 종료될 수도 있습니다.
	 반대로 원래 스레드는 새 스레드가 예약되기 전까지 임의의 시간 동안 실행될 수 있습니다.
	 순서를 확실히 정해야 하는 경우 세마포어 또는 다른 형식의 동기화를 사용하십시오.
	 제공된 코드는 새 스레드의 'priority' 멤버를 PRIGE로 설정하지만 실제 우선순위 스케줄링은
	 구현되지 않습니다.
	 우선 순위 예약은 문제 1-3의 목표입니다.
	 */

// thread_create(name, parent->priority, __do_fork, parent);
tid_t thread_create(const char *name, int priority, thread_func *function, void *aux)
{
	struct thread *t;
	tid_t tid;

	ASSERT(function != NULL);

	/* Allocate thread. */
	t = palloc_get_page(PAL_ZERO); // 4kb할당 memset
	if (t == NULL)
		return TID_ERROR;

	/* Initialize thread. */
	init_thread(t, name, priority);
	struct thread *parent = thread_current();

	// 프로세스 계층구조 구현
	list_push_back(&parent->child_list, &t->child_elem);

	// File Descripter구현
	// File Descriptor 테이블 메모리 할당
	t->fd_table = palloc_get_multiple(PAL_ZERO, FDT_PAGES); // 해당 프로세스의 FDT 공간 할당
	if (t->fd_table == NULL)
	{ // 제대로 공간이 할당되지 않았다면 에러.
		return TID_ERROR;
	}
	tid = t->tid = allocate_tid();
	t->fd_idx = 2;			// 0 : stdin, 1 : stdout이므로 새 파일이 open()하면 2부터 시작.
	t->fd_table[0] = 1; // 의미가 있는 숫자는 아니다. 다만 해당 인덱스(식별자)를 사용하는 파일이 존재하므로 넣어준 것.
	t->fd_table[1] = 2; // NULL 만들지 않으려고. 원래는 해당 파일을 가리키는 포인터가 들어가야함

	/* Call the kernel_thread if it scheduled.
	 * Note) rdi is 1st argument, and rsi is 2nd argument. */
	// 인터럽트 프레임
	t->tf.rip = (uintptr_t)kernel_thread;
	// printf("tid0!!!!!!!!!!!!!!!!!!! : %d\n", thread_current()->tid); // 3
	t->tf.R.rdi = (uint64_t)function;
	// printf("tid1??????????????????????! : %d\n", thread_current()->tid); // 3
	t->tf.R.rsi = (uint64_t)aux;
	t->tf.ds = SEL_KDSEG;
	t->tf.es = SEL_KDSEG;
	t->tf.ss = SEL_KDSEG;
	t->tf.cs = SEL_KCSEG;
	t->tf.eflags = FLAG_IF;
	// printf("tid1??????????????????????! : %d\n", thread_current()->tid); // 3
	/* Add to run queue. */
	// Thread의 unblock 후
	thread_unblock(t);

	// 현재 실행중인 thread와 우선순위를 비교하여, 새로 생성된 thread의 우선순위가 높다면
	if (preempt_by_priority())
	{
		// thread_yield()를 통해 CPU를 양보.
		thread_yield();
	}
	// printf("tid1??????????????????????! : %d\n", thread_current()->tid); // 3
	return tid;
}

/* Puts the current thread to sleep.  It will not be scheduled
	 again until awoken by thread_unblock().

	 This function must be called with interrupts turned off.  It
	 is usually a better idea to use one of the synchronization
	 primitives in synch.h.

	 현재 스레드를 절전 모드로 전환합니다.
	 thread_unblock()에 의해 활성화될 때까지 다시 예약되지 않습니다.
	 인터럽트를 끈 상태에서 이 함수를 호출해야 합니다.
	 일반적으로 synch.h에서 동기화 프리미티브 중 하나를 사용하는 것이 더 좋습니다.
		*/
void thread_block(void)
{
	ASSERT(!intr_context());
	ASSERT(intr_get_level() == INTR_OFF);
	thread_current()->status = THREAD_BLOCKED;
	schedule();
}

/* Transitions a blocked thread T to the ready-to-run state.
	 This is an error if T is not blocked.  (Use thread_yield() to
	 make the running thread ready.)

	 This function does not preempt the running thread.  This can
	 be important: if the caller had disabled interrupts itself,
	 it may expect that it can atomically unblock a thread and
	 update other data. */
void thread_unblock(struct thread *t)
{
	enum intr_level old_level;
	struct list_elem *e;

	ASSERT(is_thread(t));

	old_level = intr_disable();
	ASSERT(t->status == THREAD_BLOCKED);
	// 맨뒤로 넣지않고
	// list_push_back (&ready_list, &t->elem);
	// (Priority Scheduling - thread_unblock)
	// 선형팀색으로 적절한 자리 찾아서 들어가기(우선순위 낮은애를 뒤로)
	list_insert_ordered(&ready_list, &t->elem, &cmp_priority, NULL);
	t->status = THREAD_READY;
	intr_set_level(old_level);
}

/* Returns the name of the running thread. */
const char *
thread_name(void)
{
	return thread_current()->name;
}

/* Returns the running thread.
	 This is running_thread() plus a couple of sanity checks.
	 See the big comment at the top of thread.h for details.

	 실행 중인 스레드를 반환합니다. 이것은 running_thread()와 몇 가지 건전성 검사입니다.
	 자세한 내용은 스레드.h 상단에 있는 큰 주석을 참조하십시오.
	 */
struct thread *
thread_current(void)
{
	struct thread *t = running_thread();
	/* Make sure T is really a thread.
		 If either of these assertions fire, then your thread may
		 have overflowed its stack.  Each thread has less than 4 kB
		 of stack, so a few big automatic arrays or moderate
		 recursion can cause stack overflow.

		 T가 진짜 스레드인지 확인하십시오.
		 이러한 주장 중 하나가 실행되면 스레드가 스택을 오버플로했을 수 있습니다.
		 각 스레드는 4kB 미만의 스택을 가지므로 몇 개의 큰 자동 배열이나
		 적당한 재귀로 인해 스택 오버플로가 발생할 수 있습니다.
	*/
	ASSERT(is_thread(t));
	ASSERT(t->status == THREAD_RUNNING);

	return t;
}

/* Returns the running thread's tid. */
tid_t thread_tid(void)
{
	return thread_current()->tid;
}

/* Deschedules the current thread and destroys it.  Never
	 returns to the caller. */
void thread_exit(void)
{
	ASSERT(!intr_context());

#ifdef USERPROG
	process_exit();
#endif

	/* Just set our status to dying and schedule another process.
		 We will be destroyed during the call to schedule_tail(). */
	intr_disable();
	do_schedule(THREAD_DYING);
	NOT_REACHED();
}

/* Yields the CPU.  The current thread is not put to sleep and
	 may be scheduled again immediately at the scheduler's whim. */
// CPU를 양보하고, thread를 ready_list에 삽입(Alarm Clock)
void thread_yield(void)
{
	struct thread *curr = thread_current();
	enum intr_level old_level;

	// 외부 인터럽트를 수행중이라면 종료. 외부 인터럽트는 인터럽트 당하면 안 된다.
	ASSERT(!intr_context());

	old_level = intr_disable();		// 인터럽트를 disable한다.

	// 만약 현재 스레드가 idle 스레드가 아니라면 ready queue에 다시 담는다.
	// idle 스레드라면 담지 않는다. 어차피 static으로 선언되어 있어, 필요할 때 불러올 수 있다.
	if (curr != idle_thread) {
		list_push_back(&ready_list, &curr->elem);
	}

	do_schedule(THREAD_READY);
	intr_set_level(old_level);
}

/* Sets the current thread's priority to NEW_PRIORITY. */
void thread_set_priority(int new_priority)
{
	thread_current()->initial_priority = new_priority;

	/* --------- project1 ---------- */
	refresh_priority();
	if (preempt_by_priority())
	{
		thread_yield();
	}
	/* ----------------------------- */
}

/* Returns the current thread's priority. */
int thread_get_priority(void)
{
	return thread_current()->priority;
}

/* Sets the current thread's nice value to NICE. */
void thread_set_nice(int nice UNUSED)
{
	/* TODO: Your implementation goes here */
}

/* Returns the current thread's nice value. */
int thread_get_nice(void)
{
	/* TODO: Your implementation goes here */
	return 0;
}

/* Returns 100 times the system load average. */
int thread_get_load_avg(void)
{
	/* TODO: Your implementation goes here */
	return 0;
}

/* Returns 100 times the current thread's recent_cpu value. */
int thread_get_recent_cpu(void)
{
	/* TODO: Your implementation goes here */
	return 0;
}

/* Idle thread.  Executes when no other thread is ready to run.

	 The idle thread is initially put on the ready list by
	 thread_start().  It will be scheduled once initially, at which
	 point it initializes idle_thread, "up"s the semaphore passed
	 to it to enable thread_start() to continue, and immediately
	 blocks.  After that, the idle thread never appears in the
	 ready list.  It is returned by next_thread_to_run() as a
	 special case when the ready list is empty.

	 유휴 스레드. 실행할 준비가 된 다른 스레드가 없을 때 실행합니다.
	 유휴 스레드는 처음에 thread_start()로 준비 목록에 표시됩니다.
	 처음에 한 번 예약되며, 이때 idle_thread를 초기화하고,
	 스레드_start()를 계속 사용할 수 있도록 전달된 세마포어를 "업"한 후 즉시 차단합니다.
	 그 후 유휴 스레드는 준비 목록에 나타나지 않습니다.
	 준비 목록이 비어 있으면 next_thread_to_run()이 특수 케이스로 반환합니다.
	 */
static void
idle(void *idle_started_ UNUSED)
{
	struct semaphore *idle_started = idle_started_;

	// 현재 돌고 있는 스레드가 idle밖에 없다.
	idle_thread = thread_current();
	sema_up (idle_started);  // semaphore의 값을 1로 만들어 줘 공유 자원의 공유(인터럽트) 가능!

	for (;;)
	{
		/* Let someone else run. */
		intr_disable();		// 자기 자신(idle)을 BLOCK해주기 전까지 인터럽트 당하면 안되므로 먼저 disable한다.
		thread_block();		// 자기 자신을 BLOCK한다.

		/* Re-enable interrupts and wait for the next one.

			 The `sti' instruction disables interrupts until the
			 completion of the next instruction, so these two
			 instructions are executed atomically.  This atomicity is
			 important; otherwise, an interrupt could be handled
			 between re-enabling interrupts and waiting for the next
			 one to occur, wasting as much as one clock tick worth of
			 time.

			 See [IA32-v2a] "HLT", [IA32-v2b] "STI", and [IA32-v3a]
			 7.11.1 "HLT Instruction".
			 인터럽트를 다시 활성화하고 다음 인터럽트를 기다립니다.
			 'sti' 명령은 다음 명령이 완료될 때까지 인터럽트를 비활성화하므로
			 이 두 명령은 원자적으로 실행됩니다. 이 원자성은 중요하다;
			 그렇지 않으면 인터럽트를 다시 활성화하고 다음 인터럽트가
			 발생하기를 기다리는 사이에 인터럽트가 처리되어 클럭 틱 한 개만큼의 시간을 낭비할 수 있다.
			 [IA32-v2a] "HLT", [IA32-v2b] "STI" 및 [IA32-v3a] 7.11.1 "HLT 지침"을 참조하십시오.
			 */
		asm volatile("sti; hlt"
								 :
								 :
								 : "memory");
	}
}

/* Function used as the basis for a kernel thread. */
static void
kernel_thread(thread_func *function, void *aux)
{
	ASSERT(function != NULL);
	intr_enable(); /* The scheduler runs with interrupts off. */
	// printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ %d, %p\n", thread_current()->tid, function)
	function(aux); /* Execute the thread function. */
	thread_exit(); /* If function() returns, kill the thread. */
}

/* Does basic initialization of T as a blocked thread named
	 NAME. */
static void
init_thread(struct thread *t, const char *name, int priority)
{
	ASSERT(t != NULL);
	ASSERT(PRI_MIN <= priority && priority <= PRI_MAX);
	ASSERT(name != NULL);

	memset(t, 0, sizeof *t);
	t->status = THREAD_BLOCKED;
	// name 을 t->name로 복사
	strlcpy(t->name, name, sizeof t->name);
	// 커널 스택 포인터의 위치(메인 쓰레드의 커널 스택)
	t->tf.rsp = (uint64_t)t + PGSIZE - sizeof(void *);
	t->priority = priority;
	t->magic = THREAD_MAGIC;

	/* -------- Project 1 ----------- */
	list_init(&t->donation_list);
	t->initial_priority = priority;
	t->wait_on_lock = NULL;
	/* ------------------------------ */

	/* -------- Project 2 ----------- */
	t->exit_status = 0;
	list_init(&t->child_list);
	sema_init(&t->fork_sema, 0);
	sema_init(&t->wait_sema, 0);
	sema_init(&t->exit_sema, 0);

	t->running = NULL;
	/* ------------------------------ */
}

/* Chooses and returns the next thread to be scheduled.  Should
	 return a thread from the run queue, unless the run queue is
	 empty.  (If the running thread can continue running, then it
	 will be in the run queue.)  If the run queue is empty, return
	 idle_thread.

	 예약할 다음 스레드를 선택하고 반환합니다.
	 실행 대기열이 비어 있지 않은 경우 실행 대기열에서 스레드를 반환해야 합니다.
	 (실행 중인 스레드가 계속 실행될 수 있으면 해당 스레드는 실행 대기열에 있게 됩니다.)
	 실행 대기열이 비어 있으면 idle_thread를 반환합니다.
	 */
static struct thread *
next_thread_to_run(void)
{
	if (list_empty(&ready_list))
		return idle_thread;
	else
	{
		/* ---------------- project 1 -----------------*/
		list_sort(&ready_list, &cmp_priority, NULL);
		/* --------------------------------------------*/
		// 어떤 리스트의 원소가 담겨 있는 구조체의 시작 주소를 반환한다.
		// list_pop_front (&ready_list) => ready_list에서 팝된항목의 주소를 가져옴
		// list_entry => 쓰레드구조체에서 팝된 항목의 주소로 쓰레드 가져옴
		return list_entry(list_pop_front(&ready_list), struct thread, elem);
	}
}

/* Use iretq to launch the thread */
void do_iret(struct intr_frame *tf)
{
	__asm __volatile(
			// 인터럽트 프레임값을 레지스터에 넘겨줌 source => drain
			"movq %0, %%rsp\n"
			"movq 0(%%rsp),%%r15\n"
			"movq 8(%%rsp),%%r14\n"
			"movq 16(%%rsp),%%r13\n"
			"movq 24(%%rsp),%%r12\n"
			"movq 32(%%rsp),%%r11\n"
			"movq 40(%%rsp),%%r10\n"
			"movq 48(%%rsp),%%r9\n"
			"movq 56(%%rsp),%%r8\n"
			"movq 64(%%rsp),%%rsi\n"
			"movq 72(%%rsp),%%rdi\n"
			"movq 80(%%rsp),%%rbp\n"
			"movq 88(%%rsp),%%rdx\n"
			"movq 96(%%rsp),%%rcx\n"
			"movq 104(%%rsp),%%rbx\n"
			"movq 112(%%rsp),%%rax\n"
			"addq $120,%%rsp\n"
			"movw 8(%%rsp),%%ds\n"
			"movw (%%rsp),%%es\n"
			"addq $32, %%rsp\n"
			"iretq"
			:
			: "g"((uint64_t)tf)
			: "memory");
}

/* Switching the thread by activating the new thread's page
	 tables, and, if the previous thread is dying, destroying it.

	 At this function's invocation, we just switched from thread
	 PREV, the new thread is already running, and interrupts are
	 still disabled.

	 It's not safe to call printf() until the thread switch is
	 complete.  In practice that means that printf()s should be
	 added at the end of the function.

	 새 스레드의 페이지 테이블을 활성화하여 스레드를 전환하고
	 이전 스레드가 소멸되는 경우 스레드를 삭제합니다.
	 이 함수의 호출 시, 우리는 방금 스레드 PREV에서 전환했고,
	 새로운 스레드는 이미 실행 중이며, 인터럽트는 여전히 비활성화되어 있다.
	 스레드 스위치가 완료될 때까지 printf()를 호출하는 것은 안전하지 않습니다.
	 실제로 이는 기능 끝에 printf()s를 추가해야 한다는 것을 의미한다.
	 */
static void
thread_launch(struct thread *th)
{
	uint64_t tf_cur = (uint64_t)&running_thread()->tf;
	uint64_t tf = (uint64_t)&th->tf;
	ASSERT(intr_get_level() == INTR_OFF);

	/* The main switching logic.
	 * We first restore the whole execution context into the intr_frame
	 * and then switching to the next thread by calling do_iret.
	 * Note that, we SHOULD NOT use any stack from here
	 * until switching is done.
	 *
		주 스위칭 로직.
		먼저 전체 실행 컨텍스트를 intr_frame으로 복원한 다음
		do_iret을 호출하여 다음 스레드로 전환합니다.
		전환이 완료될 때까지 여기서 스택을 사용하지 마십시오.
	 */
	__asm __volatile(
			/* Store registers that will be used. */
			"push %%rax\n"
			"push %%rbx\n"
			"push %%rcx\n"
			/* Fetch input once */
			"movq %0, %%rax\n"
			"movq %1, %%rcx\n"
			"movq %%r15, 0(%%rax)\n"
			"movq %%r14, 8(%%rax)\n"
			"movq %%r13, 16(%%rax)\n"
			"movq %%r12, 24(%%rax)\n"
			"movq %%r11, 32(%%rax)\n"
			"movq %%r10, 40(%%rax)\n"
			"movq %%r9, 48(%%rax)\n"
			"movq %%r8, 56(%%rax)\n"
			"movq %%rsi, 64(%%rax)\n"
			"movq %%rdi, 72(%%rax)\n"
			"movq %%rbp, 80(%%rax)\n"
			"movq %%rdx, 88(%%rax)\n"
			"pop %%rbx\n" // Saved rcx
			"movq %%rbx, 96(%%rax)\n"
			"pop %%rbx\n" // Saved rbx
			"movq %%rbx, 104(%%rax)\n"
			"pop %%rbx\n" // Saved rax
			"movq %%rbx, 112(%%rax)\n"
			"addq $120, %%rax\n"
			"movw %%es, (%%rax)\n"
			"movw %%ds, 8(%%rax)\n"
			"addq $32, %%rax\n"
			"call __next\n" // read the current rip.
			"__next:\n"
			"pop %%rbx\n"
			"addq $(out_iret -  __next), %%rbx\n"
			"movq %%rbx, 0(%%rax)\n" // rip
			"movw %%cs, 8(%%rax)\n"	 // cs
			"pushfq\n"
			"popq %%rbx\n"
			"mov %%rbx, 16(%%rax)\n" // eflags
			"mov %%rsp, 24(%%rax)\n" // rsp
			"movw %%ss, 32(%%rax)\n"
			"mov %%rcx, %%rdi\n"
			"call do_iret\n"
			"out_iret:\n"
			:
			: "g"(tf_cur), "g"(tf)
			: "memory");
}

/* Schedules a new process. At entry, interrupts must be off.
 * This function modify current thread's status to status and then
 * finds another thread to run and switches to it.
 * It's not safe to call printf() in the schedule().
 *
 * 새 프로세스를 예약합니다. 입력 시 인터럽트는 해제되어 있어야 합니다.
 * 이 함수는 현재 스레드의 상태를 상태로 수정한 다음
 * 실행할 다른 스레드를 찾아 스레드로 전환합니다.
 * 스케줄에서 printf()를 호출하는 것은 안전하지 않습니다.
 *
 * */
static void
do_schedule(int status)
{
	ASSERT(intr_get_level() == INTR_OFF);
	ASSERT(thread_current()->status == THREAD_RUNNING);
	while (!list_empty(&destruction_req))
	{
		struct thread *victim =
				list_entry(list_pop_front(&destruction_req), struct thread, elem);
		palloc_free_page(victim);
	}
	thread_current()->status = status;
	// printf("tid *************************** : %d\n", thread_current()->tid);
	schedule();
}

// (Alarm Clock - sleep_list 초기화)
static void
schedule(void)
{
	// 바로전에 실행되던 쓰레드
	// rrsp () => 현재 스택의 가장위로 움직여
	// pg_round_down => 입력 변수에서 0만큼 떨어진곳 그곳 (return *void)
	// ((struct thread *) (pg_round_down (rrsp ())))
	struct thread *curr = running_thread();
	// 이제 runnung할 쓰레드
	struct thread *next = next_thread_to_run();

	// 인터럽트가 들어오지 않는 상태로 설정
	ASSERT(intr_get_level() == INTR_OFF);
	// 현재 쓰레드가 running 상태가 아니면
	ASSERT(curr->status != THREAD_RUNNING);
	// runnung할 쓰레드가 존재하면
	ASSERT(is_thread(next));

	/* Mark us as running. */
	// next를 실행상태로
	next->status = THREAD_RUNNING;

	/* Start new time slice. */
	thread_ticks = 0;

#ifdef USERPROG
	/* Activate the new address space. */
	process_activate(next);
#endif

	if (curr != next)
	{
		/* If the thread we switched from is dying, destroy its struct
			 thread. This must happen late so that thread_exit() doesn't
			 pull out the rug under itself.
			 We just queuing the page free reqeust here because the page is
			 currently used bye the stack.
			 The real destruction logic will be called at the beginning of the
			 schedule().

			 만약 우리가 바꾼 thread가 죽어가고 있다면,
			 그것의 구조 나사산을 파괴하라.
			 thread_exit()가 스스로 러그를 뽑지 않도록 이 작업은 늦게 발생해야 합니다.
			 페이지가 현재 스택에서 사용 중이므로 여기서 페이지 프리 요청을 대기열에 넣습니다.
			 실제 파괴 논리는 schedule()의 시작 부분에서 호출됩니다.

			 */
		if (curr && curr->status == THREAD_DYING && curr != initial_thread)
		{
			ASSERT(curr != next);
			list_push_back(&destruction_req, &curr->elem);
		}

		/* Before switching the thread, we first save the information
		 * of current running.
		 스레드를 전환하기 전에 먼저 전류 실행 정보를 저장합니다.
		 */
		// printf("tid !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! : %d\n", next->tid);
		thread_launch(next);
	}
}

/* Returns a tid to use for a new thread. */
static tid_t
allocate_tid(void)
{
	static tid_t next_tid = 1;
	tid_t tid;
	lock_acquire(&tid_lock);
	// printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ %d", thread_current()->tid); 3
	tid = next_tid++;
	// printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ %d", thread_current()->tid); 3
	lock_release(&tid_lock);

	return tid;
}

/* --------------------- project 1 ------------------------ */
// TODO Alarm Clock 1
// ready_list에서 제거, sleep queue에 추가
// 구현할 함수 선언(Alarm Clock - sleep_list 초기화)
// 실행 중인 쓰레드를 슬립으로 만든다
/*  make thread sleep in timer_sleep() (../device/timer.c)  */
void thread_sleep(int64_t ticks)
{
	/*
	현재 스레드가 idle 스레드가 아닐경우
	thread의 상태를 BLOCKED로 바꾸고 깨어나야 할 ticks을 저장,
	슬립 큐에 삽입하고, awake함수가 실행되어야 할 tick값을 update
	*/
	struct thread *curr;
	enum intr_level old_level;

	// 외부 인터럽트 프로세스 수행 중이면 True, 아니면 False
	ASSERT(!intr_context());
	// 해당 과정중에는 인터럽트를 받아들이지 않는다.
	old_level = intr_disable();
	// 현재 쓰레드가
	curr = thread_current();
	// idle 스레드가 아닐경우
	ASSERT(curr != idle_thread)
	// thread의 상태를 BLOCKED로 바꾸고
	// 깨어나야 할 ticks를 curr의 wakeup_tick 저장
	if (next_tick_to_awake > ticks)
	{
		// awake함수가 실행되어야 할 tick값을 update
		next_tick_to_awake = ticks;
	}
	curr->wake_up_tick = ticks;
	// 슬립 큐에 삽입하고
	list_push_back(&sleep_list, &curr->elem);
	// 현재 스레드를 슬립 큐에 삽입한 후에 !스케줄한다!
	thread_block();
	// 인터럽트 받읋수 있는 상태로 만들기
	intr_set_level(old_level);
}

/* make thread awake in timer_interrupt() (../device/timer.c) */
// 슬립큐에서 깨워야할 스레드를 깨움
void thread_awake(int64_t ticks)
{
	/*
	sleep list의 모든 entry 를 순회하며 다음과 같은 작업을 수행한다.
	현재 tick이 깨워야 할 tick 보다 크거나 같다면 슬립 큐에서 제거하고
	unblock 한다. 작다면 update_next_tick_to_awake() 를 호출한다.
	*/

	next_tick_to_awake = INT64_MAX;
	enum intr_level old_level;
	struct list_elem *e;
	ASSERT(intr_context());

	// sleep list의 모든 entry 를 순회하며 다음과 같은 작업을 수행한다.
	for (e = list_begin(&sleep_list); e != list_end(&sleep_list);)
	{
		struct thread *t = list_entry(e, struct thread, elem);
		// 현재 tick이 깨워야 할 tick 보다 크거나 같다면
		if (t->wake_up_tick <= ticks)
		{
			// 슬립 큐에서 제거하고 unblock 한다.
			e = list_remove(e);
			thread_unblock(t);
			if (preempt_by_priority())
			{
				intr_yield_on_return();
			}
		}
		// 현재틱이 더 작음
		else
		{
			if (t->wake_up_tick < next_tick_to_awake)
			{
				next_tick_to_awake = t->wake_up_tick;
			}
			e = list_next(e);
		}
	}
}

/* global function to get value of next_tick_to_awake */
// thread.c의 next_tick_to_awake 반환
int64_t get_next_tick_to_awake(void)
{
	// next_tick_to_awake 을 반환한다.
	return next_tick_to_awake;
}

/* compare priority between running thread and highest priority thread in ready_list
	if running thread priority < highest priority thread in ready_list , return true */
bool preempt_by_priority(void)
{
	int curr_priority;
	struct thread *max_ready_thread;
	struct list_elem *max_ready_elem;

	curr_priority = thread_get_priority();

	if (list_empty(&ready_list))
		return false; /* !! if ready list is empty, return false directly !!*/

	list_sort(&ready_list, &cmp_priority, NULL);
	max_ready_elem = list_begin(&ready_list);
	max_ready_thread = list_entry(max_ready_elem, struct thread, elem);

	return curr_priority < max_ready_thread->priority;
}

/* compare threads' priority of element1 and element2 by **elem** in struct thread */
bool cmp_priority(struct list_elem *element1, struct list_elem *element2, void *aux UNUSED)
{
	struct thread *t1 = list_entry(element1, struct thread, elem);
	struct thread *t2 = list_entry(element2, struct thread, elem);
	return t1->priority > t2->priority;
}

/* compare threads' priority of element1 and element2 by **donation_elem** in struct thread */
// 우선순위 순으로 들어감
bool thread_donate_priority_compare(struct list_elem *element1, struct list_elem *element2, void *aux UNUSED)
{
	struct thread *t1 = list_entry(element1, struct thread, donation_elem);
	struct thread *t2 = list_entry(element2, struct thread, donation_elem);
	return t1->priority > t2->priority;
}

/* ------------------- project 1 functions end ------------------------------- */

/* --------------------- project 2 ------------------------ */
struct thread *get_child_by_tid(tid_t tid)
{
	struct thread *parent = thread_current();
	struct thread *child;
	struct list *child_list = &parent->child_list;
	struct list_elem *e;
	// printf("tid1??????????????????????! : %d\n", thread_current()->tid); // 3
	if (list_empty(child_list))
	{
		return NULL;
	}

	for (e = list_begin(child_list); e != list_end(child_list); e = list_next(e))
	{
		child = list_entry(e, struct thread, child_elem);
		if (child->tid == tid)
		{
			// printf("tid1??????????????????????! : %d\n", thread_current()->tid); // 3
			return child;
		}
	}
	return NULL;
}

/* ------------------- project 2 functions end ------------------------------- */