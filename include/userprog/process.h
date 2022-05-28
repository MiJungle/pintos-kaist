#ifndef USERPROG_PROCESS_H
#define USERPROG_PROCESS_H

#include "threads/thread.h"

tid_t process_create_initd (const char *file_name);
tid_t process_fork (const char *name, struct intr_frame *if_);
int process_exec (void *f_name);
int process_wait (tid_t);
void process_exit (void);
void process_activate (struct thread *next);
// 함수 호출 규약에 따라 유저 스택에 프로그램 이름과 인자들을 저장(Argument Passing)
// void argument_stack(char **argv ,int argc ,void **esp);
void argument_stack(char **argv, int argc, struct intr_frame *if_);

#endif /* userprog/process.h */
