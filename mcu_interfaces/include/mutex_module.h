#ifndef _MUTEX_MODULE_H
#define _MUTEX_MODULE_H


typedef void* mutex_t;

mutex_t mutex_init(void);
void mutex_lock(mutex_t *mu);
void mutex_unlock(mutex_t *mu);

#endif