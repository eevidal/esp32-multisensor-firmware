#ifndef _MUTEX_MODULE_H
#define _MUTEX_MODULE_H


typedef void* mutex_t;

/**
 * Initializes a mutex.
 *
 * This function creates a binary semaphore and returns a pointer to it, which can be used as a mutex.
 * If memory allocation fails, the function returns NULL.
 *
 * @return A pointer to the newly created mutex, or NULL if memory allocation fails.
 */
mutex_t mutex_init(void);

/**
 * Acquires a mutex lock.
 *
 * This function blocks the calling thread until the mutex is available. 
 * Once acquired, the mutex is locked, preventing other threads from acquiring it.
 *
 * @param mutex A pointer to the mutex to be locked.
 */
void mutex_lock(mutex_t *mu);

/**
 * Releases a mutex lock.
 *
 * This function releases the mutex lock, allowing other threads to acquire it.
 *
 * @param mutex A pointer to the mutex to be unlocked.
 */
void mutex_unlock(mutex_t *mu);

#endif