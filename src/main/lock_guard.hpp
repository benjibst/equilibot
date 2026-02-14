#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class StaticMutex
{
public:
    StaticMutex()
    {
        mutex = xSemaphoreCreateMutexStatic(&mutex_buffer);
    }
    ~StaticMutex()
    {
        vSemaphoreDelete(mutex);
    }

    StaticMutex(const StaticMutex &) = delete;
    StaticMutex &operator=(const StaticMutex &) = delete;
    StaticMutex(StaticMutex &&) = delete;
    StaticMutex &operator=(StaticMutex &&) = delete;

    friend class LockGuard;

private:
    StaticSemaphore_t mutex_buffer;
    SemaphoreHandle_t mutex;
};
class LockGuard
{
public:
    explicit LockGuard(StaticMutex &mutex) : mutex(mutex)
    {
        if (xSemaphoreTake(mutex.mutex, portMAX_DELAY) != pdTRUE)
        {
            // Handle error: failed to acquire mutex
            // For example, you could throw an exception or log an error message
        }
    }

    ~LockGuard()
    {
        xSemaphoreGive(mutex.mutex);
    }

    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;
    LockGuard(LockGuard &&) = delete;
    LockGuard &operator=(LockGuard &&) = delete;

private:
    StaticMutex &mutex;
};
