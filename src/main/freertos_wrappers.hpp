#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>

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
    void lock()
    {
        if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
        {
            // Handle error: failed to acquire mutex
            // For example, you could throw an exception or log an error message
        }
    }
    void unlock()
    {
        xSemaphoreGive(mutex);
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

template <std::size_t StackSizeBytes, typename Context = void>
class StaticTask;

template <std::size_t StackSizeBytes>
class StaticTask<StackSizeBytes, void>
{
public:
    using TaskEntry = void (*)();

    static_assert(StackSizeBytes > 0, "StackSizeBytes must be > 0");
    StaticTask() = default;

    BaseType_t create(const char *name, UBaseType_t priority, TaskEntry entry)
    {
        if (task_handle_ != nullptr)
        {
            return pdPASS;
        }
        if (entry == nullptr)
        {
            return pdFAIL;
        }

        task_entry_ = entry;
        task_handle_ = xTaskCreateStatic(task_trampoline,
                                         name,
                                         static_cast<uint32_t>(StackSizeBytes),
                                         this,
                                         priority,
                                         stack_buffer_.data(),
                                         &task_buffer_);
        return (task_handle_ != nullptr) ? pdPASS : pdFAIL;
    }

    bool is_created() const
    {
        return task_handle_ != nullptr;
    }

    TaskHandle_t native_handle() const
    {
        return task_handle_;
    }

    StaticTask(const StaticTask &) = delete;
    StaticTask &operator=(const StaticTask &) = delete;
    StaticTask(StaticTask &&) = delete;
    StaticTask &operator=(StaticTask &&) = delete;

private:
    static void task_trampoline(void *arg)
    {
        auto *self = static_cast<StaticTask *>(arg);
        self->task_entry_();
    }

    std::array<StackType_t, StackSizeBytes> stack_buffer_ = {};
    StaticTask_t task_buffer_ = {};
    TaskEntry task_entry_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
};

template <std::size_t StackSizeBytes, typename Context>
class StaticTask
{
public:
    using TaskEntry = void (*)(Context &);

    static_assert(StackSizeBytes > 0, "StackSizeBytes must be > 0");
    StaticTask() = default;

    BaseType_t create(const char *name, UBaseType_t priority, TaskEntry entry, Context &context)
    {
        if (task_handle_ != nullptr)
        {
            return pdPASS;
        }
        if (entry == nullptr)
        {
            return pdFAIL;
        }

        task_entry_ = entry;
        task_context_ = &context;
        task_handle_ = xTaskCreateStatic(task_trampoline,
                                         name,
                                         static_cast<uint32_t>(StackSizeBytes),
                                         this,
                                         priority,
                                         stack_buffer_.data(),
                                         &task_buffer_);
        assert(task_handle_);
        return (task_handle_ != nullptr) ? pdPASS : pdFAIL;
    }
    StaticTask(const StaticTask &) = delete;
    StaticTask &operator=(const StaticTask &) = delete;
    StaticTask(StaticTask &&) = delete;
    StaticTask &operator=(StaticTask &&) = delete;

private:
    static void task_trampoline(void *arg)
    {
        auto *self = static_cast<StaticTask *>(arg);
        self->task_entry_(*self->task_context_);
    }

    std::array<StackType_t, StackSizeBytes> stack_buffer_ = {};
    StaticTask_t task_buffer_ = {};
    Context *task_context_ = nullptr;
    TaskEntry task_entry_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
};

template <typename ItemType, std::size_t QueueLength>
class StaticQueue
{
public:
    static_assert(QueueLength > 0, "QueueLength must be > 0");
    static_assert(std::is_trivially_copyable_v<ItemType>,
                  "StaticQueue ItemType must be trivially copyable");
    StaticQueue()
    {
        queue_handle_ = xQueueCreateStatic(QueueLength,
                                           sizeof(ItemType),
                                           queue_storage_.data(),
                                           &queue_buffer_);
        assert(queue_handle_);
    }

    BaseType_t send(const ItemType &item, TickType_t ticks_to_wait = portMAX_DELAY) const
    {
        return xQueueSend(queue_handle_, &item, ticks_to_wait);
    }

    BaseType_t receive(ItemType &item, TickType_t ticks_to_wait = portMAX_DELAY) const
    {
        return xQueueReceive(queue_handle_, &item, ticks_to_wait);
    }

    UBaseType_t messages_waiting() const
    {
        return uxQueueMessagesWaiting(queue_handle_);
    }

    StaticQueue(const StaticQueue &) = delete;
    StaticQueue &operator=(const StaticQueue &) = delete;
    StaticQueue(StaticQueue &&) = delete;
    StaticQueue &operator=(StaticQueue &&) = delete;

private:
    alignas(ItemType) std::array<std::uint8_t, sizeof(ItemType) * QueueLength> queue_storage_ = {};
    StaticQueue_t queue_buffer_ = {};
    QueueHandle_t queue_handle_ = nullptr;
};
