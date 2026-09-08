#ifndef TASK_H
#define TASK_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

class task {
	public:
		task(uint32_t loop_time,
			void (*func)(uint32_t),
			uint32_t stack_depth,
			UBaseType_t priority,
			BaseType_t core_id);

		task(uint32_t loop_time,
			void (*func_void)(),
			uint32_t stack_depth,
			UBaseType_t priority,
			BaseType_t core_id);

		task(uint32_t loop_time,
			void (*func)(uint32_t));

		task(uint32_t loop_time,
			void (*func_void)());

	public:
		void start();

	private:
		enum class type_t {
			rtos,
			esp_timer
		};

		type_t type;
		uint32_t loop_time;
		void (*task_func)(uint32_t);
		void (*task_func_void)();
		uint32_t stack_depth = 0;
		UBaseType_t priority = 0;
		BaseType_t core_id = 0;
		esp_timer_handle_t xTimer = nullptr;

	private:
		static void rtos_entry(void *arg);
		static void timer_callback(void *arg);
};

#endif
