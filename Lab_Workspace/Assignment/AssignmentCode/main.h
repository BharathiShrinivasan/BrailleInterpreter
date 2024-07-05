

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"


extern bool UI_Initialize  (void);
extern void UI_Thread      (void const *arg);

extern osThreadId ui_thread_id;
extern osThreadId can_thread_id;
