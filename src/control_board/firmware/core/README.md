# Core Code README
The "Core Code" encompasses all application specific code.
For boot sequence or linker documentation, see the [Vendor Documentation](../vendor/README.md).
For FreeRTOS documentation, see the [FreeRTOS Website](https://www.freertos.org/).

## Configuration
Located under `include/config/`, there are many configuration files used for the HAL, OS, and pin-maps.

### Boards
The boards folder, located under `include/config/boards`, gives developers an opportunity to easily test code on multiple boards.
To add to the boards list, first create the board and its pin-maps following the format from the existing boards.
Then, add the board to the list of boards under `include/config/pin_cfg.h`.

### NVIC
Contains the Nested Vector interrupt Controller (NVIC) configuration.
Messing with this file can cause FreeRTOS stalls and hard faults.

## OS
Files part of the OS include the FreeRTOS Scheduler, `C` files under the OS folder in `src`, and the HAL.
Additionally, the `FreeRTOSConfig.h` file under `include/config`.

Any file falling in the OS category should not be modified.
These files allow the base system to function correctly.
Modifying them could introduce unintended errors or faults that are difficult to diagnose.

### StdIO
To configure the Standard Input/Output port, use the function `register_stdio` provided by `syscalls.h`.
This function should be called once by the main function.
If not initialized, IO calls such as `printf` will return an error.

## Drivers
Drivers are the middleware.
They exist to provide access control between FreeRTOS tasks and the STM32 hardware.
The is project contains UART, CAN, and USB Drivers.
The drivers folder should contain only STM32 drivers that interact with the STM32 HAL.
Other drivers, such as device drivers, should be placed under [interfaces](#interfaces).

### UART
The UART driver, titled `Serial`, acts as a middleware allowing UART ports to be shared between FreeRTOS Tasks.
The Serial layer also enables `C` functions such as `printf`.
To use a Serial port for printf, initialize it in `main.c`, then initialize it as the standard IO port via `register_stdio`.

Writing to UART is a blocking operation.
Writing can be performed through the Serial Driver layer, which contains an access mutex.
Therefore, Serial ports can be shared between multiple FreeRTOS Tasks.

To read from a UART port, create a [FreeRTOS Stream Buffer](https://freertos.org/Documentation/02-Kernel/04-API-references/08-Stream-buffers/00-RTOS-stream-buffer-API).
The buffer should be created statically, and set up for whatever application specific purpose is needed.
To attach the buffer to the Serial port, call `serial_attach`.
Only one FreeRTOS task may attach a read buffer at the same time.
Read buffers are filled on UART interrupts, which are enabled automatically when a buffer is attached, and disabled automatically when a buffer is detached.

> **Update:** *(Jan 12, 2026)*
> The Serial layer was changed from being globally accessible to a private implementation.
> This is to reduce the risk of code errors and interfering writes for devices that operate over UART.
> The CAN Layer has not yet had this change applied.

### CAN
The CAN Bus driver allows tasks to register `CanMailbox_t` structures with it.
A mailbox is tailored to a specific CAN ID.
When a CAN Bus interrupt occurs, the message is forwarded to the CAN Task, which then deposits messages into mailboxes.
Tasks can read from the mailbox at any time, or block waiting for an RX event with `can_mailbox_wait`.
Tasks may choose to block indefinitely waiting for an RX event.
If blocking waiting for an RX event, the task will be woken up once the CAN task has finished.
The CAN task can register a maximum of `MAX_MAILBOXES`.
Adjust this define statement to enable more mailboxes.
This will use additional memory.

### USB
The USB Drivers used in this project are from the libusb_stm32 library.
Most of the USB Driver and HAL are directly taken from the USB CDC Demo project.
Only minor modifications have been made to the library in this project.
See [this GitHub repository for more information.](https://github.com/dmitrystu/libusb_stm32)

## User Code

### Protocols
The protocols folder has been set aside for application specific drivers and protocols.
This should not be confused with the `drivers` folder, which is used for system level drivers.

### Tasks
All user tasks should be placed within the `tasks` folder.

All tasks should follow the same generic base structure:

```c
typedef struct {
    TaskHandle_t tsk_hndl;
    StaticTask_t tsk_buf;
    StackType_t tsk_stack[STACK_SIZE];
    ...
} myTask;


void *myTask_init(myTask *, ...){
    ...
    xTaskCreateStatic(...);
    ...
}
```

This structure allows all task data and runtime nonsense to be statically initialized in main.
