# Core Code README
The "Core Code" encompasses all application specific code.
For boot sequence or linker documentation, see the [Vendor Documentation](../vendor/README.md).
For FreeRTOS documentation, see the [FreeRTOS Website](https://www.freertos.org/).

## Configuration
Located under `include/config/`, there are many configuration files used for the HAL, OS, and Driver layers.

### Boards
The boards folder, located under `include/config/boards`, gives developers an opportunity to easily test code on multiple boards.
To add to the boards list, first create the board and its pin-maps following the format from the existing boards.
Then, add the board to the list of boards under `include/config/pin_cfg.h`.

### Driver Layer
Serial, CAN bus, and ADC drivers are currently part of the OS implementation.
Therefore, the system must know which interfaces are available at compile time.

> [!WARNING]
> Do not remove lines from this file.

To choose which interfaces are available, change the value of the definition.

Example for ENABLED:
```c
#define configUSE_SERIAL1 1
```

Example for DISABLED:
```c
#define configUSE_SERIAL1 0
```

## OS
Files part of the OS include the FreeRTOS Scheduler, `C` files under the OS folder in `src`, and the HAL.
Additionally, the `FreeRTOSConfig.h` file under `include/config`.

Any file falling in the OS category should not be modified.
These files allow the base system to function correctly.
Modifying them could introduce unintended errors or faults that are difficult to diagnose.

## Drivers
Drivers are the middleware.
They exist to provide access control between FreeRTOS tasks and the STM32 hardware.
The QSET Arm Control Board utilizes UART, CAN, and USB Drivers.

### UART
The UART driver, titled `Serial`, acts as a middleware allowing UART ports to be used as files.
This enables `C` functions such as `printf` or `fprintf`.
Reading from a UART port using a file wrapper is currently not possible.
To read from a UART port, a task must attach a stream buffer to the port.
That task can then read characters from the FreeRTOS stream buffer.

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


