# INAV Bridge
Handles communication with INAV over MSP

> [!WARNING]
> MSP library may fail to build due to `nullptr_t` being "undefined".
> If so, change it to `std::nullptr_t`. Do not push this change upstream.
> Unknown on why this error occurs. `nullptr_t` should be defined in C++14+.

