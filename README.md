# dual_controller

## Debugging 

https://github.com/texane/stlink

```
// Run the st-util debugger utility
$ st-util &

// Start GDB
$ arm-none-eabi-gdb build/ch.elf

// Connect to the target
(gdb) target extended-remote localhost:4242
(gdb) tar ext :4242

// Load and run
(gdb) load
(gdb) continue
```

GDB shortcuts: http://users.ece.utexas.edu/~adnan/gdb-refcard.pdf


```
$ hbmqtt
$ hbmqtt_sub --url mqtt://localhost -t /test
```
