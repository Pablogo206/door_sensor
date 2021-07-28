       
       
       
 .bss
 .global next_edge
next_edge:
 .long 0
 .global debounce_counter
debounce_counter:
 .long 0
 .global debounce_max_count
debounce_max_count:
 .long 0
 .global edge_count
edge_count:
 .long 0
 .global edge_count_to_wake_up
edge_count_to_wake_up:
 .long 0
 .global io_number
io_number:
 .long 0
 .text
 .global entry
entry:
 move r3, io_number
 ld r3, r3, 0
 move r0, r3
 jumpr read_io_high, 16, ge
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14) + (16) - 1), (14)
 rsh r0, r0, r3
 jump read_done
read_io_high:
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14 + 16) + (2) - 1), (14 + 16)
 sub r3, r3, 16
 rsh r0, r0, r3
read_done:
 and r0, r0, 1
 move r3, next_edge
 ld r3, r3, 0
 add r3, r0, r3
 and r3, r3, 1
 jump changed, eq
 jump entry
 .global changed
changed:
 move r3, debounce_counter
 ld r2, r3, 0
 add r2, r2, 0
 jump edge_detected, eq
 sub r2, r2, 1
 st r2, r3, 0
 halt
 .global edge_detected
edge_detected:
 move r3, debounce_max_count
 move r2, debounce_counter
 ld r3, r3, 0
 st r3, r2, 0
 move r3, next_edge
 ld r2, r3, 0
 add r2, r2, 1
 and r2, r2, 1
 st r2, r3, 0
 move r3, edge_count
 ld r2, r3, 0
 add r2, r2, 1
 st r2, r3, 0
 move r3, edge_count_to_wake_up
 ld r3, r3, 0
 sub r3, r3, r2
 jump wake_up, eq
 halt
