/*
  simulator.c - functions to simulate how the buffer is emptied and the
                stepper interrupt is called

  Part of Grbl Simulator

  Copyright (c) 2012-2014 Jens Geisler
  Copyright (c) 2014-2015 Adam Shelly

  2020 - modified for grblHAL by Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include "simulator.h"
#include "eeprom.h"
#include "mcu.h"
#include "driver.h"

#include "grbl.h"

void sim_nop (void)
{
}

sim_vars_t sim = {
    .on_init = sim_nop,
    .on_tick = sim_nop,
    .on_byte = sim_nop,
    .on_shutdown = sim_nop
};

// Setup 
void init_simulator (void)
{
    sim.baud_ticks = F_CPU / 115200;

    sim.on_init();
}

// Shutdown simulator - call exit hooks, save eeprom is taken care of in atexit handler
void shutdown_simulator (void)
{
    sim.on_shutdown();
}

// Runs the hardware simulator at the desired rate until sim.exit is set
void sim_loop (void)
{
    while (sim.exit != exit_OK  ) { //don't quit until idle
      // do low level hardware
      mcu_master_clock();

      // do app-specific per-tick processing
      sim.on_tick();

      // do app-specific per-byte processing
      sim.on_byte();
    }
}

// Print serial output to args.serial_out_file
void sim_serial_out (uint8_t data)
{
    static uint8_t buf[128] = {0};
    static uint8_t len = 0;
    static bool continuation = 0;

    buf[len++] = data;
    // print when we get to newline or run out of buffer
    if(data == '\n' || data == '\r' || len >= 127) {
        if (args.comment_char && !continuation)
            fprintf(args.serial_out_file, "%c ", args.comment_char);
        buf[len] = '\0';
        fprintf(args.serial_out_file, "%s", buf);
        // don't print comment on next line if we are just printing to avoid buffer overflow
        continuation = (len >= 128); 
        len = 0;
    }
}

// Print serial output to sim.socket_fd stream
void sim_socket_out (uint8_t data)
{
    static uint8_t buf[128] = {0};
    static uint8_t len = 0;
    static bool continuation = 0;

    buf[len++] = data;
    // print when we get to newline or run out of buffer
    if(data == '\n' || data == '\r' || len >= 127) {        
        if (write(fileno(args.serial_out_file), buf, len) < 0)
            perror("Error in write(socket_fd");
        if (sim.socket_fd) {
            if(write(sim.socket_fd, buf, len) < 0)
                perror("Error in write(socket_fd");
        }
        // don't print comment on next line if we are just printing to avoid buffer overflow
        continuation = (len >= 128); 
        len = 0;
    }
}

/*
Local Variables:
c-basic-offset: 4
indent-tabs-mode: nil
End:
*/
