/*
  mcu.c - peripherals emulator code for simulator MCU

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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

#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <sys/timerfd.h>
#include <sys/select.h>

#include "mcu.h"
#include "simulator.h"
#include "hal.h"

static volatile bool irq_enable = false;
static bool booted = false;
static interrupt_handler isr[IRQ_N_HANDLERS];

mcu_uart_t uart;
mcu_timer_t timer[MCU_N_TIMERS];
mcu_timer_t systick_timer;
gpio_port_t gpio[MCU_N_GPIO];

static void default_handler (void)
{
    // NOOP - TODO: change to blocking loop?
}

void mcu_reset (void)
{
    uint_fast8_t i;

    irq_enable = false;

    for(i = 0; i < IRQ_N_HANDLERS; i++)
        isr[i] = default_handler;

    memset(&uart, 0, sizeof(mcu_uart_t));
    memset(&timer, 0, sizeof(mcu_timer_t) * MCU_N_TIMERS);
    for (i = 0; i < MCU_N_TIMERS; i++) 
      timer[i].fd = timerfd_create(CLOCK_REALTIME, TFD_CLOEXEC);
    memset(&systick_timer, 0, sizeof(mcu_timer_t));
    systick_timer.fd = timerfd_create(CLOCK_REALTIME, TFD_CLOEXEC);
    memset(&gpio, 0, sizeof(gpio_port_t) * MCU_N_GPIO);

    irq_enable = true;
    booted = true;
}

void mcu_register_irq_handler (interrupt_handler handler, irq_num_t irq_num)
{
    isr[irq_num] = handler == NULL ? default_handler : handler;
}

void mcu_enable_interrupts (void)
{
    irq_enable = true;
}

void mcu_disable_interrupts (void)
{
    irq_enable = false;
}

void mcu_timer_set(mcu_timer_t *timer, uint32_t load)
{
    timer->it.tv_sec = load * (1.0e0 / F_CPU);
    timer->it.tv_nsec = load * (1.0e9 / F_CPU);
}

void mcu_timer_start(mcu_timer_t *timer)
{
    struct itimerspec new_value = {
        .it_interval = timer->it,
        .it_value = timer->it
    };
    timerfd_settime(timer->fd, 0, &new_value, NULL);
    timer->enable = true;
}

void mcu_timer_stop(mcu_timer_t *timer)
{
    struct itimerspec new_value = {
        .it_interval = timer->it,
        .it_value = {.tv_sec = 0, .tv_nsec = 0}
    };
    timerfd_settime(timer->fd, 0, &new_value, NULL);
    timer->enable = true;
}

void mcu_master_clock (void)
{
    uint_fast8_t i;

    if(!booted)
        return;

    int max_fd;
    fd_set read_fds;
    FD_ZERO(&read_fds);
    max_fd = 0;

    for (i = 0; i < MCU_N_TIMERS; i++) {
        FD_SET(timer[i].fd, &read_fds);
        if (max_fd < timer[i].fd) max_fd = timer[i].fd;
    }
    FD_SET(systick_timer.fd, &read_fds);
    if (max_fd < systick_timer.fd) systick_timer.fd;

    struct timespec timeout = {.tv_sec = 1, .tv_nsec = 0};
    if (pselect(max_fd + 1, &read_fds, NULL, NULL, &timeout, NULL) < 0) 
        perror("Error from pselect()");

    for (i = 0; i < MCU_N_TIMERS; i++) {
        if (FD_ISSET(timer[i].fd, &read_fds)) {
            uint64_t count;
            if (read(timer[i].fd,&count,sizeof(count)) < 0)
                perror("Error from read()");
            if(timer[i].irq_enable && irq_enable)
                isr[Timer0_IRQ + i]();
        }
    }

    if (FD_ISSET(systick_timer.fd, &read_fds)) {
        uint64_t count;
        if (read(systick_timer.fd,&count,sizeof(count)) < 0)
            perror("Error from read()");
        if(systick_timer.irq_enable && irq_enable)
            isr[Systick_IRQ]();
    }

    for(i = 0; i < MCU_N_GPIO; i++) {
        if(gpio[i].irq_state.value & gpio[i].irq_mask.value)
            isr[GPIO0_IRQ + i]();
    }
}

void mcu_gpio_set (gpio_port_t *port, uint8_t pins, uint8_t mask)
{
    port->state.value = (port->state.value & ~mask) | (pins & mask);
}

uint8_t mcu_gpio_get (gpio_port_t *port, uint8_t mask)
{
    return port->state.value & mask;
}

void mcu_gpio_toggle_in (gpio_port_t *port, uint8_t pins)
{
    mcu_gpio_in(port, (port->state.value & pins) ^ pins, pins);
}

// Set input pin, trigger interrupt if enabled
void mcu_gpio_in (gpio_port_t *port, uint8_t pins, uint8_t mask)
{
    pins &= mask;

    uint8_t changed = (port->state.value & mask) ^ pins, bitflag = 1;

    do {
        if(changed & bitflag) {
            if(port->state.value & bitflag) {
                if(port->falling.value & bitflag)
                    port->irq_state.value |= bitflag;
            } else {
                if(port->rising.value & bitflag)
                    port->irq_state.value |= bitflag;
            }
            changed &= ~bitflag;
        }
        bitflag <<= 1;
    } while(changed);

    port->state.value = (port->state.value & ~mask) | pins;
}

// TODO: move to mcu_master_clock() above
void simulate_serial (void)
{
    if(!booted)
        return;

    if(uart.tx_flag) {
        sim.putchar(uart.tx_data);
        uart.tx_flag = 0;
    }

    if((uart.tx_irq = uart.tx_irq_enable))
        isr[UART_IRQ]();

    if(uart.rx_irq_enable && !uart.rx_irq && hal.stream.get_rx_buffer_free() > 100) {
        uint8_t char_in = sim.getchar();
        if (char_in) {
            uart.rx_data = char_in;
            uart.rx_irq = 1;
            isr[UART_IRQ]();
        }
    }
}

/*
Local Variables:
c-basic-offset: 4
indent-tabs-mode: nil
End:
*/
