# SPI

SPI stands for Serial Peripheral Interface, and is a way for devices to talk to one another.
The protocol is used with one device as the main device or the leader.
All other devices are followers.

There are four pins generally used:
- MOSI: Main Out Serial In
- MISO: Main In Serial Out
- SCLK: Serial Clock
- NSS: Serial Select (Chip Select)

## How to Use SPI

First create your SpiHandle object and a Config struct to configure it.
Set any configurations you want to change in the config object, then initialize the spi peripheral with it.

A typical Daisy Seed configuration might look like this

```cpp
// SpiHandle object and Spi Configuration object
SpiHandle spi_handle;
SpiHandle::Config spi_conf;

// Set some configurations
spi_conf.periph = SpiHandle::Peripheral::SPI_1;
spi_conf.mode = SpiHandle::Mode::MASTER;
spi_conf.direction = SpiHandle::Direction::TWO_LINES;
spi_conf.nss = SpiHandle::NSS::NSS_HARD_OUTPUT;
spi_conf.pin_config.sclk = Pin(PORTG, 11);
spi_conf.pin_config.miso = Pin(PORTB, 4);
spi_conf.pin_config.mosi = Pin(PORTB, 5);
spi_conf.pin_config.nss = Pin(PORTG, 10);

// Initialize the handle using our configuration
spi_handle.Init(spi_conf);
```

### Configuration

The default configuration sets the common defaults for SPI peripherals. However some peripherals may require changing these. Some other configurations you'll have to set every time you use SPI.

`spi_conf.peripheral`
The peripheral to be used. Must be specified by the user. For example `SpiHandle::Config::Peripheral::SPI_2`.
The Daisy has 6 different SPI peripherals available, each of which uses different pins. Refer to the pinout diagram for more details.

`spi_conf.mode`
Master or slave. The main device is in charge of the bus (sets the clock, and NSS), the other devices follow along. Most of the time the Daisy will be in charge of the bus.
- `SpiHandle::Config::Mode::MASTER`. The Daisy runs the bus.
- `SpiHandle::Config::Mode::SLAVE`. The Daisy follows another device.

`spi_conf.pin_config`
The pins to be used by this SPI peripheral. These will have to match the peripheral you choose. Must be specified by the user.

- `spi_conf.pin_config.miso`: Main In Serial Out: The main device reads from this pin, and the others write to it.
- `spi_conf.pin_config.mosi`: Main Out Serial In: The main device writes to this pin, and the others read from it.
- `spi_conf.pin_config.sclk`: Serial Clock. The main device outputs a clock signal on this pin.
- `spi_conf.pin_config.nss`: Serial Select. The main device uses this to indicate data is being sent. Usually active low.

If you're not using a pin (e.g. software NSS, or simplex communication) you can set it to `Pin()`.

`spi_conf.direction`
Which direction data will travel. Must be specified by the user.

- `SpiHandle::Config::Direction::TWO_LINES`. Data goes both ways. Each line only goes one direction. i.e. MOSI -> MISO and MISO <- MOSI. This is full duplex. 
- `SpiHandle::Config::Direction::TWO_LINES_TX_ONLY`. The Daisy will only send data. If Daisy's the main device: MOSI->. If the Daisy's a follower MISO->. This is simplex TX.
- `SpiHandle::Config::Direction::TWO_LINES_RX_ONLY`. The Daisy will only read data. If Daisy;s the main device MISO<-. If Daisy's a follower MOSI<-. This is simplex RX.
- `SpiHandle::Config::Direction::ONE_LINE`. Data goes both ways over one line. i.e. MOSI <-> MISO. This is half duplex.

`spi_conf.data_size`
How many bits in each transmission. Defaults to 8.
Must be in the range [4, 32] inclusive.

`spi_conf.clock_polarity`
Is the clock active low or high? 
(Determines if the "first" and "second" edges are rising or falling.)

- `SpiHandle::Config::ClockPolarity::HIGH`. The clock is active high.
- `SpiHandle::Config::ClockPolarity::LOW`. The clock is active low.

Defaults to `SpiHandle::Config::ClockPolarity::LOW`.

`spi_conf.clock_phase`
When is the data ready to be read?
The "first edge" is when we transition __to__ the clock polarity.
The "second edge" is when we transition __away from__ the clock polarity.

So if the polarity is low, the first edge is when the clock goes low.
Again, if the polarity is low, the second edge is when the clock goes high.

- `SpiHandle::Config::ClockPhase::ONE_EDGE` Data is read on the first edge.
- `SpiHandle::Config::ClockPhase::TWO_EDGE`. Data is read on the second edge.

Defaults to `SpiHandle::Config::ClockPhase::ONE_EDGE`.

`spi_conf.nss`
Serial select mode. Must be set by the user.

- `SpiHandle::Config::NSS::SOFT`. Serial select is handled in software. You can ignore the NSS pin.
- `SpiHandle::Config::NSS::HARD_INPUT`. The NSS pin is in use, and the Daisy is a follower.
- `SpiHandle::Config::NSS::HARD_OUTPUT`. The NSS pin is in use, and the Daisy is a leader.

`spi_conf.baud_prescaler`
Division of the default clock rate. 
The clock rate is always 25MHz. 
So with a prescaler of 4 for example, the final clock rate is 25MHz / 4 = 6.25MHz.

Defaults to `SpiHandle::Config::BaudPrescaler::PS_8`

## Blocking Transmit and Receive

Send / receive data in a blocking fashion. The code waits while the transmissions are taking place.

```cpp
// send 4 bytes
uint8_t buffer[4] = {0, 1, 2, 3};
spi_handle.BlockingTransmit(buffer, 4);
```

```cpp
// receive 4 bytes
uint8_t buffer[4];
spi_handle.BlockingReceive(buffer, 4);
```

```cpp
// send and receive 4 bytes
uint8_t tx_buffer[4] = {0, 1, 2, 3};
uint8_t rx_buffer[4];
spi_handle.BlockingTransmitAndReceive(tx_buffer, rx_buffer, 4);
```

## DMA Transmit and Receive

Send / receive data using DMA (Direct Memory Access). 
This allows the hardware to handle the transmission in the background while the code is doing other things.
i.e. it is non blocking.

You can also pass along a callback to be called when the transfer starts, another for when the transfer is over, and a pointer to some data to send those callbacks.

**Note:** Your buffer has to be in the DMA section of memory, as well as in a global scope.

```cpp
// buffer for sending data
uint8_t DMA_BUFFER_MEM_SECTION buffer[4];

// fill the buffer 0, 1, 2, 3
for(uint8_t i = 0; i < 4; i++)
{
    buffer[i] = i;
}

// transmit the data
spi_handle.DmaTransmit(buffer, 4, NULL, NULL, NULL);
```

```cpp
// receive 4 bytes. No callbacks or callback data.
uint8_t DMA_BUFFER_MEM_SECTION buffer[4];
spi_handle.DmaReceive(buffer, 4, NULL, NULL, NULL);
```

```cpp
// send and receive 4 bytes. No callbacks or callback data.
uint8_t DMA_BUFFER_MEM_SECTION tx_buffer[4];
uint8_t DMA_BUFFER_MEM_SECTION rx_buffer[4];

// fill the TX buffer 0, 1, 2, 3
for(uint8_t i = 0; i < 4; i++)
{
   tx_buffer[i] = i;
}

// transmit and receive
spi_handle.DmaTransmitAndReceive(tx_buffer, rx_buffer, 4, NULL, NULL, NULL);
```

### Single-client SPI1 TX admission

`TryDmaTransmit(buffer, byte_count, end_callback, persistent_context)` is an
opt-in, non-queuing path for SPI1 master, 8-bit, `TWO_LINES_TX_ONLY`, software NSS.
The existing DMA APIs retain their scheduler and callback behavior when this
mode is not used.

Contract:

- SPI1 is the only SPI DMA client, with one outstanding job. Only thread mode
  submits or performs blocking commands; commands must wait for the preceding
  DMA job's successful completion.
- PRIMASK, BASEPRI and FAULTMASK must be clear when submitting. Both SPI1 and
  DMA2 stream 3 NVIC lines must be enabled at the same encoded priority, below
  audio. Use a priority grouping that actually permits audio to preempt them.
  Configure priorities after SPI/system initialization, which sets defaults.
- For example, with audio at preemption priority 0 and all priority bits used
  for preemption:

  ```cpp
  HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 0);
  ```

- Buffer length is 1..65535 bytes; lengths are rejected rather than narrowed.
  The caller supplies DMA-accessible, correctly aligned/cache-managed memory,
  keeps CS/DC stable, and retains the source until SPI EOT, not DMA TX complete.
- Initialize the persistent callback context and the application's job ticket
  before calling. Do not pass an address of a transient stack-local job.
  Callbacks only publish flags/timestamps; they never chain transfers, operate
  GPIO, call LVGL, or run recovery.
- `ERR` with no callback means rejected admission: invalid parameters,
  context/configuration, busy/not-ready hardware, disabled/mismatched IRQs, or
  an existing terminal failure. No pointer is acquired and nothing is queued.
- An admitted HAL initialization/start failure calls `end_callback(context,
  ERR)` in thread mode before returning `ERR`. An admitted successful start
  returns `OK`, but its interrupt callback may happen before that return.
  Handle that race by preparing ownership before submission, not afterwards.
- DMA2 stream 3 completion only arms SPI EOT. DMA errors are latched and pend
  SPI1; normal/error interrupt callbacks are published from SPI1. Error wins
  over simultaneous DMA completion. Each admitted job publishes at most once.
- On admitted error, the job is terminal, both owned IRQs remain disabled, and
  the callback/context are detached before publication. No late completion can
  publish again or authorize reuse. `CheckError()` retains HAL-compatible
  SPI/DMA error bits. This is not a reusable abort or a guarantee the DMA engine
  is quiescent: retain source/context and enter a non-returning fault path.
- Missing completion needs an application foreground timer deadline. On timeout,
  disable the owned IRQs, retain the source/context, and do not return/reuse it.
  Do not call HAL abort/reinitialization from an IRQ or with SysTick masked.
- The first admission selects this mode until system reset. `Init` and legacy
  DMA calls on this handle then reject; do not start other SPI DMA clients.

Serialization and timing qualification:

Admission masks only SPI1 and DMA2 stream 3 NVIC lines, preserving their enable
states on rejection/success. DMA is restored before SPI1: a pending SPI1 fault
can disable both without the restore path subsequently re-enabling DMA.
An error arriving during admission's enable-state snapshot suppresses restoration
of either line, even if the snapshot observed an enabled line before the error.
Audio and SysTick remain unmasked. `HAL_DMA_Init` and `HAL_SPI_Transmit_DMA` run
only here, in thread mode; any HAL tick-timeout polling therefore has an
advancing SysTick. Readiness/stream-busy checks reject rather than spin.

The opt-in IRQ handlers perform fixed register/state operations, without HAL
dispatch, loops, global interrupt masks, initialization, polling, queue restart,
or callback chaining. Equal SPI/DMA NVIC priorities prevent mutual preemption;
the foreground masks both before accessing job ownership. Audio must not access
this SPI/job state. These are synchronization preconditions, not assumptions
that a host state-machine test can establish.

`SpiDmaTxJob_gtest` covers admission rejection, EOT ownership, immediate failure,
duplicate/late notification suppression, DMA error precedence and successive
context ownership; `IrqObserver_gtest` covers observer dispatch. Neither host
tests nor cross-compilation establish hardware timing or IRQ/vector coverage.
Actual interrupt duration, audio preemption, missing-completion/error injection,
CS/EOT behavior and masked-time measurements require integrated hardware
qualification. No hardware timing bound is claimed by this API.