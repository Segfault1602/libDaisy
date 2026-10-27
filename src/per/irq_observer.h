#pragma once
#ifndef DSY_IRQ_OBSERVER_H
#define DSY_IRQ_OBSERVER_H

#include <stdint.h>

namespace daisy
{
/** Stable identities for the observed software interrupt handlers. */
enum class IrqId : uint8_t
{
    SAI1_DMA_A  = 0, /**< DMA1 stream 0. */
    SAI1_DMA_B  = 1, /**< DMA1 stream 1. */
    SPI1_GLOBAL = 2, /**< SPI1 peripheral IRQ. */
    SPI_TX_DMA  = 3, /**< DMA2 stream 3, shared by DMA-capable SPI handles. */
};

enum class IrqPhase : uint8_t
{
    ENTRY = 0,
    EXIT  = 1,
};

/** Normalized raw DMA status bits, independent of interrupt enables. */
enum DmaIrqFlags : uint32_t
{
    DMA_IRQ_HALF_TRANSFER     = 1U << 0,
    DMA_IRQ_TRANSFER_COMPLETE = 1U << 1,
    DMA_IRQ_TRANSFER_ERROR    = 1U << 2,
    DMA_IRQ_DIRECT_MODE_ERROR = 1U << 3,
    DMA_IRQ_FIFO_ERROR        = 1U << 4,
};

enum class IrqDmaDirection : uint8_t
{
    NONE             = 0,
    PERIPH_TO_MEMORY = 1,
    MEMORY_TO_PERIPH = 2,
    MEMORY_TO_MEMORY = 3,
};

/** A read-only DMA position/status sample.
 *  remaining is the hardware NDTR; total is the programmed transfer length in
 *  DMA elements, not bytes or stereo frames. SAI uses 32-bit words and SPI uses
 *  bytes. A circular SAI half contains total / 2 elements.
 *  direction and enabled come from the hardware stream CR. NONE denotes an
 *  absent sample; total == 0 denotes an unknown/unstarted transfer.
 *  Register reads are sequential, not an atomic sample of DMA progress.
 */
struct IrqDmaSnapshot
{
    uint32_t        flags     = 0;
    uint32_t        remaining = 0;
    uint32_t        total     = 0;
    IrqDmaDirection direction = IrqDmaDirection::NONE;
    bool            enabled   = false;
};

/** One synchronous observation immediately before or after HAL dispatch.
 *  ENTRY captures raw flags before HAL can clear them; EXIT is a fresh sample,
 *  not a repeat of ENTRY. Save ENTRY if both samples are needed.
 *
 *  peripheral is one-based (SAI1/SPI1 == 1), or zero if the shared SPI DMA has
 *  no active owner. The shared DMA owner may change during HAL dispatch.
 *  dma describes the handled SAI block or SPI TX stream. On SPI1 events it is
 *  absent unless SPI1 owns DMA2 stream 3; on SPI_TX_DMA it samples that physical
 *  stream even without an owner (total is then zero).
 *
 *  output_dma is present on both SAI events when exactly one SAI1 block is
 *  configured to transmit, including on the receive handler that runs the audio
 *  callback. It samples that transmitter, not the handled receive stream.
 *  On Daisy Seed rev4, A transmits and B receives. SPI events leave it absent.
 *
 *  peripheral_status is the raw SPI1 SR on SPI1 events, otherwise zero.
 *  Flags are observations, not proof of callback count, lost halves, event-to-
 *  entry latency or absolute deadline compliance. DMA progress is not peripheral
 *  completion. C-level observation excludes exception entry/return and some
 *  instrumentation overhead.
 */
struct IrqEvent
{
    IrqId          irq;
    IrqPhase       phase;
    uint8_t        peripheral = 0;
    IrqDmaSnapshot dma;
    IrqDmaSnapshot output_dma;
    uint32_t       peripheral_status = 0;
};

/** Runs in IRQ context; event is valid only for the duration of this call.
 *  Keep it bounded and nonblocking: no allocation, floating-point processing,
 *  peripheral reconfiguration or HAL calls. Higher-priority IRQs may preempt
 *  observers, so shared callback state must support nesting.
 *  Entry/exit timer reads and cumulative audio ticks can be used to subtract
 *  nested audio time from a SPI handler span. No timer or allocation is supplied.
 */
typedef void (*IrqObserverCallback)(void* context, const IrqEvent& event);

} // namespace daisy

#endif
