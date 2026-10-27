#include "util/irq_observer.h"
#include <gtest/gtest.h>
#include <vector>

using namespace daisy;

namespace
{
void RecordEvent(void* context, const IrqEvent& event)
{ static_cast<std::vector<IrqEvent>*>(context)->push_back(event); }
} // namespace

TEST(IrqObserver, UnregisteredDispatchDoesNotCapture)
{
    detail::IrqObserver observer;
    int                 captures = 0;
    int                 calls    = 0;
    const auto          capture  = [&]
    {
        ++captures;
        return IrqEvent{};
    };
    observer.Observe(IrqId::SAI1_DMA_A, capture, [&] { ++calls; });
    std::vector<IrqEvent> events;
    observer.Set(RecordEvent, &events);
    observer.Set(nullptr, nullptr);
    observer.Observe(IrqId::SAI1_DMA_A, capture, [&] { ++calls; });
    EXPECT_EQ(calls, 2);
    EXPECT_EQ(captures, 0);
    EXPECT_TRUE(events.empty());
}

TEST(IrqObserver, CapturesFlagsBeforeDispatchAndFreshOutputPositionAtExit)
{
    detail::IrqObserver   observer;
    std::vector<IrqEvent> events;
    observer.Set(RecordEvent, &events);
    IrqEvent state{};
    state.peripheral    = 1;
    state.dma.flags     = DMA_IRQ_HALF_TRANSFER | DMA_IRQ_TRANSFER_COMPLETE
                          | DMA_IRQ_TRANSFER_ERROR;
    state.dma.direction = IrqDmaDirection::PERIPH_TO_MEMORY;
    state.output_dma.direction = IrqDmaDirection::MEMORY_TO_PERIPH;
    state.output_dma.remaining = state.output_dma.total = 256;
    state.output_dma.enabled                            = true;
    observer.Observe(
        IrqId::SAI1_DMA_B,
        [&] { return state; },
        [&]
        {
            ASSERT_EQ(events.size(), 1U);
            EXPECT_EQ(events[0].phase, IrqPhase::ENTRY);
            state.dma.flags            = 0;
            state.output_dma.remaining = 100;
        });
    ASSERT_EQ(events.size(), 2U);
    EXPECT_EQ(events[0].irq, IrqId::SAI1_DMA_B);
    EXPECT_EQ(events[1].irq, IrqId::SAI1_DMA_B);
    EXPECT_EQ(events[1].phase, IrqPhase::EXIT);
    EXPECT_EQ(events[0].dma.flags,
              DMA_IRQ_HALF_TRANSFER | DMA_IRQ_TRANSFER_COMPLETE
                  | DMA_IRQ_TRANSFER_ERROR);
    EXPECT_EQ(events[1].dma.flags, 0U);
    EXPECT_EQ(events[0].output_dma.remaining, 256U);
    EXPECT_EQ(events[1].output_dma.remaining, 100U);
    EXPECT_EQ(events[1].output_dma.total, 256U);
    EXPECT_EQ(events[1].output_dma.direction,
              IrqDmaDirection::MEMORY_TO_PERIPH);
}

TEST(IrqObserver, NestedAudioPreservesSpiEntryExitPair)
{
    detail::IrqObserver   spi, audio;
    std::vector<IrqEvent> events;
    spi.Set(RecordEvent, &events);
    audio.Set(RecordEvent, &events);
    const auto capture = [] { return IrqEvent{}; };
    spi.Observe(IrqId::SPI_TX_DMA,
                capture,
                [&] { audio.Observe(IrqId::SAI1_DMA_B, capture, [] {}); });
    ASSERT_EQ(events.size(), 4U);
    EXPECT_EQ(events[0].irq, IrqId::SPI_TX_DMA);
    EXPECT_EQ(events[0].phase, IrqPhase::ENTRY);
    EXPECT_EQ(events[1].irq, IrqId::SAI1_DMA_B);
    EXPECT_EQ(events[1].phase, IrqPhase::ENTRY);
    EXPECT_EQ(events[2].irq, IrqId::SAI1_DMA_B);
    EXPECT_EQ(events[2].phase, IrqPhase::EXIT);
    EXPECT_EQ(events[3].irq, IrqId::SPI_TX_DMA);
    EXPECT_EQ(events[3].phase, IrqPhase::EXIT);
}
