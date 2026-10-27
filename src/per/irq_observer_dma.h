#pragma once
#ifndef DSY_IRQ_OBSERVER_DMA_H
#define DSY_IRQ_OBSERVER_DMA_H

#include "per/irq_observer.h"
#include "stm32h7xx_hal.h"

namespace daisy
{
namespace detail
{
    inline IrqDmaSnapshot ReadDmaIrqSnapshot(DMA_Stream_TypeDef* stream,
                                             uint32_t            total)
    {
        struct
        {
            DMA_Stream_TypeDef* Instance;
        } handle = {stream};

        const uint32_t ht  = __HAL_DMA_GET_HT_FLAG_INDEX(&handle);
        const uint32_t tc  = __HAL_DMA_GET_TC_FLAG_INDEX(&handle);
        const uint32_t te  = __HAL_DMA_GET_TE_FLAG_INDEX(&handle);
        const uint32_t dme = __HAL_DMA_GET_DME_FLAG_INDEX(&handle);
        const uint32_t fe  = __HAL_DMA_GET_FE_FLAG_INDEX(&handle);
        const uint32_t raw
            = __HAL_DMA_GET_FLAG(&handle, ht | tc | te | dme | fe);
        IrqDmaSnapshot result;
        result.remaining  = stream->NDTR;
        const uint32_t cr = stream->CR;
        result.flags      = ((raw & ht) ? DMA_IRQ_HALF_TRANSFER : 0U)
                            | ((raw & tc) ? DMA_IRQ_TRANSFER_COMPLETE : 0U)
                            | ((raw & te) ? DMA_IRQ_TRANSFER_ERROR : 0U)
                            | ((raw & dme) ? DMA_IRQ_DIRECT_MODE_ERROR : 0U)
                            | ((raw & fe) ? DMA_IRQ_FIFO_ERROR : 0U);
        result.total      = total;
        result.enabled    = (cr & DMA_SxCR_EN) != 0;
        switch(cr & DMA_SxCR_DIR)
        {
            case DMA_PERIPH_TO_MEMORY:
                result.direction = IrqDmaDirection::PERIPH_TO_MEMORY;
                break;
            case DMA_MEMORY_TO_PERIPH:
                result.direction = IrqDmaDirection::MEMORY_TO_PERIPH;
                break;
            case DMA_MEMORY_TO_MEMORY:
                result.direction = IrqDmaDirection::MEMORY_TO_MEMORY;
                break;
            default: break;
        }
        return result;
    }
} // namespace detail
} // namespace daisy

#endif
