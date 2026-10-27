#pragma once
#ifndef DSY_UTIL_IRQ_OBSERVER_H
#define DSY_UTIL_IRQ_OBSERVER_H

#include "per/irq_observer.h"

namespace daisy
{
namespace detail
{
    class IrqObserver
    {
      public:
        void Set(IrqObserverCallback callback, void* context)
        {
            context_  = context;
            callback_ = callback;
        }

        template <typename Capture, typename Handler>
        void Observe(IrqId irq, Capture capture, Handler handler) const
        {
            if(!callback_)
            {
                handler();
                return;
            }

            IrqEvent event = capture();
            event.irq      = irq;
            event.phase    = IrqPhase::ENTRY;
            callback_(context_, event);
            handler();
            event       = capture();
            event.irq   = irq;
            event.phase = IrqPhase::EXIT;
            callback_(context_, event);
        }

      private:
        IrqObserverCallback callback_ = nullptr;
        void*               context_  = nullptr;
    };
} // namespace detail
} // namespace daisy

#endif
