#pragma once
#ifndef DSY_SPI_DMA_TX_JOB_H
#define DSY_SPI_DMA_TX_JOB_H

#include "per/spi.h"

namespace daisy
{
namespace detail
{
    class SpiDmaTxJob
    {
      public:
        struct Completion
        {
            SpiHandle::EndCallbackFunctionPtr callback = nullptr;
            void*                             context  = nullptr;
            SpiHandle::Result                 result   = SpiHandle::Result::ERR;

            void Publish() const
            {
                if(callback)
                    callback(context, result);
            }
        };

        bool Start(uint8_t*                          buffer,
                   size_t                            size,
                   bool                              ready,
                   SpiHandle::EndCallbackFunctionPtr callback,
                   void*                             context)
        {
            if(!buffer || size == 0 || size > UINT16_MAX || !ready || Active()
               || state_ == State::FAILED)
                return false;

            callback_      = callback;
            context_       = context;
            pending_error_ = false;
            state_         = State::ACTIVE;
            return true;
        }

        bool Enabled() const { return state_ != State::DISABLED; }
        bool Failed() const { return state_ == State::FAILED; }
        bool Active() const
        { return state_ == State::ACTIVE || state_ == State::AWAITING_EOT; }

        void DmaError()
        {
            if(Active())
                pending_error_ = true;
        }

        bool DmaComplete()
        {
            if(state_ != State::ACTIVE || pending_error_)
                return false;
            state_ = State::AWAITING_EOT;
            return true;
        }

        bool AwaitingEot() const { return state_ == State::AWAITING_EOT; }
        bool ErrorPending() const { return pending_error_; }

        Completion Finish(SpiHandle::Result result)
        {
            Completion completion;
            if(!Active())
                return completion;

            if(pending_error_)
                result = SpiHandle::Result::ERR;
            completion.callback = callback_;
            completion.context  = context_;
            completion.result   = result;
            callback_           = nullptr;
            context_            = nullptr;
            state_
                = result == SpiHandle::Result::OK ? State::IDLE : State::FAILED;
            return completion;
        }

      private:
        enum class State
        {
            DISABLED,
            IDLE,
            ACTIVE,
            AWAITING_EOT,
            FAILED
        };

        State                             state_         = State::DISABLED;
        bool                              pending_error_ = false;
        SpiHandle::EndCallbackFunctionPtr callback_      = nullptr;
        void*                             context_       = nullptr;
    };
} // namespace detail
} // namespace daisy

#endif
