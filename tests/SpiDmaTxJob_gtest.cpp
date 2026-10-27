#include "util/spi_dma_tx_job.h"
#include <gtest/gtest.h>

using namespace daisy;

namespace
{
struct Publication
{
    unsigned          calls  = 0;
    SpiHandle::Result result = SpiHandle::Result::ERR;

    static void Record(void* context, SpiHandle::Result result)
    {
        auto& self  = *static_cast<Publication*>(context);
        self.result = result;
        ++self.calls;
    }
};
} // namespace

TEST(SpiDmaTxJob, RejectionsDoNotAcquireContextOrPublish)
{
    detail::SpiDmaTxJob job;
    Publication         rejected, accepted;
    uint8_t             data[1] = {};
    EXPECT_FALSE(job.Start(nullptr, 1, true, Publication::Record, &rejected));
    EXPECT_FALSE(job.Start(data, 0, true, Publication::Record, &rejected));
    EXPECT_FALSE(job.Start(data, 65536, true, Publication::Record, &rejected));
    EXPECT_FALSE(job.Start(data, 1, false, Publication::Record, &rejected));
    EXPECT_FALSE(job.Enabled());
    ASSERT_TRUE(job.Start(data, 1, true, Publication::Record, &accepted));
    EXPECT_FALSE(job.Start(data, 1, true, Publication::Record, &rejected));
    EXPECT_FALSE(job.Start(data, 1, false, Publication::Record, &rejected));
    job.Finish(SpiHandle::Result::OK).Publish();
    EXPECT_EQ(accepted.calls, 1U);
    EXPECT_EQ(rejected.calls, 0U);
}

TEST(SpiDmaTxJob, DmaCompletionOnlyArmsEotAndDuplicateInterruptsDoNotPublish)
{
    detail::SpiDmaTxJob job;
    Publication         publication;
    uint8_t             data[1] = {};
    ASSERT_TRUE(
        job.Start(data, 65535, true, Publication::Record, &publication));
    EXPECT_FALSE(job.AwaitingEot());
    EXPECT_TRUE(job.DmaComplete());
    EXPECT_TRUE(job.Active());
    EXPECT_TRUE(job.AwaitingEot());
    EXPECT_FALSE(job.DmaComplete());
    EXPECT_EQ(publication.calls, 0U);
    job.Finish(SpiHandle::Result::OK).Publish();
    job.Finish(SpiHandle::Result::OK).Publish();
    job.Finish(SpiHandle::Result::ERR).Publish();
    EXPECT_EQ(publication.calls, 1U);
    EXPECT_EQ(publication.result, SpiHandle::Result::OK);
    EXPECT_FALSE(job.Active());
}

TEST(SpiDmaTxJob, ImmediateStartFailurePublishesOnceAndPermanentlyRejects)
{
    detail::SpiDmaTxJob job;
    Publication         publication;
    uint8_t             data[1] = {};
    ASSERT_TRUE(job.Start(data, 1, true, Publication::Record, &publication));
    job.Finish(SpiHandle::Result::ERR).Publish();
    EXPECT_EQ(publication.calls, 1U);
    EXPECT_EQ(publication.result, SpiHandle::Result::ERR);
    EXPECT_FALSE(job.DmaComplete());
    job.Finish(SpiHandle::Result::OK).Publish();
    job.Finish(SpiHandle::Result::ERR).Publish();
    EXPECT_FALSE(job.Start(data, 1, true, Publication::Record, &publication));
    EXPECT_EQ(publication.calls, 1U);
}

TEST(SpiDmaTxJob, DmaErrorWinsOverSimultaneousOrPreviouslyArmedCompletion)
{
    for(bool complete_first : {false, true})
    {
        detail::SpiDmaTxJob job;
        Publication         publication;
        uint8_t             data[1] = {};
        ASSERT_TRUE(
            job.Start(data, 1, true, Publication::Record, &publication));
        if(complete_first)
        {
            EXPECT_TRUE(job.DmaComplete());
        }
        job.DmaError();
        EXPECT_FALSE(job.DmaComplete());
        EXPECT_EQ(publication.calls, 0U);
        job.Finish(SpiHandle::Result::OK).Publish();
        job.Finish(SpiHandle::Result::ERR).Publish();
        EXPECT_EQ(publication.calls, 1U);
        EXPECT_EQ(publication.result, SpiHandle::Result::ERR);
        EXPECT_FALSE(
            job.Start(data, 1, true, Publication::Record, &publication));
    }
}

TEST(SpiDmaTxJob, ACompletedJobsContextCannotLeakIntoTheNextJob)
{
    detail::SpiDmaTxJob job;
    Publication         first, second;
    uint8_t             data[1] = {};
    ASSERT_TRUE(job.Start(data, 1, true, Publication::Record, &first));
    ASSERT_TRUE(job.DmaComplete());
    job.Finish(SpiHandle::Result::OK).Publish();
    ASSERT_TRUE(job.Start(data, 17, true, Publication::Record, &second));
    ASSERT_TRUE(job.DmaComplete());
    job.Finish(SpiHandle::Result::OK).Publish();
    job.Finish(SpiHandle::Result::ERR).Publish();
    EXPECT_EQ(first.calls, 1U);
    EXPECT_EQ(second.calls, 1U);
    EXPECT_EQ(second.result, SpiHandle::Result::OK);
}
