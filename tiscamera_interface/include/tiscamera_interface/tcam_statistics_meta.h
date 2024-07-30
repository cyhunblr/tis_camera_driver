#ifndef TCAM_STATISTICS_META_H
#define TCAM_STATISTICS_META_H

#include <gst/gst.h>

typedef struct _GstMetaTcamStatistics TcamStatisticsMeta;

struct _GstMetaTcamStatistics
{
    GstMeta meta;

    GstStructure* structure;
};

#endif /* TCAM_STATISTICS_META_H */
