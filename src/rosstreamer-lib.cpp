/*

rostreamer library source
Author: Ayman Saleh

*/

#include "rosstreamer-lib.h"
#include <stdio.h>
#include <stdlib.h>

struct RosStreamerCtx
{
    RosStreamerInitParams initParams;
};

RosStreamerCtx *RosStreamerCtxInit(RosStreamerInitParams *initParams)
{

    RosStreamerCtx *ctx = (RosStreamerCtx *)calloc(1, sizeof(RosStreamerCtx));
    ctx->initParams = *initParams;
    return ctx;
}

// RosStreamerProcess is meant as a processing function in the case of future
// needs to preprocess stream before a publisher publishes image msgs
RosStreamerOutput *RosStreamerProcess(RosStreamerCtx *ctx, unsigned char *data)
{

    RosStreamerOutput *out = (RosStreamerOutput *)calloc(1, sizeof(RosStreamerOutput));

    if (data != NULL)
    {
        //Processing logic goes here
    }

    return out;
}

void RosStreamerCtxDeInit(RosStreamerCtx *ctx)
{
    free(ctx);
}
