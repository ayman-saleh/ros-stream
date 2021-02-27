/*

rostreamer library header
Author: Ayman Saleh

*/

#ifndef __ROS_STREAMER_LIB_H_
#define __ROS_STREAMER_LIB_H_

#define MAX_TOPIC_NAME_SIZE 128

typedef struct RosStreamerCtx RosStreamerCtx;

// Init parameter structure as input for RosStreamerCtx
typedef struct
{
    // ROS topic name
    char topic_name[MAX_TOPIC_NAME_SIZE];

} RosStreamerInitParams;

// Output data retured after processing
typedef struct
{
    int publish_success;
} RosStreamerOutput;

// Init library ctx
RosStreamerCtx *RosStreamerCtxInit(RosStreamerInitParams *init_params);

// Dequeue processed output
RosStreamerOutput *RosStreamerProcess(RosStreamerCtx *ctx, unsigned char *data);

// Deinit library context
void RosStreamerCtxDeInit(RosStreamerCtx *ctx);

#endif
