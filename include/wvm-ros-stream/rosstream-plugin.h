/*

rosstream plugin header
Author: Ayman Saleh

*/

#ifndef __ROSSTREAM_PLUGIN_H__
#define __ROSSTREAM_PLUGIN_H__

#include <gst/base/gstbasetransform.h>
#include <gst/video/video.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cuda.h>
#include <cuda_runtime.h>
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"
#include "gst-nvquery.h"
#include "gstnvdsmeta.h"

#include "rosstreamer-lib.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// Plugin details requrired for plugin_init for gst
#define PACKAGE "rosstream"
#define VERSION "1."
#define LICENSE "MIT"
#define DESCRIPTION "Ros Stream plugin for GST"
#define BINARY_PACKAGE "Ros Stream plugin"
#define URL "http://jpl.nasa.gov"

// Boilerplate code for gst plugin
G_BEGIN_DECLS

typedef struct _GstRosStream GstRosStream;
typedef struct _GstRosStreamClass GstRosStreamClass;

#define GST_TYPE_ROSSTREAM (gst_rosstream_get_type())
#define GST_ROSSTREAM(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROSSTREAM, GstRosStream))
#define GST_ROSSTREAM_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROSSTREAM, GstRosStreamClass))
#define GST_ROSSTREAM_GET_CLASS(obj) (G_TYPE_INSTANCE_GET_CLASS((obj), GST_TYPE_ROSSTREAM, GstRosStreamClass))
#define GST_IS_ROSSTREAM(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_ROSSTREAM))
#define GST_IS_ROSSTREAM_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_ROSSTREAM))
#define GST_ROSSTREAM_CAST(obj) ((GstRosStream *)(obj))

// Utility Macro Definition
#define CHECK_NVDS_MEMORY_AND_GPUID(object, surface)                                                           \
    ({                                                                                                         \
        int _errtype = 0;                                                                                      \
        do                                                                                                     \
        {                                                                                                      \
            if ((surface->memType == NVBUF_MEM_DEFAULT || surface->memType == NVBUF_MEM_CUDA_DEVICE) &&        \
                (surface->gpuId != object->gpu_id))                                                            \
            {                                                                                                  \
                GST_ELEMENT_ERROR(object, RESOURCE, FAILED,                                                    \
                                  ("Input surface gpu-id doesnt match with configured gpu-id for element,"     \
                                   " please allocate input using unified memory, or use same gpu-ids"),        \
                                  ("surface-gpu-id=%d,%s-gpu-id=%d", surface->gpuId, GST_ELEMENT_NAME(object), \
                                   object->gpu_id));                                                           \
                _errtype = 1;                                                                                  \
            }                                                                                                  \
        } while (0);                                                                                           \
        _errtype;                                                                                              \
    })

#define CHECK_NPP_STATUS(npp_status, error_str)                   \
    do                                                            \
    {                                                             \
        if ((npp_status) != NPP_SUCCESS)                          \
        {                                                         \
            g_print("Error: %s in %s at line %d: NPP Error %d\n", \
                    error_str, __FILE__, __LINE__, npp_status);   \
            goto error;                                           \
        }                                                         \
    } while (0)

#define CHECK_CUDA_STATUS(cuda_status, error_str)                                  \
    do                                                                             \
    {                                                                              \
        if ((cuda_status) != cudaSuccess)                                          \
        {                                                                          \
            g_print("Error: %s in %s at line %d (%s)\n",                           \
                    error_str, __FILE__, __LINE__, cudaGetErrorName(cuda_status)); \
            goto error;                                                            \
        }                                                                          \
    } while (0)

struct _GstRosStream
{
    GstBaseTransform base_trans;

    // Context of rosstream lib
    RosStreamerCtx *rosstream_ctx;

    // Unique ID of the element. The labels generated by the element will be
    // updated at index `unique_id` of attr_info array in NvDsObjectParams.
    guint unique_id;

    // Frame number of the current input buffer
    guint64 frame_num;

    // Cuda stream used for allocating the CUDA task
    cudaStream_t cuda_stream;

    // Host buffer to store RGB data for use by rosstream lib
    void *host_rgb_buf;

    // the intermediate scratch buffer for conversions RGBA
    NvBufSurface *inter_buf;

    // Resolution at which frames/objects should be processed
    gint processing_width;
    gint processing_height;

    // OpenCv mat containing RGB data
    cv::Mat *cvmat;

    // Input video info (resolution, color_format, framerate, etc)
    GstVideoInfo video_info;

    // Amount of objects processed in single call to rosstream lib
    guint batch_size;

    // GPU ID on which to execute the task
    guint gpu_id;

    // ROS Init flag
    gboolean ros_init;

    // ROS Node Name
    GString *ros_node_name;

    // ROS Topic Publisher Name
    GString *ros_topic_name;

    // ROS Node Handle
    ros::NodeHandle nh;

    // ROS Image Publisher
    image_transport::Publisher image_pub;
};

// Boiler plate struct def
struct _GstRosStreamClass
{
    GstBaseTransformClass parent_class;
};

GType gst_rosstream_get_type(void);

G_END_DECLS

#endif
