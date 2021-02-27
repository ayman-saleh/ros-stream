/*

    rosstream plugin source
    Author: Ayman Saleh

*/

#include <string.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include "rosstream-plugin.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

GST_DEBUG_CATEGORY_STATIC(gst_rosstream_debug);

#define GST_CAT_DEFAULT gst_rosstream_debug
#define USE_EGLIMAGE 1

static GQuark _rsmeta_quark = 0;

// Enum to identify properties
// Left as enum to add on more properties as plugin develops
enum
{
    PROP_0,
    PROP_UNIQUE_ID,
    PROP_ROS_NODE_NAME,
    PROP_ROS_TOPIC_NAME,
    PROP_PROCESSING_WIDTH,
    PROP_PROCESSING_HEIGHT,
    PROP_GPU_DEVICE_ID
};

// Default values for properties
#define DEFAULT_ROS_NODE_NAME "ros_stream"
#define DEFAULT_ROS_TOPIC_NAME "camera0"
#define DEFAULT_PROP_UNIQUE_ID 20
#define DEFAULT_PROCESSING_WIDTH 640
#define DEFAULT_PROCESSING_HEIGHT 480
#define DEFAULT_GPU_ID 0

#define RGB_BYTES_PER_PIXEL 3
#define RGBA_BYTES_PER_PIXEL 4
#define Y_BYTES_PER_PIXEL 1
#define UV_BYTES_PER_PIXEL 2

// By default NVIDIA hardware allocated memory flows throught the pipeline,
// therefore, this can be the only type of memory allocated
#define GST_CAPS_FEATURE_MEMORY_NVMM "memory:NVMM"
static GstStaticPadTemplate gst_rosstream_sink_template =
    GST_STATIC_PAD_TEMPLATE("sink",
                            GST_PAD_SINK,
                            GST_PAD_ALWAYS,
                            GST_STATIC_CAPS(GST_VIDEO_CAPS_MAKE_WITH_FEATURES(GST_CAPS_FEATURE_MEMORY_NVMM,
                                                                              "{ NV12, RGBA, I420 }")));

static GstStaticPadTemplate gst_rosstream_src_template =
    GST_STATIC_PAD_TEMPLATE("src",
                            GST_PAD_SRC,
                            GST_PAD_ALWAYS,
                            GST_STATIC_CAPS(GST_VIDEO_CAPS_MAKE_WITH_FEATURES(GST_CAPS_FEATURE_MEMORY_NVMM,
                                                                              "{ NV12, RGBA, I420 }")));

// Define element type, standard GObject/Gstreamer boilerplate
G_DEFINE_TYPE(GstRosStream, gst_rosstream, GST_TYPE_BASE_TRANSFORM);

static void gst_rosstream_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec);
static void gst_rosstream_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec);

static gboolean gst_rosstream_set_caps(GstBaseTransform *btrans, GstCaps *incaps, GstCaps *outcaps);
static gboolean gst_rosstream_start(GstBaseTransform *btrans);
static gboolean gst_rosstream_stop(GstBaseTransform *btrans);

static GstFlowReturn gst_rosstream_transform_ip(GstBaseTransform *btrans, GstBuffer *inbuf);

static void attach_metadata_full_frame(GstRosStream *rosstream, NvDsFrameMeta *frame_meta, gdouble scale_ratio, RosStreamerOutput *output, guint batch_id);
static void attach_metadata_object(GstRosStream *rosstream, NvDsObjectMeta *obj_meta, RosStreamerOutput *output);

static void gst_ros_init(GstRosStream *rosstream);

// Install properties, set sink and src pad
static void gst_rosstream_class_init(GstRosStreamClass *klass)
{

    GObjectClass *gobject_class;
    GstElementClass *gstelement_class;
    GstBaseTransformClass *gstbasetransform_class;

    // Set DS_NEW_BUFAPI to true to use buf api
    g_setenv("DS_NEW_BUFAPI", "1", TRUE);

    gobject_class = (GObjectClass *)klass;
    gstelement_class = (GstElementClass *)klass;
    gstbasetransform_class = (GstBaseTransformClass *)klass;

    /* Overide base class functions */
    gobject_class->set_property = GST_DEBUG_FUNCPTR(gst_rosstream_set_property);
    gobject_class->get_property = GST_DEBUG_FUNCPTR(gst_rosstream_get_property);

    gstbasetransform_class->set_caps = GST_DEBUG_FUNCPTR(gst_rosstream_set_caps);
    gstbasetransform_class->start = GST_DEBUG_FUNCPTR(gst_rosstream_start);
    gstbasetransform_class->stop = GST_DEBUG_FUNCPTR(gst_rosstream_stop);

    gstbasetransform_class->transform_ip = GST_DEBUG_FUNCPTR(gst_rosstream_transform_ip);

    /* Install Properties */
    g_object_class_install_property(gobject_class, PROP_UNIQUE_ID,
                                    g_param_spec_uint("unique-id",
                                                      "Unique ID",
                                                      "Unique ID for the element. Can be used to identify output of the"
                                                      " element",
                                                      0, G_MAXUINT, DEFAULT_PROP_UNIQUE_ID, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_ROS_NODE_NAME,
                                    g_param_spec_string("ros-node-name",
                                                        "Ros Node Name",
                                                        "Ros node name for publisher",
                                                        DEFAULT_ROS_NODE_NAME, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_ROS_TOPIC_NAME,
                                    g_param_spec_string("ros-topic-name",
                                                        "Ros Topic Name",
                                                        "Ros topic name for publisher",
                                                        DEFAULT_ROS_TOPIC_NAME, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_PROCESSING_WIDTH,
                                    g_param_spec_int("processing-width",
                                                     "Processing Width",
                                                     "Width of the input buffer to algorithm",
                                                     1, G_MAXINT, DEFAULT_PROCESSING_WIDTH, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_PROCESSING_HEIGHT,
                                    g_param_spec_int("processing-height",
                                                     "Processing Height",
                                                     "Height of the input buffer to algorithm",
                                                     1, G_MAXINT, DEFAULT_PROCESSING_HEIGHT, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_GPU_DEVICE_ID,
                                    g_param_spec_uint("gpu-id",
                                                      "Set GPU Device ID",
                                                      "Set GPU Device ID",
                                                      0, G_MAXUINT, DEFAULT_GPU_ID, GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS | GST_PARAM_MUTABLE_READY)));

    /* Set sink and src pad capabilities */
    gst_element_class_add_pad_template(gstelement_class, gst_static_pad_template_get(&gst_rosstream_src_template));
    gst_element_class_add_pad_template(gstelement_class, gst_static_pad_template_get(&gst_rosstream_sink_template));

    /* Set metadata describing the element */
    gst_element_class_set_details_simple(gstelement_class,
                                         "RosStream Plugin",
                                         "Wavemaker Labs RosStream plugin",
                                         "Publish images within deepstream pipleine into ROS framework",
                                         "Wavemaker Labs");
}

static void gst_rosstream_init(GstRosStream *rosstream)
{

    GstBaseTransform *btrans = GST_BASE_TRANSFORM(rosstream);

    // Updating metadata in place, no additional buffer init
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(btrans), TRUE);

    // No change to input caps, set to passthrough
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(btrans), TRUE);

    // Init all property variables to default values
    rosstream->unique_id = DEFAULT_PROP_UNIQUE_ID;
    rosstream->processing_width = DEFAULT_PROCESSING_WIDTH;
    rosstream->processing_height = DEFAULT_PROCESSING_HEIGHT;
    rosstream->gpu_id = DEFAULT_GPU_ID;
    rosstream->ros_init = FALSE;
    rosstream->ros_node_name = g_string_new(DEFAULT_ROS_NODE_NAME);
    rosstream->ros_topic_name = g_string_new(DEFAULT_ROS_TOPIC_NAME);

    // This quark is requried to identify NvDsMeta when iterating through
    // the buffer metadatas
    if (!_rsmeta_quark)
    {
        _rsmeta_quark = g_quark_from_static_string(NVDS_META_STRING);
    }
}

static void gst_ros_init(GstRosStream *rosstream)
{
    // Handle logic to init ROS node
    // Create dummy argc & argv to init node
    int argc = 0;
    char *argv[] = {(char *)"placeholder"};

    ROS_INFO("Initializing ROS Node %s", rosstream->ros_node_name->str);
    ros::init(argc, argv, rosstream->ros_node_name->str, ros::init_options::AnonymousName);

    // Init and assign ros topic
    // Define ROS publishers a Node Handles
    ros::NodeHandle nh;

    ROS_INFO("Initializing ROS Topic %s", rosstream->ros_topic_name->str);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise(rosstream->ros_topic_name->str, 1);

    // Assign topic to rosstream object
    rosstream->image_pub = image_pub;
    rosstream->ros_init = TRUE;

    return;
}

// Function called when a property of the element is set (Boilerplate)
static void gst_rosstream_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec)
{
    GstRosStream *rosstream = GST_ROSSTREAM(object);
    switch (prop_id)
    {
    case PROP_UNIQUE_ID:
        rosstream->unique_id = g_value_get_uint(value);
        break;
    case PROP_ROS_NODE_NAME:
        g_string_erase(rosstream->ros_node_name, 0, rosstream->ros_node_name->len);
        g_string_overwrite(rosstream->ros_node_name, 0, g_value_get_string(value));
        break;
    case PROP_ROS_TOPIC_NAME:
        g_string_erase(rosstream->ros_topic_name, 0, rosstream->ros_topic_name->len);
        g_string_overwrite(rosstream->ros_topic_name, 0, g_value_get_string(value));
        break;
    case PROP_PROCESSING_WIDTH:
        rosstream->processing_width = g_value_get_int(value);
        break;
    case PROP_PROCESSING_HEIGHT:
        rosstream->processing_height = g_value_get_int(value);
        break;
    case PROP_GPU_DEVICE_ID:
        rosstream->gpu_id = g_value_get_uint(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

// Function called when a property of the element is requested (Boilerplate)
static void gst_rosstream_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec)
{
    GstRosStream *rosstream = GST_ROSSTREAM(object);

    switch (prop_id)
    {
    case PROP_UNIQUE_ID:
        g_value_set_uint(value, rosstream->unique_id);
        break;
    case PROP_ROS_NODE_NAME:
        g_value_set_string(value, rosstream->ros_node_name->str);
        break;
    case PROP_ROS_TOPIC_NAME:
        g_value_set_string(value, rosstream->ros_topic_name->str);
        break;
    case PROP_PROCESSING_WIDTH:
        g_value_set_uint(value, rosstream->processing_width);
        break;
    case PROP_PROCESSING_HEIGHT:
        g_value_set_uint(value, rosstream->processing_height);
        break;
    case PROP_GPU_DEVICE_ID:
        g_value_set_uint(value, rosstream->gpu_id);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

// Initialize all resources and start the output thread
static gboolean gst_rosstream_start(GstBaseTransform *btrans)
{
    GstRosStream *rosstream = GST_ROSSTREAM(btrans);
    NvBufSurfaceCreateParams create_params;
    RosStreamerInitParams init_params = {rosstream->unique_id};

    GstQuery *queryparams = NULL;
    guint batch_size = 1;

    // Algorithm specific initialization and resource allocation
    rosstream->rosstream_ctx = RosStreamerCtxInit(&init_params);

    GST_DEBUG_OBJECT(rosstream, "ctx lib %p \n", rosstream->rosstream_ctx);
    CHECK_CUDA_STATUS(cudaSetDevice(rosstream->gpu_id), "Unable to set cuda device");

    rosstream->batch_size = 1;

    queryparams = gst_nvquery_batch_size_new();

    if (gst_pad_peer_query(GST_BASE_TRANSFORM_SINK_PAD(btrans), queryparams) || gst_pad_peer_query(GST_BASE_TRANSFORM_SRC_PAD(btrans), queryparams))
    {
        if (gst_nvquery_batch_size_parse(queryparams, &batch_size))
        {
            rosstream->batch_size = batch_size;
        }
    }

    GST_DEBUG_OBJECT(rosstream, "Setting batch-size %d \n", rosstream->batch_size);
    gst_query_unref(queryparams);

    if (rosstream->inter_buf)
    {
        NvBufSurfaceDestroy(rosstream->inter_buf);
    }

    CHECK_CUDA_STATUS(cudaStreamCreate(&rosstream->cuda_stream), "Could not create cuda stream");

    rosstream->inter_buf = NULL;

    // An intermediate buffer for NV12/RGBA to BGR conversion will be
    // requried. Can be skipped if custom algorithm can work directly on NV12/RGBA

    create_params.gpuId = rosstream->gpu_id;
    create_params.width = rosstream->processing_width;
    create_params.height = rosstream->processing_height;
    create_params.size = 0;
    create_params.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
    create_params.layout = NVBUF_LAYOUT_PITCH;

#ifdef __aarch64__
    create_params.memType = NVBUF_MEM_DEFAULT;
#else
    create_params.memType = NVBUF_MEM_CUDA_UNIFIED;
#endif

    if (NvBufSurfaceCreate(&rosstream->inter_buf, 1, &create_params) != 0)
    {
        GST_ERROR("Error: Could not allocate internal buffer for rosstream");
        goto error;
    }

    // Create host memory for storing converted/scaled interleaved RGB data
    CHECK_CUDA_STATUS(cudaMallocHost(&rosstream->host_rgb_buf, rosstream->processing_width * rosstream->processing_height * RGB_BYTES_PER_PIXEL), "Could not allocate cuda host buffer");

    GST_DEBUG_OBJECT(rosstream, "allocated cuda buffer %p \n", rosstream->host_rgb_buf);

    // CV Mat containing interleaved RGB data. This call does not allocate memeory
    // as is uses host_rgb_buf as data
    rosstream->cvmat = new cv::Mat(rosstream->processing_height, rosstream->processing_width,
                                   CV_8UC3, rosstream->host_rgb_buf, rosstream->processing_width * RGB_BYTES_PER_PIXEL);

    if (!rosstream->cvmat)
        goto error;

    GST_DEBUG_OBJECT(rosstream, "Created Cv Mat\n");

    return TRUE;

error:
    if (rosstream->host_rgb_buf)
    {
        cudaFreeHost(rosstream->host_rgb_buf);
        rosstream->host_rgb_buf = NULL;
    }

    if (rosstream->cuda_stream)
    {
        cudaStreamDestroy(rosstream->cuda_stream);
        rosstream->cuda_stream = NULL;
    }

    if (rosstream->rosstream_ctx)
        RosStreamerCtxDeInit(rosstream->rosstream_ctx);

    return FALSE;
}

// Stop the output thread and free up all the resources
static gboolean gst_rosstream_stop(GstBaseTransform *btrans)
{
    GstRosStream *rosstream = GST_ROSSTREAM(btrans);

    if (rosstream->inter_buf)
        NvBufSurfaceDestroy(rosstream->inter_buf);
    rosstream->inter_buf = NULL;

    if (rosstream->cuda_stream)
        cudaStreamDestroy(rosstream->cuda_stream);
    rosstream->cuda_stream = NULL;

    delete rosstream->cvmat;
    rosstream->cvmat = NULL;

    if (rosstream->host_rgb_buf)
    {
        cudaFreeHost(rosstream->host_rgb_buf);
        rosstream->host_rgb_buf = NULL;
    }

    GST_DEBUG_OBJECT(rosstream, "deleted CV Mat \n");

    /* Deinit the algorithm library */
    RosStreamerCtxDeInit(rosstream->rosstream_ctx);
    rosstream->rosstream_ctx = NULL;

    GST_DEBUG_OBJECT(rosstream, "ctx lib released \n");

    return TRUE;
}

// Called when source / sink pad capabilities have been negotiated
static gboolean gst_rosstream_set_caps(GstBaseTransform *btrans, GstCaps *incaps, GstCaps *outcaps)
{

    GstRosStream *rosstream = GST_ROSSTREAM(btrans);

    // Save the input video information, since this will be requried later
    gst_video_info_from_caps(&rosstream->video_info, incaps);

    return TRUE;
}

// Scale the entire frame to the processing resolution while maintaining aspect ratio,
// or crop and scale objects to the processing resolution maintaining the aspect ratio
// Remove the padding requried by hardware and convert from RGBA to RGB using OpenCv
static GstFlowReturn get_converted_mat(GstRosStream *rosstream, NvBufSurface *input_buf, gint idx, NvOSD_RectParams *crop_rect_params, gdouble &ratio, gint input_width, gint input_height)
{
    NvBufSurfTransform_Error err;
    NvBufSurfTransformConfigParams transform_config_params;
    NvBufSurfTransformParams transform_params;
    NvBufSurfTransformRect src_rect;
    NvBufSurfTransformRect dst_rect;
    NvBufSurface ip_surf;
    cv::Mat in_mat;
    ip_surf = *input_buf;

    ip_surf.numFilled = ip_surf.batchSize = 1;
    ip_surf.surfaceList = &(input_buf->surfaceList[idx]);

    gint src_left = GST_ROUND_UP_2((unsigned int)crop_rect_params->left);
    gint src_top = GST_ROUND_UP_2((unsigned int)crop_rect_params->top);
    gint src_width = GST_ROUND_DOWN_2((unsigned int)crop_rect_params->width);
    gint src_height = GST_ROUND_DOWN_2((unsigned int)crop_rect_params->height);

    double hdest = rosstream->processing_width * src_height / (double)src_width;
    double wdest = rosstream->processing_height * src_width / (double)src_height;
    guint dest_width, dest_height;

    if (hdest <= rosstream->processing_height)
    {
        dest_width = rosstream->processing_width;
        dest_height = hdest;
    }
    else
    {
        dest_width = wdest;
        dest_height = rosstream->processing_height;
    }

    // Configure transform session parameters for the transformation
    transform_config_params.compute_mode = NvBufSurfTransformCompute_Default;
    transform_config_params.gpu_id = rosstream->gpu_id;
    transform_config_params.cuda_stream = rosstream->cuda_stream;

    // Set the transform session parameters for the convertsions executed in this thread
    err = NvBufSurfTransformSetSessionParams(&transform_config_params);
    if (err != NvBufSurfTransformError_Success)
    {
        GST_ELEMENT_ERROR(rosstream, STREAM, FAILED, ("NvBufSurfTransformSetSessionParams failed with error %d", err), (NULL));
        goto error;
    }

    // Calculate scaling ratio while maintaing aspect ration
    ratio = MIN(1.0 * dest_width / src_width, 1.0 * dest_height / src_height);
    if ((crop_rect_params->width == 0) || (crop_rect_params->height == 0))
    {
        GST_ELEMENT_ERROR(rosstream, STREAM, FAILED, ("%s:crop_rect_params dimensions are zero", __func__), (NULL));
        goto error;
    }

#ifdef __aarch64__
    if (ratio <= 1.0 / 16 || ratio >= 16.0)
    {
        // Currently cannot scale by ratio > 16 or < 1/16 for Jetson
        goto error;
    }
#endif

    // Set the transform ROIs for source and destination
    src_rect = {(guint)src_top, (guint)src_left, (guint)src_width, (guint)src_height};
    dst_rect = {0, 0, (guint)dest_width, (guint)dest_height};

    // Set the transform parameters
    transform_params.src_rect = &src_rect;
    transform_params.dst_rect = &dst_rect;
    transform_params.transform_flag = NVBUFSURF_TRANSFORM_FILTER | NVBUFSURF_TRANSFORM_CROP_SRC | NVBUFSURF_TRANSFORM_CROP_DST;
    transform_params.transform_filter = NvBufSurfTransformInter_Default;

    // Memset the memory
    NvBufSurfaceMemSet(rosstream->inter_buf, 0, 0, 0);

    GST_DEBUG_OBJECT(rosstream, "Scaling and converting input buffer\n");

    // Transformation scaling+format conversion if any.
    err = NvBufSurfTransform(&ip_surf, rosstream->inter_buf, &transform_params);
    if (err != NvBufSurfTransformError_Success)
    {
        GST_ELEMENT_ERROR(rosstream, STREAM, FAILED, ("NvBufSurfTransform failed with error %d while converting buffer", err), (NULL));
        goto error;
    }

    // Map the buffer so that it can be accessed by CPU
    if (NvBufSurfaceMap(rosstream->inter_buf, 0, 0, NVBUF_MAP_READ) != 0)
    {
        goto error;
    }

    // Cache the mapped data for CPU access
    NvBufSurfaceSyncForCpu(rosstream->inter_buf, 0, 0);

    // Use openCV to remove padding and convert RGBA to BGR. Can be skipped if
    // algorithm can handle padded RGBA data.
    in_mat =
        cv::Mat(rosstream->processing_height, rosstream->processing_width,
                CV_8UC4, rosstream->inter_buf->surfaceList[0].mappedAddr.addr[0],
                rosstream->inter_buf->surfaceList[0].pitch);

#if (CV_MAJOR_VERSION >= 4)
    cv::cvtColor(in_mat, *rosstream->cvmat, cv::COLOR_RGBA2BGR);
#else
    cv::cvtColor(in_mat, *rosstream->cvmat, CV_RGBA2BGR);
#endif

    if (NvBufSurfaceUnMap(rosstream->inter_buf, 0, 0))
    {
        goto error;
    }

#ifdef __aarch64__
    // To use the converted buffer in CUDA, create an EGLImage and then use
    // CUDA-EGL interop APIs
    if (USE_EGLIMAGE)
    {
        if (NvBufSurfaceMapEglImage(rosstream->inter_buf, 0) != 0)
        {
            goto error;
        }

        // dsexample->inter_buf->surfaceList[0].mappedAddr.eglImage
        //Use interop APIs cuGraphicsEGLRegisterImage and
        // cuGraphicsResourceGetMappedEglFrame to access the buffer in CUDA

        // Destroy the EGLImage
        NvBufSurfaceUnMapEglImage(rosstream->inter_buf, 0);
    }
#endif

    // We will first convert only the Region of Interest (the entire frame or the
    // object bounding box) to RGB and then scale the converted RGB frame to
    //  processing resolution.
    return GST_FLOW_OK;

error:
    return GST_FLOW_ERROR;
}

//Called when element recieves an input buffer from upstream element.
static GstFlowReturn gst_rosstream_transform_ip(GstBaseTransform *btrans, GstBuffer *inbuf)
{
    GstRosStream *rosstream = GST_ROSSTREAM(btrans);
    GstMapInfo in_map_info;
    GstFlowReturn flow_ret = GST_FLOW_ERROR;
    gdouble scale_ratio = 1.0;
    RosStreamerOutput *output;

    NvBufSurface *surface = NULL;
    NvDsBatchMeta *batch_meta = NULL;
    NvDsFrameMeta *frame_meta = NULL;
    NvDsMetaList *l_frame = NULL;
    guint i = 0;

    // Check if ROS is initialized
    if (!rosstream->ros_init){
        gst_ros_init(rosstream);
    }

    CHECK_CUDA_STATUS(cudaSetDevice(rosstream->gpu_id), "Unable to set cuda device");

    memset(&in_map_info, 0, sizeof(in_map_info));

    if (!gst_buffer_map(inbuf, &in_map_info, GST_MAP_READ))
    {
        g_print("Error: Failed to map gst buffer\n");
        goto error;
    }

    nvds_set_input_system_timestamp(inbuf, GST_ELEMENT_NAME(rosstream));
    surface = (NvBufSurface *)in_map_info.data;

    GST_DEBUG_OBJECT(rosstream, "Processing Frame %" G_GUINT64_FORMAT " Surface %p\n", rosstream->frame_num, surface);

    if (CHECK_NVDS_MEMORY_AND_GPUID(rosstream, surface))
        goto error;

    batch_meta = gst_buffer_get_nvds_batch_meta(inbuf);
    if (batch_meta == nullptr)
    {
        GST_ELEMENT_ERROR(rosstream, STREAM, FAILED, ("NvDsBatchMeta not found for input buffer."), (NULL));
        return GST_FLOW_ERROR;
    }

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL; l_frame = l_frame->next)
    {
        frame_meta = (NvDsFrameMeta *)(l_frame->data);
        NvOSD_RectParams rect_params;

        // Scale the entire frame to processing resolution
        rect_params.left = 0;
        rect_params.top = 0;
        rect_params.width = rosstream->video_info.width;
        rect_params.height = rosstream->video_info.height;

        // Scale and convert the frame
        if (get_converted_mat(rosstream, surface, i, &rect_params, scale_ratio, rosstream->video_info.width, rosstream->video_info.height) != GST_FLOW_OK)
        {
            goto error;
        }

        // Process to get the output
        output = RosStreamerProcess(rosstream->rosstream_ctx, rosstream->cvmat->data);

        // Attach the metadata for the full frame
        i++;
        free(output);

        // Begin ROS Publish Process

        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg;

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, *rosstream->cvmat);
        
        cv::Size s = rosstream->cvmat->size();
        int rows = s.height;
        int cols = s.width;

        ROS_DEBUG("PUBLISHING ROS IMAGE WITH height = %d, width = %d", rows, cols);

        img_bridge.toImageMsg(img_msg);
        rosstream->image_pub.publish(img_msg);
    }

    flow_ret = GST_FLOW_OK;

error:

    nvds_set_output_system_timestamp(inbuf, GST_ELEMENT_NAME(rosstream));
    gst_buffer_unmap(inbuf, &in_map_info);
    return flow_ret;
}

// Boiler plate for registering a plugin and an element
static gboolean rosstream_plugin_init(GstPlugin *plugin)
{
    GST_DEBUG_CATEGORY_INIT(gst_rosstream_debug, "rosstream", 0, "rosstream plugin");

    return gst_element_register(plugin, "rosstream", GST_RANK_PRIMARY, GST_TYPE_ROSSTREAM);
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR, rosstream_plugin, DESCRIPTION, rosstream_plugin_init, VERSION, LICENSE, BINARY_PACKAGE, URL)