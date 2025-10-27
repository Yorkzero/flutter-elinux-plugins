// Copyright 2022 Sony Group Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gst_camera.h"

#include <iostream>

GstCamera::GstCamera(std::unique_ptr<CameraStreamHandler> handler)
    : stream_handler_(std::move(handler)) {
  gst_.pipeline = nullptr;
  gst_.source = nullptr;
  gst_.jpegdec = nullptr;
  gst_.video_convert = nullptr;
  gst_.video_sink = nullptr;
  gst_.output = nullptr;
  gst_.bus = nullptr;
  gst_.buffer = nullptr;

  if (!CreatePipeline()) {
    std::cerr << "Failed to create a pipeline" << std::endl;
    DestroyPipeline();
    return;
  }

  // Prerolls before getting information from the pipeline.
  Preroll();

  // Note: v4l2src supports digital zoom (0-3)
  max_zoom_level_ = 3.0;
  min_zoom_level_ = 0.0;
}

GstCamera::~GstCamera() {
  Stop();
  DestroyPipeline();
}

// static
void GstCamera::GstLibraryLoad() { gst_init(NULL, NULL); }

// static
void GstCamera::GstLibraryUnload() { gst_deinit(); }

bool GstCamera::Play() {
  auto result = gst_element_set_state(gst_.pipeline, GST_STATE_PLAYING);
  if (result == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to change the state to PLAYING" << std::endl;
    return false;
  }

  // Waits until the state becomes GST_STATE_PLAYING.
  if (result == GST_STATE_CHANGE_ASYNC) {
    GstState state;
    result =
        gst_element_get_state(gst_.pipeline, &state, NULL, GST_CLOCK_TIME_NONE);
    if (result == GST_STATE_CHANGE_FAILURE) {
      std::cerr << "Failed to get the current state" << std::endl;
    }
  }

  return true;
}

bool GstCamera::Pause() {
  if (gst_element_set_state(gst_.pipeline, GST_STATE_PAUSED) ==
      GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to change the state to PAUSED" << std::endl;
    return false;
  }
  return true;
}

bool GstCamera::Stop() {
  if (gst_element_set_state(gst_.pipeline, GST_STATE_READY) ==
      GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to change the state to READY" << std::endl;
    return false;
  }
  return true;
}

void GstCamera::TakePicture(OnNotifyCaptured on_notify_captured) {
  // Note: v4l2src doesn't have built-in capture capability like camerabin
  // This would need to be implemented using a separate pipeline or appsink
  std::cerr << "TakePicture is not supported with v4l2src pipeline" << std::endl;
  on_notify_captured_ = on_notify_captured;
}

bool GstCamera::SetZoomLevel(float zoom) {
  if (!gst_.source) {
    std::cerr << "Source not initialized" << std::endl;
    return false;
  }
  
  // Clamp zoom level to valid range (0-3 for this camera)
  if (zoom < min_zoom_level_) {
    std::cerr << "zoom level(" << zoom << ") is under the min-zoom level("
              << min_zoom_level_ << ")" << std::endl;
    return false;
  }
  if (zoom > max_zoom_level_) {
    std::cerr << "zoom level(" << zoom << ") is over the max-zoom level("
              << max_zoom_level_ << ")" << std::endl;
    return false;
  }
  
  // Set zoom via V4L2 control
  int zoom_int = static_cast<int>(zoom);
  g_object_set(gst_.source, "extra-controls", 
               gst_structure_new("controls", 
                                "zoom-absolute", G_TYPE_INT, zoom_int, 
                                NULL), 
               NULL);
  
  std::cout << "Set zoom level to: " << zoom_int << std::endl;
  return true;
}

const uint8_t* GstCamera::GetPreviewFrameBuffer() {
  std::shared_lock<std::shared_mutex> lock(mutex_buffer_);
  if (!gst_.buffer) {
    return nullptr;
  }

  const uint32_t pixel_bytes = width_ * height_ * 4;
  gst_buffer_extract(gst_.buffer, 0, pixels_.get(), pixel_bytes);
  return reinterpret_cast<const uint8_t*>(pixels_.get());
}

// Creates a camera pipeline using v4l2src with MJPG format for high frame rate.
// $ gst-launch-1.0 v4l2src device=/dev/video34 ! image/jpeg,width=1920,height=1080,framerate=30/1 !
// jpegdec ! videoconvert ! video/x-raw,format=RGBA ! fakesink
bool GstCamera::CreatePipeline() {
  gst_.pipeline = gst_pipeline_new("pipeline");
  if (!gst_.pipeline) {
    std::cerr << "Failed to create a pipeline" << std::endl;
    return false;
  }
  
  // Create v4l2src as video source
  gst_.source = gst_element_factory_make("v4l2src", "source");
  if (!gst_.source) {
    std::cerr << "Failed to create v4l2src" << std::endl;
    return false;
  }
  
  // Set the device to /dev/video34
  g_object_set(gst_.source, "device", "/dev/video34", NULL);
  
  // Create jpegdec to decode MJPG stream
  gst_.jpegdec = gst_element_factory_make("jpegdec", "jpegdec");
  if (!gst_.jpegdec) {
    std::cerr << "Failed to create jpegdec" << std::endl;
    return false;
  }
  
  gst_.video_convert = gst_element_factory_make("videoconvert", "videoconvert");
  if (!gst_.video_convert) {
    std::cerr << "Failed to create a videoconvert" << std::endl;
    return false;
  }
  
  gst_.video_sink = gst_element_factory_make("fakesink", "videosink");
  if (!gst_.video_sink) {
    std::cerr << "Failed to create a videosink" << std::endl;
    return false;
  }
  
  gst_.bus = gst_pipeline_get_bus(GST_PIPELINE(gst_.pipeline));
  if (!gst_.bus) {
    std::cerr << "Failed to create a bus" << std::endl;
    return false;
  }
  gst_bus_set_sync_handler(gst_.bus, HandleGstMessage, this, NULL);

  // Sets properties to fakesink to get the callback of a decoded frame.
  g_object_set(G_OBJECT(gst_.video_sink), "sync", TRUE, "qos", FALSE, NULL);
  g_object_set(G_OBJECT(gst_.video_sink), "signal-handoffs", TRUE, NULL);
  g_signal_connect(G_OBJECT(gst_.video_sink), "handoff",
                   G_CALLBACK(HandoffHandler), this);

  // Add all elements to the pipeline
  gst_bin_add_many(GST_BIN(gst_.pipeline), gst_.source, gst_.jpegdec, 
                   gst_.video_convert, gst_.video_sink, NULL);

  // Set caps for MJPG format @ 1920x1080 30fps
  auto* mjpeg_caps = gst_caps_from_string("image/jpeg,width=1920,height=1080,framerate=30/1");
  
  // Link: v4l2src -> jpegdec (with MJPG caps filter)
  if (!gst_element_link_filtered(gst_.source, gst_.jpegdec, mjpeg_caps)) {
    std::cerr << "Failed to link source to jpegdec" << std::endl;
    gst_caps_unref(mjpeg_caps);
    return false;
  }
  gst_caps_unref(mjpeg_caps);
  
  // Link: jpegdec -> videoconvert
  if (!gst_element_link(gst_.jpegdec, gst_.video_convert)) {
    std::cerr << "Failed to link jpegdec to videoconvert" << std::endl;
    return false;
  }
  
  // Adds caps to the converter to convert the color format to RGBA.
  auto* rgba_caps = gst_caps_from_string("video/x-raw,format=RGBA");
  
  // Link: videoconvert -> fakesink (with RGBA caps filter)
  auto link_ok = gst_element_link_filtered(gst_.video_convert, gst_.video_sink, rgba_caps);
  gst_caps_unref(rgba_caps);
  if (!link_ok) {
    std::cerr << "Failed to link videoconvert to sink" << std::endl;
    return false;
  }

  return true;
}

void GstCamera::Preroll() {
  if (!gst_.source) {
    return;
  }

  auto result = gst_element_set_state(gst_.pipeline, GST_STATE_PAUSED);
  if (result == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to change the state to PAUSED" << std::endl;
    return;
  }

  // Waits until the state becomes GST_STATE_PAUSED.
  if (result == GST_STATE_CHANGE_ASYNC) {
    GstState state;
    result =
        gst_element_get_state(gst_.pipeline, &state, NULL, GST_CLOCK_TIME_NONE);
    if (result == GST_STATE_CHANGE_FAILURE) {
      std::cerr << "Failed to get the current state" << std::endl;
    }
  }
}

void GstCamera::DestroyPipeline() {
  if (gst_.video_sink) {
    g_object_set(G_OBJECT(gst_.video_sink), "signal-handoffs", FALSE, NULL);
  }

  if (gst_.pipeline) {
    gst_element_set_state(gst_.pipeline, GST_STATE_NULL);
  }

  if (gst_.buffer) {
    gst_buffer_unref(gst_.buffer);
    gst_.buffer = nullptr;
  }

  if (gst_.bus) {
    gst_object_unref(gst_.bus);
    gst_.bus = nullptr;
  }

  if (gst_.pipeline) {
    gst_object_unref(gst_.pipeline);
    gst_.pipeline = nullptr;
  }

  if (gst_.source) {
    gst_.source = nullptr;
  }

  if (gst_.jpegdec) {
    gst_.jpegdec = nullptr;
  }

  if (gst_.video_sink) {
    gst_.video_sink = nullptr;
  }

  if (gst_.video_convert) {
    gst_.video_convert = nullptr;
  }
}

void GstCamera::GetZoomMaxMinSize(float& max, float& min) {
  // v4l2src supports digital zoom via V4L2 controls (0-3 for this camera)
  max = 3.0;
  min = 0.0;
}

// static
void GstCamera::HandoffHandler(GstElement* fakesink, GstBuffer* buf,
                               GstPad* new_pad, gpointer user_data) {
  auto* self = reinterpret_cast<GstCamera*>(user_data);
  auto* caps = gst_pad_get_current_caps(new_pad);
  auto* structure = gst_caps_get_structure(caps, 0);

  int width;
  int height;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);
  gst_caps_unref(caps);
  if (width != self->width_ || height != self->height_) {
    self->width_ = width;
    self->height_ = height;
    self->pixels_.reset(new uint32_t[width * height]);
    std::cout << "Pixel buffer size: width = " << width
              << ", height = " << height << std::endl;
  }

  std::lock_guard<std::shared_mutex> lock(self->mutex_buffer_);
  if (self->gst_.buffer) {
    gst_buffer_unref(self->gst_.buffer);
    self->gst_.buffer = nullptr;
  }
  self->gst_.buffer = gst_buffer_ref(buf);
  self->stream_handler_->OnNotifyFrameDecoded();
}

// static
GstBusSyncReply GstCamera::HandleGstMessage(GstBus* bus,
                                            GstMessage* message,
                                            gpointer user_data) {
  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ELEMENT: {
      auto const* st = gst_message_get_structure(message);
      if (st) {
        auto* self = reinterpret_cast<GstCamera*>(user_data);
        if (gst_structure_has_name(st, "image-done") &&
            self->on_notify_captured_) {
          auto const* filename = gst_structure_get_string(st, "filename");
          self->on_notify_captured_(filename);
        }
      }
      break;
    }
    case GST_MESSAGE_WARNING: {
      gchar* debug;
      GError* error;
      gst_message_parse_warning(message, &error, &debug);
      g_printerr("WARNING from element %s: %s\n", GST_OBJECT_NAME(message->src),
                 error->message);
      g_printerr("Warning details: %s\n", debug);
      g_free(debug);
      g_error_free(error);
      break;
    }
    case GST_MESSAGE_ERROR: {
      gchar* debug;
      GError* error;
      gst_message_parse_error(message, &error, &debug);
      g_printerr("ERROR from element %s: %s\n", GST_OBJECT_NAME(message->src),
                 error->message);
      g_printerr("Error details: %s\n", debug);
      g_free(debug);
      g_error_free(error);
      break;
    }
    default:
      break;
  }

  gst_message_unref(message);

  return GST_BUS_DROP;
}
