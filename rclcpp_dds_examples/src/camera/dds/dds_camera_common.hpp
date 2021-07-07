
/*
 * Copyright 2019 Real-Time Innovations, Inc.  All rights reserved.
 *
 * RTI grants Licensee a license to use, modify, compile, and create derivative
 * works of the Software.  Licensee has the right to distribute object form
 * only for use with RTI products.  The Software is provided "as is", with no
 * warranty of any type, including any warranty for fitness for any purpose.
 * RTI is under no obligation to maintain or support the Software.  RTI shall
 * not be liable for any incidental or consequential damages arising out of the
 * use or inability to use the software.
 */

#ifndef dds_camera_common_hpp
#define dds_camera_common_hpp

/* Common.hpp
 *
 * Utilities used by CameraImage_publisher.cxx and CameraImage_subscriber.cxx.
 */

#include <dds/dds.hpp>
#include <dds/core/cond/GuardCondition.hpp>
#include <dds/pub/DataWriter.hpp>
#include <dds/pub/DataWriterListener.hpp>

#define CAMERA_TOPIC_PING      "rt/ping"
#define CAMERA_TOPIC_PONG      "rt/pong"
#define CAMERA_TYPE_NAME       "PingMessage"

#include "camera/CameraCommon.hpp"
#include "camera/CameraCommonFlat.hpp"
#include "camera/CameraImage.hpp"
#include "camera/CameraImageFlat.hpp"
#include "camera/CameraImageFlatZc.hpp"
#include "camera/CameraImageZc.hpp"

/**
 * Wait for discovery
 */
template <typename T>
void wait_for_reader(dds::pub::DataWriter<T> &writer, bool match = true)
{
    while ((match && dds::pub::matched_subscriptions(writer).empty())
           || (!match && !dds::pub::matched_subscriptions(writer).empty())) {
        rti::util::sleep(dds::core::Duration::from_millisecs(100));
    }
}

template <typename T>
void wait_for_writer(dds::sub::DataReader<T> &reader)
{
    while (dds::sub::matched_publications(reader).empty()) {
        rti::util::sleep(dds::core::Duration::from_millisecs(100));
    }
}

// CameraImageType can be flat_types::CameraImage or
// flat_zero_copy_types::CameraImage
template <typename CameraImageType>
void populate_flat_sample(CameraImageType &sample, int count)
{
    auto image = sample.root();
    image.format(rti::camera::common::Format::RGB);
    image.resolution().height(4320);
    image.resolution().width(7680);

    auto image_data = image.data();
    for (int i = 0; i < 4; i++) {
        uint8_t image_value = (48 + count) % 124;
        image_data.set_element(i, image_value);
    }
}

// CameraImageType can be zero_copy_types::CameraImage or
// plain_types::CameraImage
template <typename CameraImageType>
void populate_plain_sample(CameraImageType &sample, int count)
{
    sample.format(rti::camera::common::Format::RGB);
    sample.resolution().height(4320);
    sample.resolution().width(7680);

    for (int i = 0; i < 4; i++) {
        uint8_t image_value = (48 + count) % 124;
        sample.data()[i] = image_value;
    }
}

template <typename CameraImageType>
void display_flat_sample(const CameraImageType &sample)
{
    auto image = sample.root();

    std::cout << "\nTimestamp " << image.timestamp() << " " << image.format();

    std::cout << " Data (4 Bytes) ";
    const uint8_t *image_data = image.data().get_elements();
    for (int i = 0; i < 4; i++) {
        std::cout << image_data[i];
    }
    std::cout << std::endl;
}

template <typename CameraImageType>
void display_plain_sample(const CameraImageType &sample)
{
    std::cout << "\nTimestamp " << sample.timestamp() << " " << sample.format();

    std::cout << " Data (4 Bytes) ";
    for (int i = 0; i < 4; i++) {
        std::cout << sample.data()[i];
    }
    std::cout << std::endl;
}

struct ApplicationOptions {
    ApplicationOptions()
            : domain_id(0),
              mode(0),
              sample_count(-1),  // infinite
              execution_time(30000000),
              display_sample(false)
    {
    }

    int domain_id;
    int mode;
    int sample_count;
    uint64_t execution_time;
    bool display_sample;
    std::string nic;  // default: empty (no nic will be explicitly picked)
};

inline void configure_nic(
        dds::domain::qos::DomainParticipantQos &qos,
        const std::string &nic)
{
    using rti::core::policy::Property;

    if (!nic.empty()) {
        qos.policy<Property>().set(
                { "dds.transport.UDPv4.builtin.parent.allow_interfaces", nic });
    }
}

inline void print_latency(int total_latency, int count)
{
    if (count > 0) {
        std::cout << "Average end-to-end latency: "
                  << total_latency / (count * 2) << " microseconds\n";
    } else {
        std::cout << "No samples received\n";
    }
}

inline dds::pub::qos::DataWriterQos test_load_datawriter_qos()
{
  using namespace dds::core;

  std::string qos_profile = "BuiltinQosLibExp::Generic.StrictReliable.LargeData";
  auto qos_provider = QosProvider::Default();

  auto writer_qos = qos_provider.datawriter_qos(qos_profile);
  // Only keep the latest written sample
  writer_qos << policy::History(policy::HistoryKind::KEEP_LAST, 1);
  rti::core::policy::Property props;
  // Optional optimization: prevent dynamic allocation of serialization buffer.
  // This configuration is only possible because the data type is not "unbouded",
  // otherwise we must provide a finite limite to this pool or run out of memory.
  props.set({
    "dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size",
    "LENGTH_UNLIMITED"
  }, false);
  writer_qos << props;
  // Use transient local Durability
  writer_qos << policy::Durability::TransientLocal();

  return writer_qos;
}

inline dds::sub::qos::DataReaderQos test_load_datareader_qos()
{
  using namespace dds::core;

  std::string qos_profile = "BuiltinQosLibExp::Generic.StrictReliable.LargeData";

  auto qos_provider = QosProvider::Default();

  auto reader_qos = qos_provider.datareader_qos(qos_profile);
  // Only keep the latest received sample
  reader_qos << policy::History(policy::HistoryKind::KEEP_LAST, 1);
  // Optional optimization: prevent dynamic allocation of received fragments.
  // This might cause more memory allocation upfront, but may improve latency.
  rti::core::policy::DataReaderResourceLimits dr_limits;
  dr_limits.dynamically_allocate_fragmented_samples(false);
  reader_qos << dr_limits;
  // Use transient local Durability
  reader_qos << policy::Durability::TransientLocal();

  return reader_qos;
}

#endif  // dds_camera_common_hpp
