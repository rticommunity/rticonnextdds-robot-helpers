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

#include "dds_camera_pub.hpp"

void publisher_plain(const ApplicationOptions &options)
{
    using namespace rti::camera::plain;
    using namespace dds::core::policy;

    std::cout << "Running publisher_plain\n";

    auto participant_qos = dds::core::QosProvider::Default().participant_qos();
    configure_nic(participant_qos, options.nic);
    dds::domain::DomainParticipant participant(
            options.domain_id,
            participant_qos);

    auto writer_qos = test_load_datawriter_qos();
    auto reader_qos = test_load_datareader_qos();

    // Create the ping DataWriter
    dds::topic::Topic<CameraImage> ping_topic(
      participant, CAMERA_TOPIC_PING, CAMERA_TYPE_NAME);

    // Create the pong DataWriter with a profile from USER_QOS_PROFILES.xml
    dds::pub::DataWriter<CameraImage> writer(
            dds::pub::Publisher(participant),
            ping_topic,
            writer_qos);

    // Create the pong DataReader
    dds::topic::Topic<CameraImage> pong_topic(
      participant, CAMERA_TOPIC_PONG, CAMERA_TYPE_NAME);
    dds::sub::DataReader<CameraImage> reader(
            dds::sub::Subscriber(participant),
            pong_topic,
            reader_qos);

    // Create a ReadCondition for any data on the pong reader, and attach it to
    // a Waitset
    dds::sub::cond::ReadCondition read_condition(
            reader,
            dds::sub::status::DataState::any());
    dds::core::cond::WaitSet waitset;
    waitset += read_condition;

    std::unique_ptr<CameraImage> ping_sample(new CameraImage);

    std::cout << "Waiting for the subscriber application\n";
    wait_for_reader(writer);
    wait_for_writer(reader);
    std::cout << "Discovery complete\n";

    int count = 0;
    uint64_t total_latency = 0;
    uint64_t latency_interval_start_time = 0;
    uint64_t start_ts = participant->current_time().to_microsecs();
    while (count <= options.sample_count || options.sample_count == -1) {
        uint64_t send_ts = participant->current_time().to_microsecs();
        if ((count == options.sample_count)
            || (send_ts - start_ts >= options.execution_time)) {
            ping_sample->timestamp(0);
            writer.write(*ping_sample);
            break;
        }

        // Write the ping sample:
        ping_sample->timestamp(send_ts);
        writer.write(*ping_sample);
        if (latency_interval_start_time == 0) {
            latency_interval_start_time = send_ts;
        }

        // Wait for the pong
        auto conditions = waitset.wait(dds::core::Duration(10));
        if (conditions.empty()) {
            std::cout << "Wait for pong: timeout\n";
        }

        auto pong_samples = reader.take();
        if (pong_samples.length() > 0 && pong_samples[0].info().valid()) {
            count++;
            uint64_t recv_ts = participant->current_time().to_microsecs();
            uint64_t latency = recv_ts - pong_samples[0].data().timestamp();
            total_latency += latency;
            if (recv_ts - latency_interval_start_time > 4000000) {
                print_latency(total_latency, count);
                latency_interval_start_time = 0;
            }
        }
    }
    print_latency(total_latency, count);

    // Wait for unmatch
    wait_for_reader(writer, false);
    std::cout << "Publisher shutting down\n";
}
