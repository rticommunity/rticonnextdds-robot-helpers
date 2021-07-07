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

#include "dds_camera_sub.hpp"

void subscriber_zero_copy(const ApplicationOptions &options)
{
    using namespace rti::camera::zc;
    using namespace rti::core::policy;

    std::cout << "Running subscriber_zero_copy\n";

    dds::domain::DomainParticipant participant(options.domain_id);

    auto writer_qos = test_load_datawriter_qos();
    auto reader_qos = test_load_datareader_qos();

    // Create the ping DataReader
    dds::topic::Topic<CameraImage> ping_topic(
      participant, CAMERA_TOPIC_PING, CAMERA_TYPE_NAME);
    dds::sub::DataReader<CameraImage> reader(
            dds::sub::Subscriber(participant),
            ping_topic,
            reader_qos);

    // Create the pong DataWriter
    dds::topic::Topic<CameraImage> pong_topic(
      participant, CAMERA_TOPIC_PONG, CAMERA_TYPE_NAME);

    dds::pub::DataWriter<CameraImage> writer(
            dds::pub::Publisher(participant),
            pong_topic,
            writer_qos);

    // Create a ReadCondition for any data on the ping reader, and attach it to
    // a Waitset
    dds::sub::cond::ReadCondition read_condition(
            reader,
            dds::sub::status::DataState::any());
    dds::core::cond::WaitSet waitset;
    waitset += read_condition;

    std::cout << "Waiting for the publisher application\n";
    wait_for_reader(writer);
    wait_for_writer(reader);
    std::cout << "Discovery complete\n";

    while (1) {
        // Wait for a ping
        auto conditions = waitset.wait(dds::core::Duration(10));
        if (conditions.empty()) {
            std::cout << "Wait for ping: timeout\n";
        }

        auto ping_samples = reader.take();
        if (ping_samples.length() > 0 && ping_samples[0].info().valid()) {
            if (ping_samples[0].data().timestamp() == 0) {
                // last sample received, break out of receive loop
                break;
            }
            if (options.display_sample) {
                display_plain_sample(ping_samples[0].data());
            }

            // Write the pong sample:
            CameraImage *pong_sample = writer.extensions().get_loan();
            pong_sample->timestamp(ping_samples[0].data().timestamp());
            if (reader->is_data_consistent(ping_samples[0])) {
                writer.write(*pong_sample);
            }
        }
    }
    std::cout << "Subscriber shutting down\n";
}
