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

void publisher_copy_sample(int domain_id, int sample_count)
{
    using namespace rti::camera::plain;

    std::cout << "Running publisher_copy_sample\n";

    dds::domain::DomainParticipant participant(domain_id);

    std::unique_ptr<CameraImage> ping_sample(new CameraImage);
    std::unique_ptr<CameraImage> copy(new CameraImage);

    int count = 0;
    uint64_t total_latency = 0;
    while (count < sample_count || sample_count == 0) {
        ping_sample->data()[45345] = count + sample_count + 100;
        ping_sample->timestamp(participant->current_time().to_microsecs());
        *copy = *ping_sample;
        if (copy->data()[45345] == 3) {
            return;
        }

        count++;

        uint64_t latency = participant->current_time().to_microsecs()
                - ping_sample->timestamp();
        total_latency += latency;
        if (count % 10 == 0) {
            std::cout << "Average end-to-end latency: "
                      << total_latency / (count * 1) << " microseconds\n";
        }
    }
}

void print_help(const char *program_name)
{
    std::cout << "Usage: " << program_name << "[options]\nOptions:\n";
    std::cout << " -domainId    <domain ID>    Domain ID\n";
    std::cout << " -mode        <1,2,3,4>      Publisher modes\n";
    std::cout << "                                 1. publisher_flat\n";
    std::cout << "                                 2. publisher_zero_copy\n";
    std::cout
            << "                                 3. publisher_flat_zero_copy\n";
    std::cout << "                                 4. publisher_plain\n";
    std::cout << " -sampleCount <sample count> Sample count\n";
    std::cout << "                             Default: -1 (infinite)\n";
    std::cout << " -executionTime <sec>        Execution time in seconds\n";
    std::cout << "                             Default: 30\n";
    std::cout << " -nic         <IP address>   Use the nic specified by "
                 "<ipaddr> to send\n";
    std::cout << "                             Default: automatic\n";
    std::cout << " -help                       Displays this information\n";
}

int main(int argc, char *argv[])
{
    ApplicationOptions options;

    for (int i = 1; i < argc; i++) {
        if (strstr(argv[i], "-h") == argv[i]) {
            print_help(argv[0]);
            return 0;
        } else if (strstr(argv[i], "-d") == argv[i]) {
            options.domain_id = atoi(argv[++i]);
        } else if (strstr(argv[i], "-m") == argv[i]) {
            options.mode = atoi(argv[++i]);
        } else if (strstr(argv[i], "-s") == argv[i]) {
            options.sample_count = atoi(argv[++i]);
        } else if (strstr(argv[i], "-e") == argv[i]) {
            options.execution_time = atoi(argv[++i]) * 1000000;
        } else if (strstr(argv[i], "-n") == argv[i]) {
            options.nic = (argv[++i]);
        } else {
            std::cout << "unexpected option: " << argv[i] << std::endl;
            return -1;
        }
    }


    // To turn on additional logging, include <rti/config/Logger.hpp> and
    // uncomment the following line:
    // rti::config::Logger::instance().verbosity(rti::config::Verbosity::WARNING);

    try {
        switch (options.mode) {
        case 1:
            publisher_flat(options);
            break;
        case 2:
            publisher_zero_copy(options);
            break;
        case 3:
            publisher_flat_zero_copy(options);
            break;
        case 4:
            publisher_plain(options);
            break;
        case 5:
            publisher_copy_sample(0, 0);
            break;
        }

    } catch (const std::exception &ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in publisher: " << ex.what() << std::endl;
        return -1;
    }

    // RTI Connext provides a finalize_participant_factory() method
    // if you want to release memory used by the participant factory singleton.
    // Uncomment the following line to release the singleton:
    //
    // dds::domain::DomainParticipant::finalize_participant_factory();

    return 0;
}
