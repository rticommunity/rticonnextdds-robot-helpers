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

void print_help(const char *program_name)
{
    std::cout << "Usage: " << program_name << "[options]\nOptions:\n";
    std::cout << " -domainId    <domain ID>    Domain ID\n";
    std::cout << " -mode        <1,2,3,4>      Subscriber modes\n";
    std::cout << "                                 1. subscriber_flat\n";
    std::cout << "                                 2. subscriber_zero_copy\n";
    std::cout << "                                 3. "
                 "subscriber_flat_zero_copy\n";
    std::cout << "                                 4. subscriber_plain\n";
    std::cout << " -displaySample              Displays the sample\n";
    std::cout << " -nic         <IP address>   Use the nic specified by "
                 "<ipaddr> to send\n";
    std::cout << "                             Default: 127.0.0.1\n";
    std::cout << " -help                       Displays this information\n";
}

int main(int argc, char *argv[])
{
    ApplicationOptions options;


    for (int i = 1; i < argc; i++) {
        if (strstr(argv[i], "-h") == argv[i]) {
            print_help(argv[0]);
            return 0;
        } else if (strstr(argv[i], "-di") == argv[i]) {
            options.display_sample = true;
        } else if (strstr(argv[i], "-d") == argv[i]) {
            options.domain_id = atoi(argv[++i]);
        } else if (strstr(argv[i], "-m") == argv[i]) {
            options.mode = atoi(argv[++i]);
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
            subscriber_flat(options);
            break;
        case 2:
            subscriber_zero_copy(options);
            break;
        case 3:
            subscriber_flat_zero_copy(options);
            break;
        case 4:
            subscriber_plain(options);
            break;
        }

    } catch (const std::exception &ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in subscriber: " << ex.what() << std::endl;
        return -1;
    }

    // RTI Connext provides a finalize_participant_factory() method
    // if you want to release memory used by the participant factory singleton.
    // Uncomment the following line to release the singleton:
    //
    // dds::domain::DomainParticipant::finalize_participant_factory();

    return 0;
}
