// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_DDS_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_DDS_HPP

#include <dds/dds.hpp>

#include <rti/core/cond/AsyncWaitSet.hpp>

#include <rti/ros2/processor/processor_base.hpp>

namespace rti { namespace ros2 { namespace processor {

template<typename T, typename V>
class DdsProcessorNode : public BaseProcessorNode<T,V>
{
public:
  DdsProcessorNode(
    const std::string & node_name,
    const std::string & topic_in,
    const std::string & type_in,
    const std::string & topic_out,
    const std::string & type_out,
    const int32_t domain_id = 0,
    const int32_t history_depth = 10,
    const size_t prealloc_max = 1024,
    const int32_t thread_pool_size = 1)
  : BaseProcessorNode<T,V>(node_name, topic_in, topic_out),
    domain_id_(domain_id),
    history_depth_(history_depth),
    prealloc_max_(prealloc_max),
    type_in_(type_in),
    type_out_(type_out)
  {
    // Create an AsyncWaitSet to call our listeners in dedicated thread
    // instead of calling them from the middleware's transport receive threads.
    rti::core::cond::AsyncWaitSetProperty ws_props;
    ws_props.thread_pool_size(thread_pool_size);
    ws_ = std::make_unique<rti::core::cond::AsyncWaitSet>(ws_props);
  }

  virtual ~DdsProcessorNode()
  {
    stop();
  }

  dds::domain::DomainParticipant
  lookup_participant()
  {
    auto participant = dds::domain::find(domain_id_);
    if (dds::core::null == participant) {
      RCLCPP_ERROR(this->get_logger(), "failed to look up DomainParticipant. "
        "Is the application running on rmw_connextdds?\n");
      throw new std::runtime_error("failed to look up DomainParticipant");
    }
    return participant;
  }

protected:
  virtual void create_entities()
  {
    BaseProcessorNode<T,V>::create_entities();
    this->start();
  }

  virtual void publish_result(const V & msg)
  {
    pub_.write(msg);
  }

  virtual void create_output()
  {
    dds::domain::DomainParticipant participant = lookup_participant();
    auto publisher = dds::pub::Publisher(participant);
    auto topic = dds::topic::Topic<V>(participant, this->topic_out_, type_out_);
    dds::pub::qos::DataWriterQos writer_qos; 
    configure_output_qos(participant, publisher, topic, writer_qos);
    pub_ = dds::pub::DataWriter<V>(publisher, topic, writer_qos);
  }

  virtual void configure_output_qos(
    dds::domain::DomainParticipant & participant,
    dds::pub::Publisher & publisher,
    dds::topic::Topic<V> & topic,
    dds::pub::qos::DataWriterQos & writer_qos)
  {
    (void)participant;
    (void)topic;

    publisher.default_datawriter_qos(writer_qos);

    rti::core::policy::Property properties;
    std::string prealloc_max_str = std::to_string(prealloc_max_);
    properties.set(
      {"dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size",
        prealloc_max_str.c_str()},
      false);
    writer_qos << properties;

    dds::core::policy::History history(
      dds::core::policy::HistoryKind::KEEP_LAST, history_depth_);
    writer_qos << history;
  }

  virtual void create_input()
  {
    dds::domain::DomainParticipant participant = lookup_participant();
    auto subscriber = dds::sub::Subscriber(participant);
    auto topic = dds::topic::Topic<T>(participant, this->topic_in_, type_in_);
    dds::sub::qos::DataReaderQos reader_qos; 
    configure_input_qos(participant, subscriber, topic, reader_qos);
    sub_ = dds::sub::DataReader<T>(subscriber, topic, reader_qos);
    
    sub_condition_ = dds::sub::cond::ReadCondition(
      sub_,
      dds::sub::status::DataState(
        dds::sub::status::SampleState::not_read(),
        dds::sub::status::ViewState::any(),
        dds::sub::status::InstanceState::any()),
      std::bind(DdsProcessorNode::on_data_available, this));
    
    ws_->attach_condition(sub_condition_);
  }

  virtual void configure_input_qos(
    dds::domain::DomainParticipant & participant,
    dds::sub::Subscriber & subscriber,
    dds::topic::Topic<T> & topic,
    dds::sub::qos::DataReaderQos & reader_qos)
  {
    (void)participant;
    (void)topic;

    subscriber.default_datareader_qos(reader_qos);

    rti::core::policy::Property properties;
    std::string prealloc_max_str = std::to_string(prealloc_max_);
    properties.set(
      {"dds.data_reader.history.memory_manager.fast_pool.pool_buffer_max_size",
        prealloc_max_str.c_str()},
      false);
    reader_qos << properties;

    reader_qos << dds::core::policy::Reliability::Reliable();
    dds::core::policy::History history(
      dds::core::policy::HistoryKind::KEEP_LAST, history_depth_);
    reader_qos << history;
  }

  static void on_data_available(DdsProcessorNode * const self)
  {
    dds::sub::LoanedSamples<T> samples = self->sub_.take();
    for (auto it = samples.begin(); it != samples.end(); it++)
    {
      if (it->info().valid()) {
        self->on_data(it->data());
      }
    }
  }

  static void async_waitset_thread(DdsProcessorNode * self)
  {
    std::unique_lock<std::mutex> lock(self->ws_thread_mutex_);
    self->ws_->start();
    self->ws_thread_cond_.wait(lock);
    self->ws_->stop();
  }

  void start()
  {
    ws_thread_ = std::thread(DdsProcessorNode::async_waitset_thread, this);
  }

  void stop()
  {
    {
      std::unique_lock<std::mutex> lock(ws_thread_mutex_);
      ws_thread_cond_.notify_one();
    }
    ws_thread_.join();
  }

  const int32_t domain_id_;
  const int32_t history_depth_;
  const size_t prealloc_max_;
  std::string type_in_;
  std::string type_out_;
  dds::sub::DataReader<T> sub_{nullptr};
  dds::pub::DataWriter<V> pub_{nullptr};
  std::thread ws_thread_;
  bool ws_thread_started_{false};
  std::mutex ws_thread_mutex_;
  std::condition_variable ws_thread_cond_;
  std::unique_ptr<rti::core::cond::AsyncWaitSet> ws_;
  dds::sub::cond::ReadCondition sub_condition_{nullptr};
};

}  // namespace processor
}  // namespace ros2
}  // namespace rti

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_DDS_HPP
