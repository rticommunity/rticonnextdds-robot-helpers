// Copyright 2021 Real-Time Innovations, Inc. (RTI)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP_DDS__DDS_NODE_WRAPPER_HPP_
#define RCLCPP_DDS__DDS_NODE_WRAPPER_HPP_

#include "ros2dds/ros2dds.hpp"

#include "rclcpp_dds/dds_node_options.hpp"
#include "rclcpp_dds/visibility_control.hpp"

namespace rclcpp_dds
{

template<
  typename NodeT,
  typename NotifierT = ros2dds::AsyncConditionNotifier>
class DdsNodeMixin : public NodeT
{
public:
  using TopicKind = ros2dds::TopicKind;

  explicit DdsNodeMixin(
    const std::string & node_name,
    const DdsNodeOptions & options = DdsNodeOptions())
  : NodeT(node_name, options),
    node_options_(options)
  {}
  
  explicit DdsNodeMixin(
    const std::string & node_name,
    const std::string & namespace_,
    const DdsNodeOptions & options = DdsNodeOptions())
  : NodeT(node_name, namespace_, options),
    node_options_(options)
  {}

  virtual ~DdsNodeMixin() = default;

  /**
   * @brief Access the DDS domain joined by this node.
   * 
   * @return An unsigned integer identifying the DDS domain joined by this node. 
   */
  size_t domain_id() const
  {
    return ros2dds::domain_id(*this);
  }

  /**
   * @brief Access the DDS DomainParticipant associated with this node.
   * 
   * @return The dds::domain::DomainParticipant instance associated with this node.
   */
  dds::domain::DomainParticipant &
  domain_participant() const
  {
    return ros2dds::domain_participant(*this);
  }

  template<typename MessageT>
  dds::topic::Topic<MessageT>
  lookup_topic(
    const std::string & topic_name,
    const TopicKind topic_kind = TopicKind::Topic)
  {
    return ros2dds::lookup_topic<MessageT>(*this,
      topic_name,
      topic_kind,
      node_options_.use_ros_naming_conventions());
  }

  template<typename MessageT>
  dds::topic::Topic<MessageT>
  create_topic(
    const std::string & topic_name,
    const TopicKind topic_kind = TopicKind::Topic,
    const std::string & topic_type_name = std::string())
  {
    return ros2dds::create_topic<MessageT>(*this,
      topic_name,
      topic_kind,
      node_options_.use_ros_naming_conventions(),
      topic_type_name);
  }

  template<typename MessageT>
  dds::topic::Topic<MessageT>
  assert_topic(
    const std::string & topic_name,
    const TopicKind topic_kind = TopicKind::Topic,
    const std::string & topic_type_name = std::string())
  {
    return ros2dds::assert_topic<MessageT>(*this,
      topic_name,
      topic_kind,
      node_options_.use_ros_naming_conventions(),
      topic_type_name);
  }

  template<typename MessageT>
  dds::topic::ContentFilteredTopic<MessageT>
  create_filtered_topic(
    dds::topic::Topic<MessageT> & base_topic,
    const std::string & filter_name,
    const std::string & filter_expression,
    const std::vector<std::string> & filter_parameters = std::vector<std::string>())
  {
    return ros2dds::create_filtered_topic<MessageT>(*this,
      base_topic,
      filter_name,
      filter_expression,
      filter_parameters);
  }

  template<typename MessageT>
  dds::topic::ContentFilteredTopic<MessageT>
  create_filtered_topic(
    const std::string & topic_name,
    const std::string & filter_name,
    const std::string & filter_expression,
    const TopicKind topic_kind = TopicKind::Topic,
    const std::string & topic_type_name = std::string(),
    const std::vector<std::string> & filter_parameters = std::vector<std::string>())
  {
    return ros2dds::create_filtered_topic<MessageT>(*this,
      topic_name,
      filter_name,
      filter_expression,
      filter_parameters,
      topic_kind,
      node_options_.use_ros_naming_conventions(),
      topic_type_name);
  }

  /**
   * @brief 
   * 
   * @param topic_name 
   * @return dds::pub::qos::DataWriterQos 
   */
  dds::pub::qos::DataWriterQos
  default_datawriter_qos(
    const std::string & topic_name = std::string(),
    const TopicKind topic_kind = TopicKind::Topic)
  {
    if (topic_name.length() > 0) {
      return ros2dds::default_datawriter_qos(*this,
        topic_name,
        true /* expand */,
        topic_kind,
        node_options_.use_ros_naming_conventions());
    } else {
      return ros2dds::default_datawriter_qos(*this);
    }
  }

  /**
   * @brief 
   * 
   * @param topic_name 
   * @return dds::sub::qos::DataReaderQos 
   */
  dds::sub::qos::DataReaderQos
  default_datareader_qos(
    const std::string & topic_name = std::string(),
    const TopicKind topic_kind = TopicKind::Topic)
  {
    if (topic_name.length() > 0) {
      return ros2dds::default_datareader_qos(*this,
        topic_name,
        true /* expand */,
        topic_kind,
        node_options_.use_ros_naming_conventions());
    } else {
      return ros2dds::default_datareader_qos(*this);
    }
  }
  
  /**
   * @brief 
   * 
   * @param qos_profile 
   * @param topic_name 
   * @return dds::pub::qos::DataWriterQos 
   */
  dds::pub::qos::DataWriterQos
  datawriter_qos_profile(const std::string & qos_profile)
  {
    // TODO(asorbini) support get_w_topic_name()
    return ros2dds::datawriter_qos_profile(*this, qos_profile);
  }

  /**
   * @brief 
   * 
   * @param qos_profile 
   * @param topic_name 
   * @return dds::pub::qos::DataReaderQos 
   */
  dds::sub::qos::DataReaderQos
  datareader_qos_profile(const std::string & qos_profile)
  {
    // TODO(asorbini) support get_w_topic_name()
    return ros2dds::datawriter_qos_profile(*this, qos_profile);
  }

  template<typename MessageT>
  dds::pub::DataWriter<MessageT>
  create_datawriter(
    const std::string & topic_name,
    const TopicKind topic_kind = TopicKind::Topic,
    const std::string & type_name = std::string())
  {
    return ros2dds::create_datawriter<MessageT>(*this,
      topic_name,
      topic_kind,
      node_options_.use_ros_naming_conventions(),
      type_name);
  }

  template<typename MessageT>
  dds::pub::DataWriter<MessageT>
  create_datawriter(
    const std::string & topic_name,
    const dds::pub::qos::DataWriterQos & qos,
    const TopicKind topic_kind = TopicKind::Topic,
    const std::string & type_name = std::string())
  {
    return ros2dds::create_datawriter<MessageT>(*this,
      topic_name,
      qos,
      topic_kind,
      node_options_.use_ros_naming_conventions(),
      type_name);
  }

  template<typename MessageT>
  dds::pub::DataWriter<MessageT>
  create_datawriter(dds::topic::Topic<MessageT> & topic)
  {
    return ros2dds::create_datawriter<MessageT>(*this, topic);
  }

  template<typename MessageT>
  dds::pub::DataWriter<MessageT>
  create_datawriter(
    dds::topic::Topic<MessageT> & topic,
    const dds::pub::qos::DataWriterQos & qos)
  {
    return ros2dds::create_datawriter<MessageT>(*this, topic, qos);
  }

  template<typename MessageT>
  dds::sub::DataReader<MessageT>
  create_datareader(
    const std::string & topic_name,
    const TopicKind topic_kind = TopicKind::Topic,
    const std::string & type_name = std::string())
  {
    return ros2dds::create_datareader<MessageT>(*this,
      topic_name,
      topic_kind,
      node_options_.use_ros_naming_conventions(),
      type_name);
  }

  template<typename MessageT>
  dds::sub::DataReader<MessageT>
  create_datareader(
    const std::string & topic_name,
    const dds::sub::qos::DataReaderQos & qos,
    const TopicKind topic_kind = TopicKind::Topic,
    const std::string & type_name = std::string())
  {
    return ros2dds::create_datareader<MessageT>(*this,
      topic_name,
      qos,
      topic_kind,
      node_options_.use_ros_naming_conventions(),
      type_name);
  }

  template<typename MessageT>
  dds::sub::DataReader<MessageT>
  create_datareader(dds::topic::Topic<MessageT> & topic)
  {
    return ros2dds::create_datareader<MessageT>(*this, topic);
  }

  template<typename MessageT>
  dds::sub::DataReader<MessageT>
  create_datareader(
    dds::topic::Topic<MessageT> & topic,
    const dds::sub::qos::DataReaderQos & qos)
  {
    return ros2dds::create_datareader<MessageT>(*this, topic, qos);
  }

  template<typename MessageT>
  void
  set_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::DataCallbackFunction<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = notifier_.template data_callback<MessageT>(reader, cb, data_state);
    read_conditions_.push_back(condition);
  }

  template<typename MessageT>
  void
  set_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::DataQueryCallbackFunction<MessageT> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
    const std::vector<std::string> & query_parameters = std::vector<std::string>())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = notifier_.template data_callback<MessageT>(
      reader, cb, query_expression, data_state, query_parameters);
    query_conditions_.push_back(condition);
  }

  template<typename MessageT>
  void
  set_read_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::MessageCallbackFunction<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = notifier_.template read_data_callback<MessageT>(reader, cb, data_state);
    read_conditions_.push_back(condition);
  }

  template<typename MessageT>
  void
  set_read_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::MessageCallbackFunction<MessageT> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
    const std::vector<std::string> & query_parameters = std::vector<std::string>())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = notifier_.template read_data_callback<MessageT>(
      reader, cb, query_expression, data_state, query_parameters);
    query_conditions_.push_back(condition);
  }

  template<typename MessageT>
  void
  set_take_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::MessageCallbackFunction<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = notifier_.template take_data_callback<MessageT>(reader, cb, data_state);
    read_conditions_.push_back(condition);
  }

  template<typename MessageT>
  void
  set_take_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::MessageCallbackFunction<MessageT> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
    const std::vector<std::string> & query_parameters = std::vector<std::string>())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = notifier_.template take_data_callback<MessageT>(
      reader, cb, query_expression, data_state, query_parameters);
    query_conditions_.push_back(condition);
  }

  template<typename MessageT>
  void
  set_status_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::DataReaderStatusCallbackFunction<MessageT> cb,
    const dds::core::status::StatusMask enabled_statuses)
  {
    detach_status_callback<MessageT>(reader);
    auto condition = notifier_.template status_callback<MessageT>(reader, cb, enabled_statuses);
    status_conditions_readers_.push_back(condition);
  }

  template<typename MessageT>
  void
  set_status_callback(
    dds::pub::DataWriter<MessageT> writer,
    ros2dds::DataWriterStatusCallbackFunction<MessageT> cb,
    const dds::core::status::StatusMask enabled_statuses)
  {
    detach_status_callback<MessageT>(writer);
    auto condition = notifier_.template status_callback<MessageT>(writer, cb, enabled_statuses);
    status_conditions_writers_.push_back(condition);
  }

  dds::core::cond::GuardCondition
  add_user_callback(ros2dds::UserCallbackFunction cb)
  {
    return notifier_.user_callback(cb);
  }

  dds::core::cond::GuardCondition
  add_user_callback(ros2dds::SimpleUserCallbackFunction cb)
  {
    return notifier_.user_callback(cb);
  }

  void
  cancel(const dds::core::cond::GuardCondition & user_condition)
  {
    notifier_.detach(user_condition);
  }

  template<typename MessageT>
  void
  cancel_data_callback(const dds::sub::DataReader<MessageT> & reader)
  {
    detach_data_callback<MessageT>(reader);
  }

  template<typename MessageT>
  void
  cancel_status_callback(const dds::sub::DataReader<MessageT> & reader)
  {
    detach_status_callback<MessageT>(reader);
  }

  template<typename MessageT>
  void
  cancel_status_callback(const dds::pub::DataWriter<MessageT> & writer)
  {
    detach_status_callback<MessageT>(writer);
  }

protected:
  template<typename MessageT>
  void detach_data_callback(const dds::sub::DataReader<MessageT> & reader)
  {
    auto any_reader = dds::sub::AnyDataReader(reader);
    for (auto it = read_conditions_.begin(); it != read_conditions_.end(); ++it) {
      if ((*it).data_reader() == any_reader) {
        notifier_.detach(*it);
        read_conditions_.erase(it);
        return;
      }
    }
    for (auto it = query_conditions_.begin(); it != query_conditions_.end(); ++it) {
      if ((*it).data_reader() == any_reader) {
        notifier_.detach(*it);
        query_conditions_.erase(it);
        return;
      }
    }
  }

  template<typename MessageT>
  void detach_status_callback(const dds::sub::DataReader<MessageT> & reader)
  {
    auto any_reader = dds::sub::AnyDataReader(reader);
    for (auto it = status_conditions_readers_.begin();
      it != status_conditions_readers_.end(); ++it)
    {
      auto e_any_reader = dds::core::polymorphic_cast<dds::sub::AnyDataReader>((*it).entity());
      if (e_any_reader == any_reader) {
        notifier_.detach(*it);
        status_conditions_readers_.erase(it);
        return;
      }
    }
  }

  template<typename MessageT>
  void detach_status_callback(const dds::pub::DataWriter<MessageT> & writer)
  {
    auto any_writer = dds::pub::AnyDataWriter(writer);
    for (auto it = status_conditions_writers_.begin();
      it != status_conditions_writers_.end(); ++it)
    {
      auto e_any_writer = dds::core::polymorphic_cast<dds::pub::AnyDataWriter>((*it).entity());
      if (e_any_writer == any_writer) {
        notifier_.detach(*it);
        status_conditions_writers_.erase(it);
        return;
      }
    }
  }

private:
  const DdsNodeOptions node_options_;
  NotifierT notifier_;

  std::vector<dds::sub::cond::ReadCondition> read_conditions_;
  std::vector<dds::sub::cond::QueryCondition> query_conditions_;
  std::vector<dds::core::cond::StatusCondition> status_conditions_readers_;
  std::vector<dds::core::cond::StatusCondition> status_conditions_writers_;
};

}  // namespace rclcpp_dds


#endif  // RCLCPP_DDS__DDS_NODE_WRAPPER_HPP_