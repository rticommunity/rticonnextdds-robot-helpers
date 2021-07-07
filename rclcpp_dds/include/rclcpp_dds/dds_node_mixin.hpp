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

#ifndef RCLCPP_DDS__DDS_NODE_MIXIN_HPP_
#define RCLCPP_DDS__DDS_NODE_MIXIN_HPP_

#include <string>
#include <vector>
#include <memory>

#include "ros2dds/ros2dds.hpp"

#include "rclcpp_dds/dds_node_options.hpp"
#include "rclcpp_dds/visibility_control.hpp"

namespace rclcpp_dds
{

// template<
//   typename NodeT,
//   typename ExecutorT>
template<typename NodeT>
class DDSNodeMixin : public NodeT
{
public:
  using TopicKind = ros2dds::TopicKind;

  explicit DDSNodeMixin(
    const std::string & node_name,
    const DDSNodeOptions & options = DDSNodeOptions())
  : NodeT(node_name, options),
    node_options_(options)
  {
    if (nullptr != options.dds_executor()) {
      exec_ = options.dds_executor();
    } else {
      exec_ = std::make_shared<ros2dds::AsyncWaitSetExecutor>();
    }
  }

  explicit DDSNodeMixin(
    const std::string & node_name,
    const std::string & namespace_,
    const DDSNodeOptions & options = DDSNodeOptions())
  : NodeT(node_name, namespace_, options),
    node_options_(options)
  {
    if (nullptr != options.dds_executor()) {
      exec_ = options.dds_executor();
    } else {
      exec_ = std::make_shared<ros2dds::AsyncWaitSetExecutor>();
    }
  }

  virtual ~DDSNodeMixin() = default;

  /**
   * @brief Access the DDS domain joined by this node.
   *
   * @return An unsigned integer identifying the DDS domain joined by this node.
   */
  size_t domain_id()
  {
    return ros2dds::domain_id(*this);
  }

  /**
   * @brief Access the DDS DomainParticipant associated with this node.
   *
   * @return The dds::domain::DomainParticipant instance associated with this node.
   */
  dds::domain::DomainParticipant &
  domain_participant()
  {
    // Cache domain participant for later lookups.
    if (nullptr == domain_participant_) {
      domain_participant_ = ros2dds::domain_participant(*this);
    }
    return domain_participant_;
  }

  template<typename MessageT>
  dds::topic::Topic<MessageT>
  create_topic(
    const std::string & topic_name,
    const std::string & custom_type_name = std::string())
  {
    return ros2dds::create_topic<MessageT>(
      *this,
      topic_name,
      TopicKind::Topic,
      node_options_.use_ros_naming_conventions(),
      custom_type_name);
  }

  template<typename MessageT>
  dds::topic::Topic<MessageT>
  create_service_topic(
    const std::string & service_name,
    const bool request = true,
    const std::string & custom_type_name = std::string())
  {
    return ros2dds::create_topic<MessageT>(
      *this,
      service_name,
      (request) ? TopicKind::Request : TopicKind::Reply,
      node_options_.use_ros_naming_conventions(),
      custom_type_name);
  }

  template<typename MessageT>
  dds::topic::Topic<MessageT>
  lookup_topic(const std::string & topic_name)
  {
    return ros2dds::lookup_topic<MessageT>(
      *this,
      topic_name,
      TopicKind::Topic,
      node_options_.use_ros_naming_conventions());
  }

  template<typename MessageT>
  dds::topic::Topic<MessageT>
  lookup_service_topic(
    const std::string & service_name,
    const bool request = true)
  {
    return ros2dds::lookup_topic<MessageT>(
      *this,
      service_name,
      (request) ? TopicKind::Request : TopicKind::Reply,
      node_options_.use_ros_naming_conventions());
  }

  template<typename MessageT>
  dds::topic::ContentFilteredTopic<MessageT>
  create_content_filtered_topic(
    dds::topic::Topic<MessageT> & base_topic,
    const std::string & filter_name,
    const std::string & filter_expression,
    const std::vector<std::string> & filter_parameters = std::vector<std::string>())
  {
    return ros2dds::create_content_filtered_topic<MessageT>(
      *this,
      base_topic,
      filter_name,
      filter_expression,
      filter_parameters);
  }

  template<typename MessageT>
  dds::topic::ContentFilteredTopic<MessageT>
  lookup_content_filtered_topic(const std::string & filter_name)
  {
    return ros2dds::lookup_content_filtered_topic<MessageT>(*this, filter_name);
  }

  /**
   * @brief
   *
   * @param fq_topic_name
   * @return dds::pub::qos::DataWriterQos
   */
  dds::pub::qos::DataWriterQos
  get_default_datawriter_qos(const std::string & fq_topic_name = std::string())
  {
    if (fq_topic_name.length() > 0) {
      return ros2dds::default_datawriter_qos(
        *this,
        fq_topic_name,
        false /* expand */);
    } else {
      return ros2dds::default_datawriter_qos(*this);
    }
  }

  /**
   * @brief
   *
   * @param fq_topic_name
   * @return dds::sub::qos::DataReaderQos
   */
  dds::sub::qos::DataReaderQos
  get_default_datareader_qos(const std::string & fq_topic_name = std::string())
  {
    if (fq_topic_name.length() > 0) {
      return ros2dds::default_datareader_qos(
        *this,
        fq_topic_name,
        false /* expand */);
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
  get_datawriter_qos_profile(const std::string & qos_profile)
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
  get_datareader_qos_profile(const std::string & qos_profile)
  {
    // TODO(asorbini) support get_w_topic_name()
    return ros2dds::datareader_qos_profile(*this, qos_profile);
  }

  template<typename MessageT>
  dds::pub::DataWriter<MessageT>
  create_datawriter(
    const std::string & topic_name,
    const std::string & type_name = std::string())
  {
    return ros2dds::create_datawriter<MessageT>(
      *this,
      topic_name,
      TopicKind::Topic,
      node_options_.use_ros_naming_conventions(),
      type_name);
  }

  template<typename MessageT>
  dds::pub::DataWriter<MessageT>
  create_datawriter(
    const std::string & topic_name,
    const dds::pub::qos::DataWriterQos & qos,
    const std::string & type_name = std::string())
  {
    return ros2dds::create_datawriter<MessageT>(
      *this,
      topic_name,
      qos,
      TopicKind::Topic,
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

  /////

  template<typename MessageT>
  dds::pub::DataWriter<MessageT>
  create_service_datawriter(
    const std::string & topic_name,
    const bool request = true,
    const std::string & custom_type_name = std::string())
  {
    return ros2dds::create_datawriter<MessageT>(
      *this,
      topic_name,
      (request) ? TopicKind::Request : TopicKind::Reply,
      node_options_.use_ros_naming_conventions(),
      custom_type_name);
  }

  template<typename MessageT>
  dds::pub::DataWriter<MessageT>
  create_service_datawriter(
    const std::string & topic_name,
    const dds::pub::qos::DataWriterQos & qos,
    const bool request = false,
    const std::string & custom_type_name = std::string())
  {
    return ros2dds::create_datawriter<MessageT>(
      *this,
      topic_name,
      qos,
      (request) ? TopicKind::Request : TopicKind::Reply,
      node_options_.use_ros_naming_conventions(),
      custom_type_name);
  }
  /////

  template<typename MessageT>
  dds::sub::DataReader<MessageT>
  create_datareader(
    const std::string & topic_name,
    const std::string & custom_type_name = std::string())
  {
    return ros2dds::create_datareader<MessageT>(
      *this,
      topic_name,
      TopicKind::Topic,
      node_options_.use_ros_naming_conventions(),
      custom_type_name);
  }

  template<typename MessageT>
  dds::sub::DataReader<MessageT>
  create_datareader(
    const std::string & topic_name,
    const dds::sub::qos::DataReaderQos & qos,
    const std::string & custom_type_name = std::string())
  {
    return ros2dds::create_datareader<MessageT>(
      *this,
      topic_name,
      qos,
      TopicKind::Topic,
      node_options_.use_ros_naming_conventions(),
      custom_type_name);
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

  ////
  template<typename MessageT>
  dds::sub::DataReader<MessageT>
  create_service_datareader(
    const std::string & topic_name,
    const bool request = true,
    const std::string & custom_type_name = std::string())
  {
    return ros2dds::create_datareader<MessageT>(
      *this,
      topic_name,
      (request) ? TopicKind::Request : TopicKind::Reply,
      node_options_.use_ros_naming_conventions(),
      custom_type_name);
  }

  template<typename MessageT>
  dds::sub::DataReader<MessageT>
  create_service_datareader(
    const std::string & topic_name,
    const dds::sub::qos::DataReaderQos & qos,
    const bool request = true,
    const std::string & custom_type_name = std::string())
  {
    return ros2dds::create_datareader<MessageT>(
      *this,
      topic_name,
      qos,
      (request) ? TopicKind::Request : TopicKind::Reply,
      node_options_.use_ros_naming_conventions(),
      custom_type_name);
  }
  ////

  template<typename MessageT>
  void
  set_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::condition::DataCallback<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = ros2dds::condition::template data<MessageT>(reader, cb, data_state);
    attach_condition(condition);
    read_conditions_.push_back(std::move(condition));
  }

  template<typename MessageT>
  void
  set_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::condition::DataQueryCallback<MessageT> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
    const std::vector<std::string> & query_parameters = std::vector<std::string>())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = ros2dds::condition::template query<MessageT>(
      reader, cb, query_expression, data_state, query_parameters);
    attach_condition(condition);
    query_conditions_.push_back(std::move(condition));
  }

  template<typename MessageT, bool Take = false>
  void
  set_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::condition::MessageCallback<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = ros2dds::condition::template read_data<MessageT, Take>(reader, cb, data_state);
    attach_condition(condition);
    read_conditions_.push_back(std::move(condition));
  }

  template<typename MessageT, bool Take = false>
  void
  set_data_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::condition::MessageCallback<MessageT> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
    const std::vector<std::string> & query_parameters = std::vector<std::string>())
  {
    detach_data_callback<MessageT>(reader);
    auto condition = ros2dds::condition::template read_query<MessageT, Take>(
      reader, cb, query_expression, data_state, query_parameters);
    attach_condition(condition);
    query_conditions_.push_back(std::move(condition));
  }

  template<typename MessageT>
  void
  set_status_callback(
    dds::sub::DataReader<MessageT> reader,
    ros2dds::condition::DataReaderStatusCallback<MessageT> cb,
    const dds::core::status::StatusMask enabled_statuses)
  {
    detach_status_callback<MessageT>(reader);
    auto condition = ros2dds::condition::template status<MessageT>(reader, cb, enabled_statuses);
    attach_condition(condition);
    status_conditions_readers_.push_back(std::move(condition));
  }

  template<typename MessageT>
  void
  set_status_callback(
    dds::pub::DataWriter<MessageT> writer,
    ros2dds::condition::DataWriterStatusCallback<MessageT> cb,
    const dds::core::status::StatusMask enabled_statuses)
  {
    detach_status_callback<MessageT>(writer);
    auto condition = ros2dds::condition::template status<MessageT>(writer, cb, enabled_statuses);
    attach_condition(condition);
    status_conditions_writers_.push_back(std::move(condition));
  }

  template<typename CallbackT>
  dds::core::cond::GuardCondition
  add_user_callback(CallbackT cb)
  {
    auto condition = ros2dds::condition::guard(cb);
    attach_condition(condition);
    guard_conditions_.push_back(condition);
    return condition;
  }

  void
  cancel_user_callback(const dds::core::cond::GuardCondition & user_condition)
  {
    for (auto it = guard_conditions_.begin(); it != guard_conditions_.end(); ++it) {
      if (*it == user_condition) {
        detach_condition(*it);
        guard_conditions_.erase(it);
        return;
      }
    }
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

  RCLCPP_DDS_PUBLIC
  std::shared_ptr<ros2dds::WaitSetExecutor> &
  dds_executor()
  {
    return exec_;
  }

  RCLCPP_DDS_PUBLIC
  std::shared_ptr<ros2dds::WaitSetExecutor>
  dds_executor() const
  {
    return exec_;
  }

protected:
  void
  get_callback_conditions(std::vector<dds::core::cond::Condition> & conditions_seq)
  {
    for (auto && cond_ref : read_conditions_) {
      dds::core::cond::Condition cond = cond_ref;
      conditions_seq.push_back(std::move(cond));
    }
    for (auto && cond_ref : query_conditions_) {
      dds::core::cond::Condition cond = cond_ref;
      conditions_seq.push_back(std::move(cond));
    }
    for (auto && cond_ref : status_conditions_readers_) {
      dds::core::cond::Condition cond = cond_ref;
      conditions_seq.push_back(std::move(cond));
    }
    for (auto && cond_ref : status_conditions_writers_) {
      dds::core::cond::Condition cond = cond_ref;
      conditions_seq.push_back(std::move(cond));
    }
  }

  void
  attach_condition(const dds::core::cond::Condition & condition)
  {
    exec_->attach(condition);
  }

  void
  detach_condition(const dds::core::cond::Condition & condition)
  {
    exec_->detach(condition);
  }

  template<typename MessageT>
  bool detach_data_callback(const dds::sub::DataReader<MessageT> & reader)
  {
    auto any_reader = dds::sub::AnyDataReader(reader);
    for (auto it = read_conditions_.begin(); it != read_conditions_.end(); ++it) {
      if ((*it).data_reader() == any_reader) {
        detach_condition(*it);
        read_conditions_.erase(it);
        return true;
      }
    }
    for (auto it = query_conditions_.begin(); it != query_conditions_.end(); ++it) {
      if ((*it).data_reader() == any_reader) {
        detach_condition(*it);
        query_conditions_.erase(it);
        return true;
      }
    }
    return false;
  }

  template<typename MessageT>
  bool detach_status_callback(const dds::sub::DataReader<MessageT> & reader)
  {
    auto any_reader = dds::sub::AnyDataReader(reader);
    for (auto it = status_conditions_readers_.begin();
      it != status_conditions_readers_.end(); ++it)
    {
      auto e_any_reader = dds::core::polymorphic_cast<dds::sub::AnyDataReader>((*it).entity());
      if (e_any_reader == any_reader) {
        detach_condition(*it);
        status_conditions_readers_.erase(it);
        return true;
      }
    }
    return false;
  }

  template<typename MessageT>
  bool detach_status_callback(const dds::pub::DataWriter<MessageT> & writer)
  {
    auto any_writer = dds::pub::AnyDataWriter(writer);
    for (auto it = status_conditions_writers_.begin();
      it != status_conditions_writers_.end(); ++it)
    {
      auto e_any_writer = dds::core::polymorphic_cast<dds::pub::AnyDataWriter>((*it).entity());
      if (e_any_writer == any_writer) {
        detach_condition(*it);
        status_conditions_writers_.erase(it);
        return true;
      }
    }
    return false;
  }

private:
  const DDSNodeOptions node_options_;

  dds::domain::DomainParticipant domain_participant_{nullptr};
  // std::vector<dds::sub::AnyDataReader> datareaders_;
  // std::vector<dds::pub::AnyDataWriter> datawriters_;

  std::shared_ptr<ros2dds::WaitSetExecutor> exec_;

  std::vector<dds::sub::cond::ReadCondition> read_conditions_;
  std::vector<dds::sub::cond::QueryCondition> query_conditions_;
  std::vector<dds::core::cond::StatusCondition> status_conditions_readers_;
  std::vector<dds::core::cond::StatusCondition> status_conditions_writers_;
  std::vector<dds::core::cond::GuardCondition> guard_conditions_;
};

}  // namespace rclcpp_dds


#endif  // RCLCPP_DDS__DDS_NODE_MIXIN_HPP_
