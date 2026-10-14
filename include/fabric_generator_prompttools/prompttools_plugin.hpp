#pragma once
#include <cerrno>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <fabric_base/generation_base.hpp>
#include <prompt_msgs/srv/prompt.hpp>
#include <capabilities2_msgs/srv/get_runnable_specs.hpp>

namespace fabric
{
/**
 * @brief Syntax validation plugin for XML plans.
 *
 * This plugin checks the XML plan for basic syntax errors (well-formedness, root tag, etc).
 */
class PromptToolsGenerator : public GenerationBase
{
  using Prompt = prompt_msgs::srv::Prompt;
  using RunnableSpec = capabilities2_msgs::srv::GetRunnableSpecs;

public:
  PromptToolsGenerator() = default;
  virtual ~PromptToolsGenerator() = default;

  /**
   * @brief Initialize the XML parser plugin.
   *
   * @param node Shared pointer to the ROS2 node.
   */
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    initialize_base(node, "PromptToolsGenerator");

    node_->declare_parameter<int>(plugin_name_ + ".service_wait_timeout_sec", 10);
    node_->declare_parameter<int>(plugin_name_ + ".prompt_request_timeout_sec", 90);
    service_wait_timeout_sec_ = node_->get_parameter(plugin_name_ + ".service_wait_timeout_sec").as_int();
    prompt_request_timeout_sec_ = node_->get_parameter(plugin_name_ + ".prompt_request_timeout_sec").as_int();

    node_->declare_parameter<std::string>(plugin_name_ + ".services.prompt", "/prompt/prompt");
    node_->declare_parameter<std::string>(plugin_name_ + ".services.get_runnable_specs", "/capabilities/get_runnable_specs");
    node_->get_parameter(plugin_name_ + ".services.prompt", prompt_service_name_);
    node_->get_parameter(plugin_name_ + ".services.get_runnable_specs", runnable_specs_service_name_);

    prompt_service_client_ = node_->create_client<Prompt>(prompt_service_name_);

    if (!prompt_service_client_->wait_for_service(std::chrono::seconds(service_wait_timeout_sec_)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Prompt service not available after waiting at %s", prompt_service_name_.c_str());
      throw fabric::fabric_exception("Prompt service not available after waiting");
    }

    runnable_spec_service_client_ = node_->create_client<RunnableSpec>(runnable_specs_service_name_);

    if (!runnable_spec_service_client_->wait_for_service(std::chrono::seconds(service_wait_timeout_sec_)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Runnable spec service not available after waiting at %s", runnable_specs_service_name_.c_str());
      throw fabric::fabric_exception("Runnable spec service not available after waiting");
    }

    node_->declare_parameter(plugin_name_ + ".model_family", "openai");
    node_->declare_parameter(plugin_name_ + ".model", "gpt-5.1");
    node_->declare_parameter(plugin_name_ + ".use_cache", true);
    node_->declare_parameter(plugin_name_ + ".use_chat_mode", false);

    node_->get_parameter(plugin_name_ + ".model_family", model_family_);
    node_->get_parameter(plugin_name_ + ".model", model_);
    node_->get_parameter(plugin_name_ + ".use_cache", use_cache_);
    node_->get_parameter(plugin_name_ + ".use_chat_mode", use_chat_mode_);
  }

  /**
   * @brief Generate a plan through the prompt service.
   *
   * @param document The XMLDocument representing the plan.
   * @param plan The parsed plan.
   */
  fabric::plan::Plan generate(const std::string& task, const std::string& uuid, bool flush) override
  {
    RCLCPP_INFO(node_->get_logger(), "PromptToolsGenerator starting generation: task='%s' uuid='%s' flush=%s model_family='%s' model='%s'",
                task.c_str(), uuid.c_str(), flush ? "true" : "false", model_family_.c_str(), model_.c_str());

    auto runnable_spec_request = std::make_shared<RunnableSpec::Request>();

    std::mutex block_mutex;
    std::condition_variable cv;
    bool specs_completed = false;
    bool prompt_completed = false;
    specs_response_.reset();
    prompt_response_.reset();

    {
      std::unique_lock<std::mutex> lock_spec(block_mutex);

      RCLCPP_INFO(node_->get_logger(), "PromptToolsGenerator requesting runnable specifications from %s", runnable_specs_service_name_.c_str());
      runnable_spec_service_client_->async_send_request(
          runnable_spec_request, [this, &specs_completed, &cv](typename rclcpp::Client<RunnableSpec>::SharedFuture future) {
            if (!future.valid())
            {
              RCLCPP_ERROR(node_->get_logger(), "PromptToolsGenerator runnable specification request failed");
            }
            else
            {
              specs_response_ = future.get();
              RCLCPP_INFO(node_->get_logger(), "PromptToolsGenerator received %zu runnable specifications", specs_response_->runnable_specs.size());
            }

            specs_completed = true;
            cv.notify_all();
          });

      const bool specs_ready =
          cv.wait_for(lock_spec, std::chrono::seconds(service_wait_timeout_sec_), [&specs_completed] { return specs_completed; });

      if (!specs_ready)
      {
        throw fabric::fabric_exception("PromptToolsGenerator timed out waiting for runnable specifications from " + runnable_specs_service_name_);
      }
    }

    if (!specs_response_)
    {
      throw fabric::fabric_exception("PromptToolsGenerator did not receive runnable specifications");
    }

    std::string capabilities_info = "Capabilities information:\n";

    for (std::size_t i = 0; i < specs_response_->runnable_specs.size(); ++i)
    {
      capabilities_info += "Capability: " + specs_response_->index_of_specs[i] + "\n";
      capabilities_info += "Description: " + specs_response_->runnable_specs[i].spec + "\n\n";
    }

    std::string text = "Build a xml plan based on the following capabilities to achieve the task: " + task +
                       ". Return the xml plan along with a brief reasoning summary for the plan in the response. The response should be in the "
                       "following json format: {\"plan\": \"<xml_plan>\", \"reasoning\": \"<brief_reasoning_summary>\"}. The xml plan should be "
                       "well-formed and valid.";

    auto prompt_request = std::make_shared<Prompt::Request>();

    prompt_request->uuid = uuid;
    prompt_request->prompt.prompt = capabilities_info + "\n" + text;

    prompt_request->prompt.model_family = "openai";
    prompt_request->prompt.use_cache = true;
    prompt_request->prompt.use_chat_mode = false;
    prompt_request->prompt.flush_cache = flush;

    prompt_msgs::msg::ModelOption modelOption1;
    modelOption1.key = "model";
    modelOption1.value = "gpt-5.1";

    prompt_request->prompt.options.push_back(modelOption1);

    prompt_msgs::msg::ModelOption modelOption2;
    modelOption2.key = "stream";
    modelOption2.value = "false";
    modelOption2.type = prompt_msgs::msg::ModelOption::BOOL_TYPE;

    prompt_request->prompt.options.push_back(modelOption2);

    {
      std::unique_lock<std::mutex> lock_prompt(block_mutex);

      RCLCPP_INFO(node_->get_logger(), "PromptToolsGenerator sending prompt request to %s", prompt_service_name_.c_str());
      prompt_service_client_->async_send_request(
          prompt_request, [this, &prompt_completed, &cv](typename rclcpp::Client<Prompt>::SharedFuture future) {
            if (!future.valid())
            {
              RCLCPP_ERROR(node_->get_logger(), "PromptToolsGenerator prompt request failed");
            }
            else
            {
              prompt_response_ = future.get();
              RCLCPP_INFO(node_->get_logger(), "PromptToolsGenerator received prompt response: success=%s buffered=%s response_size=%zu",
                          prompt_response_->response.success ? "true" : "false", prompt_response_->response.buffered ? "true" : "false",
                          prompt_response_->response.response.size());
            }

            prompt_completed = true;
            cv.notify_all();
          });

      const bool prompt_ready =
          cv.wait_for(lock_prompt, std::chrono::seconds(prompt_request_timeout_sec_), [&prompt_completed] { return prompt_completed; });

      if (!prompt_ready)
      {
        throw fabric::fabric_exception("PromptToolsGenerator timed out waiting for prompt response from " + prompt_service_name_);
      }
    }

    if (!prompt_response_)
    {
      throw fabric::fabric_exception("PromptToolsGenerator did not receive a prompt response");
    }

    if (!prompt_response_->response.success)
    {
      RCLCPP_ERROR(node_->get_logger(), "PromptToolsGenerator prompt request returned failure: %s", prompt_response_->response.response.c_str());
      throw fabric::fabric_exception("PromptToolsGenerator prompt request failed: " + prompt_response_->response.response);
    }

    RCLCPP_INFO(node_->get_logger(), "PromptToolsGenerator generation completed. Parsing prompt response.");

    fabric::plan::Plan plan;

    try
    {
      const auto parsed_response = nlohmann::json::parse(prompt_response_->response.response);

      plan.plan = parsed_response.at("plan").get<std::string>();
      if (parsed_response.contains("reasoning") && !parsed_response.at("reasoning").is_null())
      {
        plan.reasoning = parsed_response.at("reasoning").get<std::string>();
      }
    }
    catch (const nlohmann::json::exception& e)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse prompt response as JSON: %s", e.what());
      plan.plan = prompt_response_->response.response;
      plan.reasoning.clear();
    }

    return plan;
  }

protected:
  /**
   * @brief Shared pointer to the experience action client.
   */
  rclcpp::Client<Prompt>::SharedPtr prompt_service_client_;

  /**
   * @brief Shared pointer to the capabilities2 runnable spec service.
   */
  rclcpp::Client<RunnableSpec>::SharedPtr runnable_spec_service_client_;

  RunnableSpec::Response::SharedPtr specs_response_;
  Prompt::Response::SharedPtr prompt_response_;

  int service_wait_timeout_sec_;
  int prompt_request_timeout_sec_;
  std::string prompt_service_name_;
  std::string runnable_specs_service_name_;
  std::string model_family_;
  std::string model_;
  bool use_cache_;
  bool use_chat_mode_;
};
}  // namespace fabric
