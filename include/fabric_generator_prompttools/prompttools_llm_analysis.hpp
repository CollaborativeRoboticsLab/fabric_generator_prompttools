#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <fabric_base/llm_analysis_base.hpp>
#include <prompt_msgs/msg/model_option.hpp>
#include <prompt_msgs/srv/embedding.hpp>
#include <prompt_msgs/srv/prompt.hpp>
#include <prompt_msgs/srv/tokenize.hpp>

namespace fabric
{

class PromptToolsLlmAnalysis : public LlmAnalysisBase
{
  using Prompt = prompt_msgs::srv::Prompt;
  using Embedding = prompt_msgs::srv::Embedding;
  using Tokenize = prompt_msgs::srv::Tokenize;

public:
  void shutdown() override
  {
    shutting_down_ = true;
  }

  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    initialize_base(node, "PromptToolsLlmAnalysis");
    shutting_down_ = false;

    node_->declare_parameter<int>(plugin_name_ + ".service_wait_timeout_sec", 10);
    node_->declare_parameter<int>(plugin_name_ + ".request_timeout_sec", 90);
    node_->declare_parameter<std::string>(plugin_name_ + ".services.prompt", "/prompt/prompt");
    node_->declare_parameter<std::string>(plugin_name_ + ".services.embedding", "/prompt/embedding");
    node_->declare_parameter<std::string>(plugin_name_ + ".services.tokenize", "/prompt/tokenizer");

    service_wait_timeout_sec_ = node_->get_parameter(plugin_name_ + ".service_wait_timeout_sec").as_int();
    request_timeout_sec_ = node_->get_parameter(plugin_name_ + ".request_timeout_sec").as_int();
    prompt_service_name_ = node_->get_parameter(plugin_name_ + ".services.prompt").as_string();
    embedding_service_name_ = node_->get_parameter(plugin_name_ + ".services.embedding").as_string();
    tokenize_service_name_ = node_->get_parameter(plugin_name_ + ".services.tokenize").as_string();

    prompt_client_ = node_->create_client<Prompt>(prompt_service_name_);
    embedding_client_ = node_->create_client<Embedding>(embedding_service_name_);
    tokenize_client_ = node_->create_client<Tokenize>(tokenize_service_name_);

    waitForService(prompt_client_, prompt_service_name_);
    waitForService(embedding_client_, embedding_service_name_);
    waitForService(tokenize_client_, tokenize_service_name_);
  }

  LlmPromptResult prompt(const LlmPromptRequest& request) override
  {
    auto prompt_request = std::make_shared<Prompt::Request>();
    prompt_request->uuid = request.request_id;
    prompt_request->prompt.prompt = request.system_prompt.empty() ? request.user_prompt : request.system_prompt + "\n" + request.user_prompt;
    prompt_request->prompt.model_family = request.model_family;
    prompt_request->prompt.use_cache = request.use_cache;
    prompt_request->prompt.flush_cache = request.flush_cache;
    prompt_request->prompt.use_chat_mode = request.use_chat_mode;

    prompt_msgs::msg::ModelOption model_option;
    model_option.key = "model";
    model_option.value = request.model;
    model_option.type = prompt_msgs::msg::ModelOption::STRING_TYPE;
    prompt_request->prompt.options.push_back(model_option);

    prompt_msgs::msg::ModelOption stream_option;
    stream_option.key = "stream";
    stream_option.value = request.stream ? "true" : "false";
    stream_option.type = prompt_msgs::msg::ModelOption::BOOL_TYPE;
    prompt_request->prompt.options.push_back(stream_option);

    auto future = prompt_client_->async_send_request(prompt_request);
    LlmPromptResult result;
    if (!waitForResponse(future))
    {
      prompt_client_->remove_pending_request(future);
      result.error_message = "Prompt request timed out or was cancelled.";
      return result;
    }

    const auto response = future.get();
    result.success = response->response.success;
    result.buffered = response->response.buffered;
    result.response = response->response.response;
    if (!result.success)
    {
      result.error_message = response->response.response;
    }
    return result;
  }

  LlmEmbeddingResult embed(const LlmEmbeddingRequest& request) override
  {
    auto embedding_request = std::make_shared<Embedding::Request>();
    embedding_request->input.text = request.text;
    embedding_request->input.model_family = request.model_family;

    prompt_msgs::msg::ModelOption encoding_option;
    encoding_option.key = "encoding_format";
    encoding_option.value = "float";
    encoding_option.type = prompt_msgs::msg::ModelOption::STRING_TYPE;
    embedding_request->input.options.push_back(encoding_option);

    prompt_msgs::msg::ModelOption model_option;
    model_option.key = "model";
    model_option.value = request.model;
    model_option.type = prompt_msgs::msg::ModelOption::STRING_TYPE;
    embedding_request->input.options.push_back(model_option);

    if (request.embedding_dim > 0)
    {
      prompt_msgs::msg::ModelOption dimension_option;
      dimension_option.key = "dimensions";
      dimension_option.value = std::to_string(request.embedding_dim);
      dimension_option.type = prompt_msgs::msg::ModelOption::INT_TYPE;
      embedding_request->input.options.push_back(dimension_option);
    }

    auto future = embedding_client_->async_send_request(embedding_request);
    LlmEmbeddingResult result;
    if (!waitForResponse(future))
    {
      embedding_client_->remove_pending_request(future);
      result.error_message = "Embedding request timed out or was cancelled.";
      return result;
    }

    const auto response = future.get();
    if (response->output.embeddings.empty() || !response->output.embeddings.front().is_float)
    {
      result.error_message = "Embedding response did not include a float embedding.";
      return result;
    }

    result.success = true;
    result.embedding = response->output.embeddings.front().float_embedding;
    return result;
  }

  LlmTokenizationResult tokenize(const LlmTokenizationRequest& request) override
  {
    auto tokenize_request = std::make_shared<Tokenize::Request>();
    tokenize_request->input.model_family = request.model_family;
    tokenize_request->input.encode = request.mode == LlmTokenizationRequest::Mode::ENCODE;
    tokenize_request->input.text = request.text;
    tokenize_request->input.tokens = request.tokens;

    prompt_msgs::msg::ModelOption model_option;
    model_option.key = "model";
    model_option.value = request.model;
    model_option.type = prompt_msgs::msg::ModelOption::STRING_TYPE;
    tokenize_request->input.options.push_back(model_option);

    auto future = tokenize_client_->async_send_request(tokenize_request);
    LlmTokenizationResult result;
    if (!waitForResponse(future))
    {
      tokenize_client_->remove_pending_request(future);
      result.error_message = "Tokenization request timed out or was cancelled.";
      return result;
    }

    const auto response = future.get();
    result.success = response->output.success;
    result.text = response->output.text;
    result.tokens = response->output.tokens;
    if (!result.success)
    {
      result.error_message = response->output.error;
    }
    return result;
  }

private:
  template<typename Future>
  bool waitForResponse(Future& future)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(request_timeout_sec_);
    while (!shutting_down_ && rclcpp::ok(node_->get_node_base_interface()->get_context()))
    {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline)
      {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      }
      const auto next_check = std::min(deadline, now + std::chrono::milliseconds(100));
      if (future.wait_until(next_check) == std::future_status::ready)
      {
        return true;
      }
    }
    return false;
  }

  void waitForService(const rclcpp::Client<Prompt>::SharedPtr& client, const std::string& service_name)
  {
    if (!client->wait_for_service(std::chrono::seconds(service_wait_timeout_sec_)))
    {
      throw fabric_exception("Service not available after waiting: " + service_name);
    }
  }

  void waitForService(const rclcpp::Client<Embedding>::SharedPtr& client, const std::string& service_name)
  {
    if (!client->wait_for_service(std::chrono::seconds(service_wait_timeout_sec_)))
    {
      throw fabric_exception("Service not available after waiting: " + service_name);
    }
  }

  void waitForService(const rclcpp::Client<Tokenize>::SharedPtr& client, const std::string& service_name)
  {
    if (!client->wait_for_service(std::chrono::seconds(service_wait_timeout_sec_)))
    {
      throw fabric_exception("Service not available after waiting: " + service_name);
    }
  }

  std::atomic<bool> shutting_down_{false};
  int service_wait_timeout_sec_{ 10 };
  int request_timeout_sec_{ 90 };
  std::string prompt_service_name_;
  std::string embedding_service_name_;
  std::string tokenize_service_name_;
  rclcpp::Client<Prompt>::SharedPtr prompt_client_;
  rclcpp::Client<Embedding>::SharedPtr embedding_client_;
  rclcpp::Client<Tokenize>::SharedPtr tokenize_client_;
};

}  // namespace fabric