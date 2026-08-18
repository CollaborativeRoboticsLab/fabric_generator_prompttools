# Fabric Generator Plugin for PromptTools

`fabric_generator_prompttools` is a standalone ROS 2 plugin package that connects the Fabric stack to the PromptTools stack.

The purpose of this repository split is to keep the core Fabric repository focused on Fabric itself, while this package carries the prompt-generation integration logic and its prompt-side dependencies.

## What This Package Does

The package exports `fabric::PromptToolsGenerator` as a `fabric::GenerationBase` plugin through `pluginlib`.

At runtime the plugin:

1. Requests runnable capability specifications from `capabilities2/get_runnable_specs`.
2. Builds a prompt that asks for a valid Fabric XML plan for the task.
3. Sends that prompt to `prompt/prompt`.
4. Parses the returned JSON payload into a `fabric::Plan`.

## Dependency Boundary

This repository is expected to depend on both the Fabric side and the prompt-generation side:

- `fabric_base`
- `rclcpp`
- `pluginlib`
- `nlohmann_json`
- `capabilities2_msgs`
- `prompt_msgs`

This package is intentionally the integration point. The Fabric core packages can stay isolated from `prompt_msgs` when this plugin lives in its own repository.

`capabilities2_msgs` is still expected here because the generator asks the capabilities server for runnable specs before composing the prompt.

## Build

Build this package in a workspace where Fabric base packages, capabilities2 interfaces, and prompt interfaces are available:

```bash
cd ~/colcon_ws

git clone https://github.com/CollaborativeRoboticsLab/capabilities2.git
git clone https://github.com/CollaborativeRoboticsLab/fabric.git
git clone https://github.com/CollaborativeRoboticsLab/prompt_tools.git
git clone https://github.com/CollaborativeRoboticsLab/fabric_generator_prompttools.git

colcon build
```

## Runtime Expectations

For the plugin to initialize and generate plans successfully, these components must already be available:

- Fabric plugin infrastructure
- The `capabilities2/get_runnable_specs` service
- The `prompt/prompt` service
- The `capabilities2_msgs/srv/GetRunnableSpecs` interface
- The `prompt_msgs/srv/Prompt` interface

The plugin currently reads these parameters from the hosting node:

- `PromptToolsGenerator.model_family`
- `PromptToolsGenerator.model`
- `PromptToolsGenerator.use_cache`
- `PromptToolsGenerator.use_chat_mode`

If either required service is unavailable, initialization fails after the service wait timeout.

## Repository Intent

Keep PromptTools-specific generation logic in this repository. Keep Fabric core packages free of prompt stack build dependencies whenever possible.
