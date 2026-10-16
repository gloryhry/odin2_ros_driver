# SDK 3.3.0 sends TCP subscriptions before listening and registers command
# replies after sending. Adapt build-tree copies for both ordering races;
# the supplier tree, headers, public API and all other sources remain intact.
function(_odin_replace_once variable before after)
  set(source "${${variable}}")
  string(LENGTH "${source}" original_length)
  string(REPLACE "${before}" "" remaining "${source}")
  string(LENGTH "${remaining}" remaining_length)
  string(LENGTH "${before}" match_length)
  math(EXPR removed "${original_length} - ${remaining_length}")
  if(NOT removed EQUAL match_length)
    message(FATAL_ERROR "Odin SDK network compatibility source mismatch; review the adapter for this SDK version")
  endif()
  string(REPLACE "${before}" "${after}" source "${source}")
  set(${variable} "${source}" PARENT_SCOPE)
endfunction()

set(_odin_device "${CMAKE_CURRENT_SOURCE_DIR}/sdk_api/sdk/device/Odin2/odin2.cpp")
set(_odin_command "${CMAKE_CURRENT_SOURCE_DIR}/sdk_api/sdk/command/odin2Command/OdinCommand.cpp")
set(_odin_channel "${CMAKE_CURRENT_SOURCE_DIR}/sdk_api/sdk/src/odin_command_channel.cpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${_odin_device}" "${_odin_command}" "${_odin_channel}")
file(READ "${_odin_device}" _odin_device_source)
file(READ "${_odin_command}" _odin_command_source)
file(READ "${_odin_channel}" _odin_channel_source)

_odin_replace_once(_odin_device_source [[  // Step 1: Send start-stream command. The command layer owns local-side
  // endpoint allocation; the device never sees the underlying port.
  OdinResult result = command_->SendStartStream(channel, transport, mode, timeout_ms);
  if (result != OdinResult::kOk) {
    LOG_ERROR("StartStream: SendStartStream failed\n");
    return result;
  }

]] "")
# Destruction must stop callbacks before freeing their assembler, including
# exception paths before ownership is transferred into the device maps.
_odin_replace_once(_odin_device_source [[  auto capture = std::make_unique<NetworkCapture>(transport);
  auto assembler = std::make_unique<OdinFrameAssembler>(handle_, transport);]]
[[  auto assembler = std::make_unique<OdinFrameAssembler>(handle_, transport);
  auto capture = std::make_unique<NetworkCapture>(transport);]])
_odin_replace_once(_odin_device_source [[  // Start capture against the local endpoint chosen by the command layer.
  // The reference is opaque to the device (typed as ITransportAddress) and
  // is forwarded straight to capture without inspecting its fields.
  if (!capture->Start(command_->GetLocalEndpoint(channel), raw_cb)) {
    LOG_ERROR("StartStream: StartCapture failed\n");
    command_->SendCloseStream(channel, timeout_ms);
    return OdinResult::kSocketError;
  }]]
[[  // The driver adapter prepares the capture after endpoint allocation and
  // before ChannelConfig, so an immediate TCP connect can succeed.
  odin_driver::sdk_compat::PrepareCapture prepare(
      [&](const ITransportAddress& endpoint) { return capture->Start(endpoint, raw_cb); });
  OdinResult result = command_->SendStartStream(channel, transport, mode, timeout_ms);
  if (result != OdinResult::kOk) {
    capture->Stop();
    // A timeout may still have configured the device. Cancel best-effort.
    command_->SendCloseStream(channel, timeout_ms);
    return result;
  }]])

_odin_replace_once(_odin_command_source [[  auto payload = BuildChannelConfigPayload(channel, transport, dst_port, cfg);]]
[[  odin_driver::sdk_compat::ScopeExit release_port([&] {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    used_ports_.erase(dst_port);
  });
  NetworkAddress prepared_endpoint(host_ip_, dst_port);
  if (!odin_driver::sdk_compat::PrepareCapture::run(prepared_endpoint)) {
    LOG_ERROR("StartStream: failed to prepare local capture before subscription\n");
    return OdinResult::kSocketError;
  }
  auto payload = BuildChannelConfigPayload(channel, transport, dst_port, cfg);]])
_odin_replace_once(_odin_command_source [[    channel_endpoints_[channel] = endpoint;
  }
  return OdinResult::kOk;]]
[[    channel_endpoints_[channel] = endpoint;
  }
  release_port.dismiss();
  return OdinResult::kOk;]])

# A reply can arrive on the receive thread before Send() returns. Publish
# its waiter first, and remove it if sending fails or only writes a prefix.
_odin_replace_once(_odin_channel_source [[  // Send via transport (uses pre-configured remote target)
  int sent = transport_->Send(frame.data(), frame.size());
  if (sent < 0 || static_cast<size_t>(sent) != frame.size()) {
    return false;
  }

  PendingCommand pending;]] [[  PendingCommand pending;]])
_odin_replace_once(_odin_channel_source
  [[  pending.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);]]
  [[  // Start the timeout after Send(), as in the supplier implementation.
  pending.deadline = std::chrono::steady_clock::time_point::max();]])
_odin_replace_once(_odin_channel_source [[    pending_[seq] = pending;
  }

  return true;]] [[    pending_[seq] = pending;
  }

  // Register before sending so an immediate ACK cannot be discarded.
  int sent = transport_->Send(frame.data(), frame.size());
  if (sent < 0 || static_cast<size_t>(sent) != frame.size()) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.erase(seq);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto it = pending_.find(seq);
    if (it != pending_.end()) {
      it->second.deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(timeout_ms);
    }
  }

  return true;]])

# SendSync reports completion separately from response.result. Preserve a
# timeout/stopped result instead of interpreting its empty payload as an ACK.
foreach(_odin_operation SendStartStream SetAdaptiveParam GetAdaptiveParam)
  set(_odin_timeout [[    LOG_ERROR("OdinCommand::@operation@: timeout\n");
    return OdinResult::kTimeout;
  }
]])
  string(REPLACE "@operation@" "${_odin_operation}" _odin_timeout "${_odin_timeout}")
  _odin_replace_once(_odin_command_source "${_odin_timeout}" "${_odin_timeout}
  if (response.result != OdinResult::kOk) {
    return response.result;
  }
")
endforeach()

set(_odin_compat_dir "${CMAKE_CURRENT_BINARY_DIR}/sdk_stream_compat")
set(_odin_compat_include "#include \"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk_stream_compat.hpp\"\n")
file(MAKE_DIRECTORY "${_odin_compat_dir}")
function(_odin_write_compat path content)
  file(WRITE "${path}.tmp" "${content}")
  execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${path}.tmp" "${path}"
    RESULT_VARIABLE result)
  file(REMOVE "${path}.tmp")
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Cannot write Odin SDK stream compatibility source: ${path}")
  endif()
endfunction()
_odin_write_compat("${_odin_compat_dir}/odin2.cpp" "${_odin_compat_include}${_odin_device_source}")
_odin_write_compat("${_odin_compat_dir}/OdinCommand.cpp" "${_odin_compat_include}${_odin_command_source}")
_odin_write_compat("${_odin_compat_dir}/odin_command_channel.cpp" "${_odin_channel_source}")
get_target_property(_odin_sources odin_device_obj SOURCES)
get_target_property(_odin_source_dir odin_device_obj SOURCE_DIR)
set(_odin_adapted_sources)
foreach(source IN LISTS _odin_sources)
  get_filename_component(absolute "${source}" REALPATH BASE_DIR "${_odin_source_dir}")
  if(absolute STREQUAL _odin_device)
    list(APPEND _odin_adapted_sources "${_odin_compat_dir}/odin2.cpp")
  elseif(absolute STREQUAL _odin_command)
    list(APPEND _odin_adapted_sources "${_odin_compat_dir}/OdinCommand.cpp")
  else()
    list(APPEND _odin_adapted_sources "${absolute}")
  endif()
endforeach()
set_property(TARGET odin_device_obj PROPERTY SOURCES "${_odin_adapted_sources}")

get_target_property(_odin_sources odin_sdk SOURCES)
list(FIND _odin_sources "${_odin_channel}" _odin_channel_index)
if(_odin_channel_index EQUAL -1)
  message(FATAL_ERROR "Cannot locate Odin SDK command channel source")
endif()
list(REMOVE_AT _odin_sources ${_odin_channel_index})
list(APPEND _odin_sources "${_odin_compat_dir}/odin_command_channel.cpp")
set_property(TARGET odin_sdk PROPERTY SOURCES "${_odin_sources}")
