/**
 * @file CIBModule.hpp
 *
 * CIBModule is a DAQModule implementation that provides a command and readout interface of the Calibration Interface Board hardware. 
 * This hardware focus on the control and operation of the ionization laser calibration system (IoLS). 
 *
 * This is part of the DUNE DAQ Software Suite, copyright 2020.
 * Licensing/copyright details are in the COPYING file that you should have
 * received with this code.
 */

#ifndef CIBMODULES_PLUGINS_CIBMODULE_HPP_
#define CIBMODULES_PLUGINS_CIBMODULE_HPP_

#include "appfwk/DAQModule.hpp"
// FIXME: Implement this module
#include "appmodel/CIBModule.hpp" 
#include "iomanager/Receiver.hpp"
#include "iomanager/Sender.hpp"
#include "utilities/WorkerThread.hpp"

#include "hsilibs/HSIEventSender.hpp"

// FIXME: Implement this structure
#include "cibmodules/opmon/CIBModule.pb.h"

//#include "CTBPacketContent.hpp"

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <shared_mutex>
#include <map>
#include <deque>

#include <boost/asio.hpp>
#include <boost/array.hpp>

// #include <atomic>
// #include <limits>
// #include <string>

namespace dunedaq::cibmodules {

  // NFB: Do we need this?
  typedef std::pair<uint64_t, uint64_t> ts_payload; // NOLINT

  class CIBModule : public dunedaq::hsilibs::HSIEventSender
  {
  public:
    /**
     * @brief CIBModule Constructor
     * @param name Instance name for this CIBModule instance
     */
    explicit CIBModule(const std::string& name);
    // destructor
    ~CIBModule();

    void init(std::shared_ptr<appfwk::ConfigurationManager> cfgMgr) override;
    // void init(const nlohmann::json& iniobj) override;

    /**
     * Disallow copy and move constructors and assignments
     */
    CIBModule(const CIBModule&) = delete;
    CIBModule& operator=(const CIBModule&) = delete;
    CIBModule(CIBModule&&) = delete;
    CIBModule& operator=(CIBModule&&) = delete;


    bool error_state() const { return m_error_state.load(); }
//    bool ErrorState() const { return m_error_state.load() ; }
    // void get_info(opmonlib::InfoCollector& ci, int level) override;

  protected:
    void generate_opmon_data() override;

    private:

      // control variables
      std::atomic<bool> m_is_running;
      std::atomic<bool> m_is_configured;
      std::atomic<bool> m_stop_requested;

      std::string               m_receiver_host;
      /*const */ unsigned int   m_receiver_port;
      std::chrono::microseconds m_receiver_timeout;
      std::atomic<bool>         m_error_state;

      bool check_port_in_use(unsigned short port);

      boost::asio::io_service m_control_ios;
      boost::asio::ip::tcp::socket m_control_socket;
      boost::asio::ip::tcp::endpoint m_control_endpoint;

      boost::asio::io_service m_receiver_ios;
      boost::asio::ip::tcp::socket m_receiver_socket;

      std::shared_ptr<dunedaq::hsilibs::HSIEventSender::raw_sender_ct> m_cib_hsi_data_sender;

      // Commands
      void do_configure(const CommandData_t &obj) override;
      void do_start(const CommandData_t &startobj) override;
      void do_stop(const CommandData_t &obj) override;
      void do_scrap(const CommandData_t & /*obj*/) override {};

      // the CIB does not need reset, since the DAQ operation is
      // decoupled from the instrumentation operation
      void send_config(const std::string &config);
      bool send_message(const std::string &msg);

      // Configuration
      std::shared_ptr<appfwk::ConfigurationManager> m_cfg;
      using conf_t = appmodel::CIBModule;
      const conf_t *m_module = nullptr;

      std::atomic<daqdataformats::run_number_t> m_run_number;

      // Threading client
      dunedaq::utilities::WorkerThread m_thread_;
      void do_hsi_work(std::atomic<bool> &);

      // variables for geo_id to construct the HSI frame
      // These are defined as uint32 in the schema, but given the way the HSI frame is consctructed from this it is unsusable.
      // THe HSI frame uses 4 Bits for the slot and 10 Bits for the crate and 6 for the DetID. So here I'm overiding the types
      uint16_t m_det;   // NOLINT
      uint16_t m_crate; // NOLINT
      uint16_t m_slot;  // NOLINT

      // Generate HSI Frame/Event
      // void send_matched_trigger_word(const content::word::trigger_t &, uint64_t); // NOLINT
      // void match_between_buffers(std::queue<content::word::trigger_t> &, std::queue<ts_payload> &, uint64_t, content::word::word_type); // NOLINT
      // static bool check_repeated_word(ts_payload &, ts_payload &, uint64_t); // NOLINT

      template <typename T>
      bool read(boost::asio::ip::tcp::socket &socket, T &obj);

      //
      // members related to calibration stream
      //

      void update_calibration_file();
      void init_calibration_file();
      bool set_calibration_stream(const std::string &prefix = "");

      bool m_calibration_stream_enable = false;
      std::string m_calibration_dir = "";
      std::string m_calibration_prefix = "";
      std::chrono::minutes m_calibration_file_interval;
      std::ofstream m_calibration_file;
      std::chrono::steady_clock::time_point m_last_calibration_file_update;

      //
      // Other auxiliary members
      //

      bool parse_hex(std::string_view s, std::uint32_t &out);

      //
      // metric utilities
      //
      // -- these have to match the protobuf definition in CIBModuleInfo.proto
      using general_metric_t = dunedaq::cibmodules::opmon::CIBModuleInfo;
      using const_message_counter_t = std::invoke_result<decltype(&general_metric_t::num_control_messages_sent), general_metric_t>::type;

      using message_counter_t = std::remove_const<const_message_counter_t>::type;
      std::atomic<message_counter_t> m_num_control_messages_sent = 0;
      std::atomic<message_counter_t> m_num_control_responses_received = 0;

      // using message_status_t = std::invoke_result<decltype(&general_metric_t::hardware_running), general_metric_t>::type;

      // std::atomic<std::remove_const<message_status_t>::type> m_is_running = false;
      // std::atomic<std::remove_const<message_status_t>::type> m_is_configured = false;

      using const_total_trigger_counter_t = std::invoke_result<decltype(&general_metric_t::num_total_triggers_received), general_metric_t>::type;
      std::atomic<std::remove_const<const_total_trigger_counter_t>::type> m_num_total_triggers_received;

      using const_run_trigger_counter_t = std::invoke_result<decltype(&general_metric_t::num_run_triggers_received), general_metric_t>::type;
      std::atomic<std::remove_const<const_run_trigger_counter_t>::type> m_num_run_triggers_received;

      //
      //
      // monitoring data/information
      //
      std::deque<uint> m_buffer_counts; // NOLINT(build/unsigned)
      std::shared_mutex m_buffer_counts_mutex;
      void update_buffer_counts(uint new_count); // NOLINT(build/unsigned)
      double read_average_buffer_counts();

      //
      // DAQ-CIB communication statistics
      //

      //
      // trigger bit to be written into the HSI event
      //
      uint32_t m_trigger_bit;
      // flag to hold the start_run until the receiver is ready
      std::atomic<bool> m_receiver_ready;
  };

} // namespace dunedaq::cibmodules

#endif // CIBMODULES_PLUGINS_CIBMODULE_HPP_
