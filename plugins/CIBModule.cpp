/**
 * @file CIBModule.cpp
 *
 * Implementations of CIBModule's functions
 *
 * This is part of the DUNE DAQ Software Suite, copyright 2020.
 * Licensing/copyright details are in the COPYING file that you should have
 * received with this code.
 */
#include "confmodel/GeoId.hpp"

// these are the data types in the application model
#include "appmodel/CIBConf.hpp"
#include "appmodel/CIBCalibrationStream.hpp"
#include "appmodel/CIBoardConf.hpp"
#include "appmodel/CIBRandomTrigger.hpp"
#include "appmodel/CIBTrigger.hpp"
#include "appmodel/CIBSockets.hpp"
#include "appmodel/CIBReceiverSocket.hpp"

#include "CIBModule.hpp"
#include "CIBModuleIssues.hpp"

#include "iomanager/IOManager.hpp"
#include "logging/Logging.hpp"

#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <queue>
#include <utility>

#define CIB_DUNEDAQ 1
// we may need this to help parse the data format arriving from the CIB
#include <cib_data_fmt.h>

/**
 * @brief Name used by TRACE TLOG calls from this source file
 */
#define TRACE_NAME "CIBModule" // NOLINT
#define TLVL_ENTER_EXIT_METHODS 10
#define TLVL_CIB_INFO 5
#define TLVL_CIB_DEBUG 15

constexpr uint16_t CIB_HSI_FRAME_VERSION = 0x1; // NOLINT
namespace dunedaq::cibmodules {

  CIBModule::CIBModule(const std::string& name)
              : hsilibs::HSIEventSender(name)
                , m_is_running(false)
                , m_is_configured(false)
                , m_stop_requested(false)
                , m_receiver_port(8871)
                , m_receiver_timeout(70000) // 70 ms - we know that triggers will come at 10 Hz max
                , m_error_state(false)
                , m_control_ios()
                , m_control_socket(m_control_ios)
                , m_control_endpoint()
                , m_receiver_ios()
                , m_receiver_socket(m_receiver_ios)
                , m_thread_(std::bind(&CIBModule::do_hsi_work, this, std::placeholders::_1))
                , m_calibration_stream_enabled(false)
                , m_calibration_dir("")
                , m_calibration_prefix("")
                , m_calibration_file_interval(std::chrono::minutes(15))
                // metric utilities
                , m_total_trigger_counter(0)
                , m_run_trigger_counter(0)
                , m_num_total_triggers(0)

                , m_num_control_messages_sent(0)
                , m_num_control_responses_received(0)

                // unsure we actually need this
                , m_module_instance(0)
                , m_trigger_bit(0)
                , m_receiver_ready(false)
  {
    // we can infer the instance from the name
    TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Instantiating a cibmodule with argument [" << name << "]";
    register_command("conf", &CIBModule::do_configure);
    register_command("start", &CIBModule::do_start);
    register_command("stop", &CIBModule::do_stop);
    TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Leaving [" << name << "]constructor.";
  }

  CIBModule::~CIBModule()
  {
    if(m_is_running.load())
    {
      const CommandData_t stopobj;

      // const nlohmann::json stopobj;
      // this should also take care of closing the streaming socket
      do_stop(stopobj);
    }
    TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": Closing the control socket " << std::endl;
    m_control_socket.close() ;

  }

  void
  CIBModule::init(std::shared_ptr<appfwk::ConfigurationManager> cfgMgr)
  {
    TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Entering init() method";
    TLOG_DEBUG(TLVL_CIB_DEBUG) << get_name() << ": Init data :  ";// << cfgMgr->get_init_data().dump();

    // init the sender
    HSIEventSender::init(cfgMgr);

    // assign the local configuration manager to the argument
    m_cfg = cfgMgr;

    // get the configuration fragment for this module
    auto mdal = cfgMgr->get_dal<appmodel::CIBModule>(get_name());
    if (! mdal)
    {
      throw cibmodules::CIBConfigFailure(ERS_HERE, "Missing Module configuration for " + get_name());
    }

    // assign the module configuration
    m_module = mdal;

    // setting up connections
    auto iom = iomanager::IOManager::get();

    using hsi_frame_t = dunedaq::hsilibs::HSI_FRAME_STRUCT;
    for ( auto con : m_module->get_outputs() )
    {
      if ( con->get_data_type() == datatype_to_string<hsi_frame_t>() )
      {
        if ( con->UID().find("IoLS")!=std::string::npos)
        {
          m_cib_hsi_data_sender = iom->get_sender<hsi_frame_t>(con->UID());
        }
      } // if data type is HSI Frame
    } // loop over outputs

    TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Exiting init() method";
  }

  void
  CIBModule::do_configure(const CommandData_t &)
  {

    TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Entering CIB do_configure()";
    TLOG_DEBUG(TLVL_CIB_DEBUG) << get_name() << ": Received configuration fragment : " << args.dump();

    auto conf = m_module ->get_configuration();
    m_receiver_port = m_module->get_board()->get_sockets()->get_receiver()->get_port();
    m_receiver_timeout = std::chrono::microseconds( m_module->get_board()->get_sockets()->get_receiver()->get_timeout() ) ;
    auto hostname = conf->get_hostname();
    TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": Board receiver network location "
        << hostname << ':' << m_receiver_port << std::endl;
    
    // identify the trigger bit that this receiver is assigned to
    m_trigger_bit = conf->get_trigger_bit();
    m_module_instance = conf->get_instance();
    TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": Instance " << m_module_instance << " assigned to trigger bit " << m_trigger_bit
        << " ( 0x" << std::hex << m_trigger_bit << std::dec << ")";
    
    // init monitoring variables
    m_num_control_messages_sent = 0;
    m_num_control_responses_received = 0;

    // figure out the identifier of the CIB
    auto board = m_module->get_board();
    auto geo_id = board->get_geo_id();
    m_det = geo_id->get_detector_id();
    m_crate = geo_id->get_crate_id();
    m_slot = geo_id->get_slot_id();

    const auto& misc = board->get_misc();
    auto session = m_cfg->get_session();

    // init trigger counters
    m_run_trigger_counter.store(0);
    m_total_trigger_counter.store(0);

    // network connection to the CIB module
    boost::asio::ip::tcp::resolver resolver( m_control_ios );
    boost::asio::ip::tcp::resolver::query query( hostname,
        std::to_string(conf->get_control_connection_port()) ) ; //"np04-iols-cib-01", 8991
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query) ;

    m_control_endpoint = iter->endpoint();
    // attempt the connection.
    try
    {

      m_control_socket.connect(m_control_endpoint);
      m_control_socket.set_option(boost::asio::ip::tcp::no_delay(true));
    }
    catch (std::exception &e)
    {
      std::ostringstream msg("");
      msg << get_name() << "Exception caught while establishing connection to CIB : " << e.what();
      // do nothing more. Just exist
      m_is_configured.store(false);
      throw CIBCommunicationError(ERS_HERE, msg.str());
    }
    TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": Successfully connected to CIB control endpoint "
                              << hostname << ':' << conf->get_control_connection_port() << std::endl;

    // if necessary, set the calibration stream
    auto stream_conf = conf->get_calibration_stream();

    if (stream_conf)
    {
      m_calibration_stream_enable = true;
      m_calibration_dir = stream_conf->get_output_directory();
      m_calibration_file_interval = std::chrono::duration_cast<decltype(m_calibration_file_interval)>(std::chrono::seconds(stream_conf->get_update_period_s()));
    }

      // at this we have to find the hostname to tell the board where to send the data
      boost::asio::ip::tcp::resolver::query query_for_local(boost::asio::ip::host_name(), "");
      iter = resolver.resolve(query_for_local);

      // create the json string out of the config fragment
      nlohmann::json config;
      try
      {
        to_json(config, m_module->get_board()->get_cib_json(*session, iter->endpoint().address().to_string()));
        auto json_dump = config.dump();
        TLOG() << "Sending configuration: " << json_dump;
      }
      catch (nlohmann::json::exception &e)
      {
        std::ostringstream msg("");
        msg << get_name() << "Caught a JSON exception converting config fragment : " << e.what();
        m_is_configured.store(false);
        throw CIBModuleError(ERS_HERE, msg.str());
      }
      catch (std::exception &e)
      {
        std::ostringstream msg("");
        msg << get_name() << "Caught STD exception while converting config fragment : " << e.what();
        // do nothing more. Just exist
        m_is_configured.store(false);
        throw CIBModuleError(ERS_HERE, msg.str());
      }
      send_config(config.dump());
    }

    void
    CIBModule::do_start(const CommandData_t &startobj)
    {

      TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Entering do_start() method";
      // actually, the first thing to check is whether the CIB has been configured
      // if not, this won't work
      if (!m_is_configured.load())
      {
        throw CIBWrongState(ERS_HERE, "CIB has not been successfully configured.");
      }

      // Set this to false early so it doesn't interfere with the start
      m_stop_requested.store(false);
      m_run_number.store(startobj.at("run").get<daqdataformats::run_number_t>());
      m_total_trigger_counter.store(0);

      TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": Sending start of run command";
      m_thread_.start_working_thread();

      // NFB: There is a potential race condition here: the socket in the working thread
      // needs to be in place before the CIB receives order to send data, or we risk having a connection
      // failure, if for some reason the CIB attempts to connect before the working thread is ready to receive.
      if (m_calibration_stream_enable)
      {
        std::stringstream run;
        run << "run" << m_run_number.load();
        set_calibration_stream(run.str());
      }
      int cnt = 0;
      while (!m_receiver_ready.load())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cnt++;
        if (cnt > 50)
        {
          // the socket didn't get ready on time
          throw CIBModuleError(ERS_HERE, "Receiver socket timed out before becoming ready.");
        }
      }
      TLOG_DEBUG(TLVL_CIB_DEBUG) << get_name() << ": All ready to signal the CIB to start";

      nlohmann::json cmd;
      cmd["command"] = "start_run";
      cmd["run_number"] = m_run_number.load();

      if (send_message(cmd.dump()))
      {
        m_is_running.store(true);
        TLOG() << get_name() << ": CIB run started successfully";
      }
      else
      {
        throw CIBCommunicationError(ERS_HERE, "Unable to start CIB run");
      }

      TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Exiting do_start() method";
    }

    void
    CIBModule::do_stop(const CommandData_t & /*stopobj*/)
    {

      TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Entering do_stop() method";
      TLOG_DEBUG(TLVL_CIB_DEBUG) << get_name() << ": Sending stop run command" << std::endl;
      // Give the do_work thread a chance to stop before stopping the CIB,
      // otherwise we end up reading from an empty buffer
      m_stop_requested.store(true);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));

      if (send_message("{\"command\":\"stop_run\"}"))
      {
        TLOG() << get_name() << ": CIB run stopped successfully";
        m_is_running.store(false);
      }
      else
      {
        // failed to sent the message to stop the run.
        // stop the collecting thread and then throw, since that
        // attempts a cleaner exit
        m_thread_.stop_working_thread();

        throw CIBCommunicationError(ERS_HERE, "Unable to stop CIB");
      }
      //
      m_thread_.stop_working_thread();

      // -- print the counters for local info
      TLOG() << get_name() << ": CIB trigger counter summary after run [" << m_run_number << "]:\n\n"
             << "IOLS trigger counter in run : " << m_run_trigger_counter << "\n"
             << "Global IOLS trigger count   : " << m_num_total_triggers << std::endl;

      // reset counters
      m_run_trigger_counter = 0;

      TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Exiting do_stop() method";
    }

  // this method is completely new
  // in fact, it is where most of the work is really done
  void
  CIBModule::do_hsi_work(std::atomic<bool>& running_flag)
  {
    TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Entering do_work() method";

    std::size_t n_bytes = 0 ;
    std::size_t n_words = 0 ;
    std::size_t prev_seq = 0 ;
    bool first = true;

    //connect to socket
    // should we keep everything local or under the class?
    boost::system::error_code ec;
    //boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(),m_receiver_port )

    unsigned short port = m_receiver_port;

    // check that this port is still available
    while(check_port_in_use(port))
    {
      port++;
    }
    // check if the port is different from the configured one
    if (port != m_receiver_port)
    {
      std::ostringstream msg("");
      msg << "Listener port [" << m_receiver_port << "] is in use. Relocating to port [" << port << "]";
      ers::warning(CIBMessage(ERS_HERE, msg.str()));
      m_receiver_port = port;
    }
    else
    {
      TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": Will set up the listener on port " << port << std::endl;
    }

    boost::asio::ip::tcp::acceptor acceptor(m_receiver_ios,boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(),port ));
    TLOG_DEBUG(0) << get_name() << ": Waiting for an incoming connection on port " << m_receiver_port << std::endl;

    acceptor.listen(boost::asio::ip::tcp::socket::max_connections, ec);
    if (ec)
    {
      std::ostringstream msg("");
      msg << get_name() << ": CIB got an error listening on socket: :" << port << " -- reason: '" << ec << "'";
      throw CIBCommunicationError(ERS_HERE,msg.str());
      return;
    }
    else
    {
      TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": Waiting for an incoming connection on port " << port << std::endl;
    }

//    std::future<void> accepting = async( std::launch::async, [&]{ acceptor.accept(receiver_socket,ec) ; } ) ;
    std::future<void> accepting = async( std::launch::async, [&]{ acceptor.accept(m_receiver_socket,ec) ; } ) ;
    if (ec)
    {
      std::stringstream msg;
      msg << "Socket opening failed:: " << ec.message();
      ers::error(CIBCommunicationError(ERS_HERE,msg.str()));
      return;
    }
    //
    m_receiver_ready.store(true);

    while ( running_flag.load()  && !m_stop_requested.load() )
    {
      if ( accepting.wait_for( m_receiver_timeout ) == std::future_status::ready )
      {
        break ;
      }
//      else
//      {
//        TLOG_DEBUG(TLVL_DEBUG_MEDIUM) << "Waiting for a bit longer";
//      }
    }

    TLOG_DEBUG(TLVL_CIB_DEBUG) << get_name() <<  ": Connection received: start reading" << std::endl;

    // -- A couple of variables to help in the data parsing
    /**
     * The structure is a bit different than in the CTB
     * The TCP packet contains a single word (the trigger)
     * But other than that, everything are triggers
     */

    dunedaq::cib::daq::iols_tcp_packet_t tcp_packet;

    //boost::system::error_code receiving_error;
    bool connection_closed = false ;

    while (running_flag.load() && !m_stop_requested.load())
    {
      update_calibration_file();

      if ( ! read(m_receiver_socket, tcp_packet ) )
      {
        connection_closed = true ;
        break;
      }

      n_bytes = tcp_packet.header.packet_size ;
      n_words = n_bytes/sizeof(dunedaq::cib::daq::iols_trigger_t);

      if (n_words != 1)
      {
        std::ostringstream msg("");
        msg << "Received more than one IoLS trigger word at once! This should never happen. Got "
            << n_bytes << " (expected " << sizeof(dunedaq::cib::daq::iols_trigger_t) << ")";
        ers::warning(CIBMessage(ERS_HERE, msg.str()));
      }
      // the CIB only ships one word per packet....so this error should be impossible
      // check continuity of the sequence numbers
      if (first)
      {
        // first word being fetched. The sequence number should be zero
        if (tcp_packet.header.sequence_id != 0)
        {
          std::ostringstream msg("");
          msg << "Missing sequence. First word should have sequence number 0. Got " << static_cast<int>(tcp_packet.header.sequence_id );
          ers::warning(CIBMessage(ERS_HERE, msg.str()));
        }
        first = false;
      }
      else
      {
        bool failed = false;
        // in case it rolled over, compare to 255
        if (tcp_packet.header.sequence_id == 0)
        {
          if (prev_seq != 255)
          {
            failed = true;
          }
        }
        else
        {
          if (tcp_packet.header.sequence_id != (prev_seq+1))
          {
            failed = true;
          }
        }
        if (failed)
        {
          std::ostringstream msg("");
          msg << "Skipped CIB word sequence. Prev word " << prev_seq << " current word " << static_cast<int>(tcp_packet.header.sequence_id );
          ers::warning(CIBMessage(ERS_HERE, msg.str()));
        }
      }
      prev_seq = tcp_packet.header.sequence_id;

      update_buffer_counts(n_words);


      if ( m_calibration_stream_enable )
      {
        m_calibration_file.write( reinterpret_cast<const char*>( & tcp_packet.word ), sizeof(tcp_packet.word) ) ; // NOLINT
        m_calibration_file.flush() ;
      } // word printing in calibration stream

      TLOG_DEBUG(TLVL_CIB_DEBUG) << get_name() << "Received IoLS trigger word!";
      ++m_num_total_triggers;
      ++m_run_trigger_counter;
      m_last_readout_timestamp = tcp_packet.word.timestamp;

      // we do not need to know anything else
      // ideally, one could add other information such as the direction
      // this should be coming packed in the trigger word
      // note, however, that to reconstruct the trace direction we also would need the source position
      // and that we cannot afford to send, so we can just make it up out of the  IoLS system
      //
      // Send HSI data to a DLH
      std::array<uint32_t, 7> hsi_struct;
      hsi_struct[0] = (0x1 << 26)     |  // some random bit that could indicate the type of frame
                      (m_slot << 22)  |  // slot number
                      (m_crate << 12) |  // crate number
                      (m_det << 6)    |  // detector number
                      CIB_HSI_FRAME_VERSION; 
      // timestamps -  I like the explicit masking here to mistakes
      hsi_struct[1] = tcp_packet.word.timestamp & 0xFFFFFFFF;       // ts low
      hsi_struct[2] = tcp_packet.word.timestamp >> 32;              // ts high

      // we shall use these 2 sets of 32 bits to define the periscope position
      // pos_m3 == linear stage
      hsi_struct[3] = tcp_packet.word.pos_m3; // lower 32b 0
      // pos_m3 == RNN600
      hsi_struct[4] = tcp_packet.word.pos_m2_msb << 15 | tcp_packet.word.pos_m2_lsb; // upper 32b
      /**
       * A note about the 5th entry
       * The trigger bit is actually mapped into a single bit, that is then remapped back
       * into an index
       */
      hsi_struct[5] = m_trigger_bit;            // trigger_map;
      hsi_struct[6] = m_run_trigger_counter;    // m_generated_counter;

      TLOG_DEBUG(TLVL_CIB_DEBUG) << get_name() << ": Formed HSI_FRAME_STRUCT for hlt "
          << std::hex
          << "0x"   << hsi_struct[0]
          << ", 0x" << hsi_struct[1]
          << ", 0x" << hsi_struct[2]
          << ", 0x" << hsi_struct[3]
          << ", 0x" << hsi_struct[4]
          << ", 0x" << hsi_struct[5]
          << ", 0x" << hsi_struct[6]
          << "\n";

      send_raw_hsi_data(hsi_struct, m_cib_hsi_data_sender.get());

      // TODO Nuno Barros Apr-02-2024 : properly fill device id
      // still need to figure this one out.
      dfmessages::HSIEvent event(m_det,
                                 m_trigger_bit,
                                 tcp_packet.word.timestamp,
                                 m_run_trigger_counter,
                                 m_run_number);

      send_hsi_event(event);

      if ( connection_closed )
      {
        break ;
      }
    }

    // // Make sure CTB run stops before closing socket
    // while (m_is_running.load())
    // {
    //   std::this_thread::sleep_for(std::chrono::microseconds(100));
    // }

    boost::system::error_code closing_error;
    // if the system is already in an error state
    // we should call for a socket shutdown to force
    // the connection to close
    if (m_error_state.load())
    {

      m_receiver_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send, closing_error);

      if (closing_error)
      {
        std::stringstream msg;
        msg << "Error in shutdown " << closing_error.message();
        ers::error(CIBCommunicationError(ERS_HERE, msg.str()));
      }
    }

    m_receiver_socket.close(closing_error) ;

    if ( closing_error )
    {
      std::stringstream msg;
      msg << "Socket closing failed:: " << closing_error.message();
      ers::error(CIBCommunicationError(ERS_HERE,msg.str()));
    }

    m_receiver_ready.store(false);

    TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": End of do_work loop: stop receiving data from the CIB";

    TLOG_DEBUG(TLVL_ENTER_EXIT_METHODS) << get_name() << ": Exiting do_work() method";
  }

  template<typename T>
  bool CIBModule::read(boost::asio::ip::tcp::socket &socket, T &obj)
  {

    boost::system::error_code receiving_error;
    boost::asio::read( socket,
                       boost::asio::buffer( &obj, sizeof(T) ),
                       receiving_error ) ;

    if ( ! receiving_error )
    {
      return true ;
    }

    if ( receiving_error == boost::asio::error::eof)
    {
      std::string error_message = "Socket closed: " + receiving_error.message();
      ers::error(CIBCommunicationError(ERS_HERE, error_message));
      return false ;
    }

    if ( receiving_error )
    {
      std::string error_message = "Read failure: " + receiving_error.message();
      ers::error(CIBCommunicationError(ERS_HERE, error_message));
      return false ;
    }

    return true ;
  }

  void CIBModule::init_calibration_file()
  {
    if ( ! m_calibration_stream_enable )
    {
      return ;
    }
    char file_name[200] = "" ;
    time_t rawtime;
    time( & rawtime ) ;
    struct tm local_tm;
    struct tm * timeinfo = localtime_r( & rawtime , &local_tm) ;
    strftime( file_name, sizeof(file_name), "%F_%H.%M.%S.iols.calib", timeinfo );
    std::string global_name = m_calibration_dir + m_calibration_prefix + file_name ;
    m_calibration_file.open( global_name, std::ofstream::binary ) ;
    m_last_calibration_file_update = std::chrono::steady_clock::now();
    // _calibration_file.setf ( std::ios::hex, std::ios::basefield );
    // _calibration_file.unsetf ( std::ios::showbase );
    TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": New Calibration Stream file: " << global_name << std::endl ;
  }

  void CIBModule::update_calibration_file()
  {

    if ( ! m_calibration_stream_enable )
    {
      return ;
    }

    std::chrono::steady_clock::time_point check_point = std::chrono::steady_clock::now();

    if ( check_point - m_last_calibration_file_update < m_calibration_file_interval )
    {
      return ;
    }

    m_calibration_file.close() ;
    init_calibration_file() ;

  }

  bool CIBModule::set_calibration_stream( const std::string & prefix )
  {

    if ( m_calibration_dir.back() != '/' )
    {
      m_calibration_dir += '/' ;
    }
    m_calibration_prefix = prefix ;
    if ( prefix.size() > 0 )
    {
      m_calibration_prefix += '_' ;
    }
    // possibly we could check here if the directory is valid and  writable before assuming the calibration stream is valid
    return true ;
  }

  void CIBModule::send_config( const std::string & config ) {

    TLOG_DEBUG(TLVL_CIB_INFO) << get_name() << ": Sending config" << std::endl;

    // structure the message to have a common management structure
    //json receiver = doc.at("ctb").at("sockets").at("receiver");

    nlohmann::json conf;
    conf["command"] = "config";
    conf["config"] = nlohmann::json::parse(config);

    TLOG_DEBUG(TLVL_CIB_DEBUG) << get_name() << ": Shipped config : " << conf.dump() << std::endl;

    if ( send_message( conf.dump() ) )
    {
      m_is_configured.store(true) ;
    }
    else
    {
      throw CIBCommunicationError(ERS_HERE, "Unable to configure CIB");
    }
  }

  bool CIBModule::send_message( const std::string & msg )
  {

    //add error options
    boost::system::error_code error;
    TLOG_DEBUG(1) << get_name() << ": Sending message: " << msg;

    m_num_control_messages_sent++;

    boost::asio::write( m_control_socket, boost::asio::buffer( msg ), error ) ;
    boost::array<char, 1024> reply_buf{" "} ;
    m_control_socket.read_some( boost::asio::buffer(reply_buf ), error);
    std::stringstream raw_answer( std::string(reply_buf .begin(), reply_buf .end() ) ) ;
    TLOG_DEBUG(1) << get_name() << ": Unformatted answer: " << raw_answer.str();

    nlohmann::json answer ;
    raw_answer >> answer ;
    nlohmann::json & messages = answer["feedback"] ;
    TLOG_DEBUG(1) << get_name() << ": Received messages: " << messages.size();

    bool ret = true ;
    for (nlohmann::json::size_type i = 0; i != messages.size(); ++i )
    {

      m_num_control_responses_received++;

      std::string type = messages[i]["type"].dump() ;
      if ( type.find("error") != std::string::npos || type.find("Error") != std::string::npos || type.find("ERROR") != std::string::npos )
      {
        ers::error(CIBMessage(ERS_HERE, messages[i]["message"].dump()));
        ret = false ;
      }
      else if ( type.find("warning") != std::string::npos || type.find("Warning") != std::string::npos || type.find("WARNING") != std::string::npos )
      {
        ers::warning(CIBMessage(ERS_HERE, messages[i]["message"].dump()));
      }
      else if ( type.find("info") != std::string::npos || type.find("Info") != std::string::npos || type.find("INFO") != std::string::npos)
      {
        TLOG() << "Message from the CIB : " << messages[i]["message"].dump();
      }
      else
      {
        std::stringstream blob;
        blob << messages[i] ;
        TLOG() << get_name() << ": Unformatted feedback from the board: " << blob.str();
      }
    }
    return ret;
  }

  void
  CIBModule::update_buffer_counts(uint new_count) // NOLINT(build/unsigned)
  {
    std::unique_lock mon_data_lock(m_buffer_counts_mutex);
    if (m_buffer_counts.size() > 1000)
    {
      m_buffer_counts.pop_front();
    }
    m_buffer_counts.push_back(new_count);
  }

  double
  CIBModule::read_average_buffer_counts()
  {
    std::unique_lock mon_data_lock(m_buffer_counts_mutex);

    double total_counts;
    uint32_t number_of_counts; // NOLINT(build/unsigned)

    total_counts = 0;
    number_of_counts = m_buffer_counts.size();

    if (number_of_counts) {
      for (uint i = 0; i < number_of_counts; ++i) { // NOLINT(build/unsigned)
        total_counts = total_counts + m_buffer_counts.at(i);
      }
      return total_counts / number_of_counts;
    } else {
      return 0;
    }
  }

  void CIBModule::generate_opmon_data()
  {
    dunedaq::cibmodules::cibmoduleinfo::CIBModuleInfo module_info;

    module_info.num_control_messages_sent = m_num_control_messages_sent.load();
    module_info.num_control_responses_received = m_num_control_responses_received.load();
    module_info.cib_hardware_run_status = m_is_running;
    module_info.cib_hardware_configuration_status = m_is_configured;
    module_info.cib_num_triggers_received = m_num_total_triggers;

    module_info.last_readout_timestamp = m_last_readout_timestamp.load();
    // -- need to define these counters (and set the code to update them
    module_info.sent_hsi_events_counter = m_sent_counter.load();
    module_info.failed_to_send_hsi_events_counter = m_failed_to_send_counter.load();
    module_info.last_sent_timestamp = m_last_sent_timestamp.load();
    module_info.average_buffer_occupancy = read_average_buffer_counts();

    publish(std::move(module_info));

    // should we also publish specific trigger info? 
    // doesn't seem necessary at this time

  }

  bool CIBModule::check_port_in_use(unsigned short port)
  {
    using namespace boost::asio;
    using ip::tcp;

    io_service svc;
    tcp::acceptor a(svc);

    boost::system::error_code ec;
    a.open(tcp::v4(), ec) || a.bind({ tcp::v4(), port }, ec);
    a.close();
    return ec == error::address_in_use;

  }




} // namespace dunedaq::cibmodules

DEFINE_DUNE_DAQ_MODULE(dunedaq::cibmodules::CIBModule)
