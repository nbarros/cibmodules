

# This is a skeleton CMakeLists.txt file, auto-generated on
# Mon Mar 25 21:46:48 UTC 2024.  The developer(s) of this package should delete
# this comment as well as adding dependent targets, packages,
# etc. specific to the package. For details on how to write a package,
# please see
# https://dune-daq-sw.readthedocs.io/en/latest/packages/daq-cmake/

cmake_minimum_required(VERSION 3.12)
project(cibmodules VERSION 1.0.0)

find_package(daq-cmake REQUIRED)

daq_setup_environment()


find_package(appfwk REQUIRED)
find_package(opmonlib REQUIRED)
# Other modules that are needed for this specific case
find_package(logging REQUIRED)
find_package(ers REQUIRED)
find_package(hsilibs REQUIRED)
find_package(daqconf REQUIRED)
##############################################################################
# Protobuf code generation for data structures
daq_protobuf_codegen( opmon/*.proto )
##############################################################################

daq_add_library( *.cpp LINK_LIBRARIES 
                 opmonlib::opmonlib ers::ers appfwk::appfwk logging::logging stdc++fs hsilibs::hsilibs )

daq_add_plugin(CIBModule duneDAQModule            LINK_LIBRARIES cibmodules )

daq_install()

