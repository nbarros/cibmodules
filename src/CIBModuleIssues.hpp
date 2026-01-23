/*
 * CIBModuleIssues.hpp
 *
 *  Created on: Mar 25, 2024
 *      Author: Nuno Barros
 */

#ifndef CIBMODULES_SRC_CIBMODULEISSUES_HPP_
#define CIBMODULES_SRC_CIBMODULEISSUES_HPP_


#include "ers/Issue.hpp"

#include <string>

namespace dunedaq {

// Disable coverage collection LCOV_EXCL_START
ERS_DECLARE_ISSUE(cibmodules,
                  CIBCommunicationError,
                  " CIB Hardware Communication Error: " << descriptor,
                  ((std::string)descriptor))

ERS_DECLARE_ISSUE(cibmodules,
                  CIBBufferWarning,
                  " CIB Buffer Issue: " << descriptor,
                  ((std::string)descriptor))

ERS_DECLARE_ISSUE(cibmodules,
                  CIBWordMatchError,
                  " CIB Word Matching Error: " << descriptor,
                  ((std::string)descriptor))

ERS_DECLARE_ISSUE(cibmodules,
                  CIBMessage,
                  " Message from CIB: " << descriptor,
                  ((std::string)descriptor))


ERS_DECLARE_ISSUE(cibmodules,
                  CIBWrongState,
                  " CIB in the wrong state: " << descriptor,
                  ((std::string)descriptor))

ERS_DECLARE_ISSUE(cibmodules,
                  CIBProcError,
                  " CIB process error: " << descriptor,
                  ((std::string)descriptor))

ERS_DECLARE_ISSUE(cibmodules,
                  CIBModuleError,
                  " CIB Module error: " << descriptor,
                  ((std::string)descriptor))

ERS_DECLARE_ISSUE(cibmodules,
                  CIBConfigFailure,
                  descriptor,
                  ((std::string)descriptor))

// Re-enable coverage collection LCOV_EXCL_STOP

} // namespace dunedaq




#endif /* CIBMODULES_SRC_CIBMODULEISSUES_HPP_ */
