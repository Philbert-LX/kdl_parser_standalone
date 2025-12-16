// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Wim Meeussen */

#ifndef KDL_PARSER__KDL_PARSER_HPP_
#define KDL_PARSER__KDL_PARSER_HPP_

#include <string>
#include <map>

#include "kdl/tree.hpp"
#include "urdf_model/model.h"

// 强制覆盖：对于静态库，不需要导出符号，直接定义为空
// 如果之前定义了 KDL_PARSER_PUBLIC（比如从 vcpkg 的旧版本），先取消定义再重新定义
#ifdef KDL_PARSER_PUBLIC
#undef KDL_PARSER_PUBLIC
#endif
#define KDL_PARSER_PUBLIC

namespace kdl_parser
{

/** Constructs a KDL tree from a file, given the file name
 * \param file The filename from where to read the xml
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromFile(const std::string & file, KDL::Tree & tree);

/** Constructs a KDL tree from a string containing xml
 * \param xml A string containting the xml description of the robot
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromString(const std::string & xml, KDL::Tree & tree);

/** Constructs a KDL tree from a URDF robot model
 * \param robot_model The URDF robot model
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromUrdfModel(const urdf::ModelInterface & robot_model, KDL::Tree & tree);

/** Constructs a KDL tree from a file, given the file name, with UUID/InnerId mapping support
 * \param file The filename from where to read the xml
 * \param link_uuid_map Output map where keys are link names and values are uuid
 *                      The map will be populated with extracted UUIDs from the URDF file
 * \param joint_innerid_map Output map where keys are joint names and values are innerId
 *                          The map will be populated with extracted innerIds from the URDF file
 * \param joint_limit_lower_map Output map where keys are joint names and values are lower limit values
 *                              The map will be populated with extracted lower limits from the URDF file
 * \param joint_limit_upper_map Output map where keys are joint names and values are upper limit values
 *                              The map will be populated with extracted upper limits from the URDF file
 * \param joint_limit_velocity_map Output map where keys are joint names and values are velocity limit values
 *                                 The map will be populated with extracted velocity limits from the URDF file
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromFileWithIds(const std::string & file, std::map<std::string, std::string> & link_uuid_map, std::map<std::string, std::string> & joint_innerid_map, std::map<std::string, double> & joint_limit_lower_map, std::map<std::string, double> & joint_limit_upper_map, std::map<std::string, double> & joint_limit_velocity_map, KDL::Tree & tree);

/** Constructs a KDL tree from a string containing xml, with UUID/InnerId mapping support
 * \param xml A string containting the xml description of the robot
 * \param link_uuid_map Output map where keys are link names and values are uuid
 *                      The map will be populated with extracted UUIDs from the URDF XML
 * \param joint_innerid_map Output map where keys are joint names and values are innerId
 *                          The map will be populated with extracted innerIds from the URDF XML
 * \param joint_limit_lower_map Output map where keys are joint names and values are lower limit values
 *                              The map will be populated with extracted lower limits from the URDF XML
 * \param joint_limit_upper_map Output map where keys are joint names and values are upper limit values
 *                              The map will be populated with extracted upper limits from the URDF XML
 * \param joint_limit_velocity_map Output map where keys are joint names and values are velocity limit values
 *                                 The map will be populated with extracted velocity limits from the URDF XML
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromStringWithIds(const std::string & xml, std::map<std::string, std::string> & link_uuid_map, std::map<std::string, std::string> & joint_innerid_map, std::map<std::string, double> & joint_limit_lower_map, std::map<std::string, double> & joint_limit_upper_map, std::map<std::string, double> & joint_limit_velocity_map, KDL::Tree & tree);

/** Constructs a KDL tree from a URDF robot model, with UUID/InnerId mapping support
 * \param robot_model The URDF robot model
 * \param xml The original XML string (required to extract uuid/innerId since ModelInterface doesn't contain them)
 * \param link_uuid_map Output map where keys are link names and values are uuid
 *                      The map will be populated with extracted UUIDs from the URDF XML
 * \param joint_innerid_map Output map where keys are joint names and values are innerId
 *                          The map will be populated with extracted innerIds from the URDF XML
 * \param joint_limit_lower_map Output map where keys are joint names and values are lower limit values
 *                              The map will be populated with extracted lower limits from the URDF model
 * \param joint_limit_upper_map Output map where keys are joint names and values are upper limit values
 *                              The map will be populated with extracted upper limits from the URDF model
 * \param joint_limit_velocity_map Output map where keys are joint names and values are velocity limit values
 *                                 The map will be populated with extracted velocity limits from the URDF model
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromUrdfModelWithIds(const urdf::ModelInterface & robot_model, const std::string & xml, std::map<std::string, std::string> & link_uuid_map, std::map<std::string, std::string> & joint_innerid_map, std::map<std::string, double> & joint_limit_lower_map, std::map<std::string, double> & joint_limit_upper_map, std::map<std::string, double> & joint_limit_velocity_map, KDL::Tree & tree);
}  // namespace kdl_parser

#endif  // KDL_PARSER__KDL_PARSER_HPP_
