/* Author: Matt Chignoli */

#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <urdf_model/constraint.h>
#include <console_bridge/console.h>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

#include "./pose.hpp"

namespace urdf{

template <typename ConstraintType>
bool parseConstraint(ConstraintType &constraint, tinyxml2::XMLElement* config)
{
  // Get Constraint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    CONSOLE_BRIDGE_logError("unnamed constraint found");
    return false;
  }
  constraint.name = name;

  // Get Parent Link
  tinyxml2::XMLElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("link");
    if (!pname)
    {
      CONSOLE_BRIDGE_logInform("no parent link name specified for Constraint link [%s]. this might be the root?", constraint.name.c_str());
    }
    else
    {
      constraint.parent_link_name = std::string(pname);
    }
  }

  // Get Child Link
  tinyxml2::XMLElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
    {
      CONSOLE_BRIDGE_logInform("no child link name specified for Constraint link [%s].", constraint.name.c_str());
    }
    else
    {
      constraint.child_link_name = std::string(pname);
    }
  }

  return true;
}

bool parseLoopConstraint(LoopConstraint &constraint, tinyxml2::XMLElement* config)
{
  constraint.clear();

  if(!parseConstraint(constraint, config))
    return false;

  // Get transform from Parent Link to Constraint Frame on Parent Link
  tinyxml2::XMLElement *parent_origin_xml = config->FirstChildElement("parent_origin");
  if (!parent_origin_xml)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: Constraint [%s] missing parent_origin tag describing transform from Parent Link to Constraint Frame, (using Identity transform).", constraint.name.c_str());
    constraint.parent_to_constraint_origin_transform.clear();
  }
  else
  {
    if (!parsePoseInternal(constraint.parent_to_constraint_origin_transform, parent_origin_xml))
    {
      constraint.parent_to_constraint_origin_transform.clear();
      CONSOLE_BRIDGE_logError("Malformed parent origin element for constraint [%s]", constraint.name.c_str());
      return false;
    }
  }

  // Get transform from Child Link to Joint Frame on Child Link
  tinyxml2::XMLElement *child_origin_xml = config->FirstChildElement("child_origin");
  if (!child_origin_xml)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: Constraint [%s] missing child_origin tag describing transform from Child Link to Constraint Frame, (using Identity transform).", constraint.name.c_str());
    constraint.child_to_constraint_origin_transform.clear();
  }
  else
  {
    if (!parsePoseInternal(constraint.child_to_constraint_origin_transform, child_origin_xml))
    {
      constraint.child_to_constraint_origin_transform.clear();
      CONSOLE_BRIDGE_logError("Malformed child origin element for constraint [%s]", constraint.name.c_str());
      return false;
    }
  }

  // Get Constraint Axes
  tinyxml2::XMLElement *pos_axis_xml = config->FirstChildElement("pos_axis");
  if (!pos_axis_xml){
    CONSOLE_BRIDGE_logDebug("urdfdom: no position_axis element for Constraint [%s], defaulting to (1,1,1) axis", constraint.name.c_str());
    constraint.position_axis = Vector3(1.0, 1.0, 1.0);
  }
  else{
    if (pos_axis_xml->Attribute("xyz")){
      try {
        constraint.position_axis.init(pos_axis_xml->Attribute("xyz"));
      }
      catch (ParseError &e) {
        constraint.position_axis.clear();
        CONSOLE_BRIDGE_logError("Malformed position_axis element for constraint [%s]: %s", constraint.name.c_str(), e.what());
        return false;
      }
    }
  }

  tinyxml2::XMLElement *rot_axis_xml = config->FirstChildElement("rot_axis");
  if (!rot_axis_xml){
    CONSOLE_BRIDGE_logDebug("urdfdom: no rotation_axis element for Constraint [%s], defaulting to (1,1,1) axis", constraint.name.c_str());
    constraint.rotation_axis = Vector3(1.0, 1.0, 1.0);
  }
  else{
    if (pos_axis_xml->Attribute("xyz")){
      try {
        constraint.rotation_axis.init(rot_axis_xml->Attribute("xyz"));
      }
      catch (ParseError &e) {
        constraint.rotation_axis.clear();
        CONSOLE_BRIDGE_logError("Malformed rotation_axis element for constraint [%s]: %s", constraint.name.c_str(), e.what());
        return false;
      }
    }
  }

  return true;

}

bool parseJointConstraint(JointConstraint &constraint, tinyxml2::XMLElement* config)
{
  constraint.clear();

  if(!parseConstraint(constraint, config))
    return false;

  // Get gear ratio
  const char* gr = config->Attribute("gear_ratio");
  if (gr == NULL){
    CONSOLE_BRIDGE_logDebug("urdfdom.gear_ratio: no gear ratio");
    return false;
  }
  else
  {
    try {
      constraint.gear_ratio = strToDouble(gr);
    } catch (std::runtime_error &) {
      CONSOLE_BRIDGE_logError("lower value (%s) is not a valid float", gr);
      return false;
    }
  }

  return true;

}

}
