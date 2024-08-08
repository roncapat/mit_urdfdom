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

  // Get Predecessor Link
  tinyxml2::XMLElement *predecessor_xml = config->FirstChildElement("predecessor");
  if (predecessor_xml)
  {
    const char *pname = predecessor_xml->Attribute("link");
    if (!pname)
    {
      CONSOLE_BRIDGE_logInform("no predecessor link name specified for Constraint link [%s]. this might be the root?", constraint.name.c_str());
    }
    else
    {
      constraint.predecessor_link_name = std::string(pname);
    }
  }

  // Get Successor Link
  tinyxml2::XMLElement *succesor_xml = config->FirstChildElement("successor");
  if (succesor_xml)
  {
    const char *sname = succesor_xml->Attribute("link");
    if (!sname)
    {
      CONSOLE_BRIDGE_logInform("no succesor link name specified for Constraint link [%s].", constraint.name.c_str());
    }
    else
    {
      constraint.successor_link_name = std::string(sname);
    }
  }

  return true;
}

bool parseLoopConstraint(LoopConstraint &constraint, tinyxml2::XMLElement* config)
{
  constraint.clear();

  if(!parseConstraint(constraint, config))
    return false;

  // Get transform from Predecessor Link to Constraint Frame on Predecessor Link
  tinyxml2::XMLElement *predecessor_origin_xml = config->FirstChildElement("predecessor_origin");
  if (!predecessor_origin_xml)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: Constraint [%s] missing predecessor_origin tag describing transform from Predecessor Link to Constraint Frame, (using Identity transform).", constraint.name.c_str());
    constraint.predecessor_to_joint_origin_transform.clear();
  }
  else
  {
    if (!parsePoseInternal(constraint.predecessor_to_joint_origin_transform, predecessor_origin_xml))
    {
      constraint.predecessor_to_joint_origin_transform.clear();
      CONSOLE_BRIDGE_logError("Malformed predecessor origin element for constraint [%s]", constraint.name.c_str());
      return false;
    }
  }

  // Get transform from Successor Link to Joint Frame on Successor Link
  tinyxml2::XMLElement *successor_origin_xml = config->FirstChildElement("successor_origin");
  if (!successor_origin_xml)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: Constraint [%s] missing successor_origin tag describing transform from Successor Link to Constraint Frame, (using Identity transform).", constraint.name.c_str());
    constraint.successor_to_joint_origin_transform.clear();
  }
  else
  {
    if (!parsePoseInternal(constraint.successor_to_joint_origin_transform, successor_origin_xml))
    {
      constraint.successor_to_joint_origin_transform.clear();
      CONSOLE_BRIDGE_logError("Malformed successor origin element for constraint [%s]", constraint.name.c_str());
      return false;
    }
  }

    // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
    CONSOLE_BRIDGE_logError("constraint [%s] has no type, check to see if it's a reference.", constraint.name.c_str());
    return false;
  }

  std::string type_str = type_char;
  if (type_str == "planar")
    constraint.type = LoopConstraint::PLANAR;
  else if (type_str == "revolute")
    constraint.type = LoopConstraint::REVOLUTE;
  else if (type_str == "continuous")
    constraint.type = LoopConstraint::CONTINUOUS;
  else if (type_str == "prismatic")
    constraint.type = LoopConstraint::PRISMATIC;
  else if (type_str == "fixed")
    constraint.type = LoopConstraint::FIXED;
  else
  {
    CONSOLE_BRIDGE_logError("Joint [%s] has no known type [%s]", constraint.name.c_str(), type_str.c_str());
    return false;
  }

    // Get Joint Axis
  if (constraint.type != LoopConstraint::FIXED)
  {
    // axis
    tinyxml2::XMLElement *axis_xml = config->FirstChildElement("axis");
    if (!axis_xml){
      CONSOLE_BRIDGE_logDebug("urdfdom: no axis element for constraint [%s], defaulting to (1,0,0) axis", constraint.name.c_str());
      constraint.axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (axis_xml->Attribute("xyz")){
        try {
          constraint.axis.init(axis_xml->Attribute("xyz"));
        }
        catch (ParseError &e) {
          constraint.axis.clear();
          CONSOLE_BRIDGE_logError("Malformed axis element for constraint [%s]: %s", constraint.name.c_str(), e.what());
          return false;
        }
      }
    }
  }

  return true;

}

bool parseCouplingConstraint(CouplingConstraint &constraint, tinyxml2::XMLElement* config)
{
  constraint.clear();

  if(!parseConstraint(constraint, config))
    return false;

  // Get ratio
  tinyxml2::XMLElement *ratio_xml = config->FirstChildElement("ratio");
  if (ratio_xml)
  {
    const char* ratio = ratio_xml->Attribute("value");
    if (ratio == NULL){
      CONSOLE_BRIDGE_logDebug("urdfdom.ratio: no ratio");
      return false;
    }
    else
    {
      try {
        constraint.ratio = strToDouble(ratio);
      } catch (std::runtime_error &) {
        CONSOLE_BRIDGE_logError("ratio (%s) is not a valid float", ratio);
        return false;
      }
    }
  }

  return true;

}

}
