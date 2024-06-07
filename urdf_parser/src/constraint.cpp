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

// TODO(@MatthewChignoli): Distinguish between JointConstraints and LoopConstraints
// bool parseJointConstraint(JointConstraint &constraint, tinyxml2::XMLElement* config)
bool parseConstraint(Constraint &constraint, tinyxml2::XMLElement* config)
{
  constraint.clear();

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

}
