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
  tinyxml2::XMLElement *predecessor_xml = config->FirstChildElement("predecessor");
  if (!predecessor_xml)
  {
    CONSOLE_BRIDGE_logError("Loop Constraint [%s] missing predecessor tag.", constraint.name.c_str());
    return false;
  }
  else
  {
    tinyxml2::XMLElement *origin_xml = predecessor_xml->FirstChildElement("origin");
    if (!origin_xml)
    {
      CONSOLE_BRIDGE_logDebug("urdfdom: Loop Constraint [%s] missing origin tag under predecessor describing transform from Predecessor Link to Constraint Frame, (using Identity transform).", constraint.name.c_str());
      constraint.predecessor_to_constraint_origin_transform.clear();
    }
    else
    {
      if (!parsePoseInternal(constraint.predecessor_to_constraint_origin_transform, origin_xml))
      {
        constraint.predecessor_to_constraint_origin_transform.clear();
        CONSOLE_BRIDGE_logError("Malformed predecessor origin element for constraint [%s]", constraint.name.c_str());
        return false;
      }
    }
  }

  // Get transform from Successor Link to Constraint Frame on Successor Link
  tinyxml2::XMLElement *successor_xml = config->FirstChildElement("successor");
  if (!successor_xml)
  {
    CONSOLE_BRIDGE_logError("Loop Constraint [%s] missing successor tag.", constraint.name.c_str());
    return false;
  }
  else
  {
    tinyxml2::XMLElement *origin_xml = successor_xml->FirstChildElement("origin");
    if (!origin_xml)
    {
      CONSOLE_BRIDGE_logDebug("urdfdom: Loop Constraint [%s] missing origin tag under predecessor describing transform from Successor Link to Constraint Frame, (using Identity transform).", constraint.name.c_str());
      constraint.successor_to_constraint_origin_transform.clear();
    }
    else
    {
      if (!parsePoseInternal(constraint.successor_to_constraint_origin_transform, origin_xml))
      {
        constraint.successor_to_constraint_origin_transform.clear();
        CONSOLE_BRIDGE_logError("Malformed succesor origin element for constraint [%s]", constraint.name.c_str());
        return false;
      }
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
    CONSOLE_BRIDGE_logError("Constraint [%s] has no known type [%s]", constraint.name.c_str(), type_str.c_str());
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

/* exports */
bool exportPose(Pose &pose, tinyxml2::XMLElement* xml);

bool exportConstraint(Constraint& constraint, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement* constraint_xml = xml->GetDocument()->NewElement("constraint");
  constraint_xml->SetAttribute("name", constraint.name.c_str());

  // Predecessor and Successor
  tinyxml2::XMLElement* predecessor_xml = constraint_xml->GetDocument()->NewElement("predecessor");
  predecessor_xml->SetAttribute("link", constraint.predecessor_link_name.c_str());

  tinyxml2::XMLElement* successor_xml = constraint_xml->GetDocument()->NewElement("successor");
  successor_xml->SetAttribute("link", constraint.successor_link_name.c_str());

  // Class specific
  if (constraint.class_type == urdf::Constraint::LOOP)
  {
    LoopConstraint* loop_constraint = dynamic_cast<LoopConstraint*>(&constraint);
    if (loop_constraint->type == urdf::LoopConstraint::REVOLUTE)
      constraint_xml->SetAttribute("type", "revolute");
    else if (loop_constraint->type == urdf::LoopConstraint::CONTINUOUS)
      constraint_xml->SetAttribute("type", "continuous");
    else if (loop_constraint->type == urdf::LoopConstraint::PRISMATIC)
      constraint_xml->SetAttribute("type", "prismatic");
    else if (loop_constraint->type == urdf::LoopConstraint::FIXED)
      constraint_xml->SetAttribute("type", "fixed");
    else if (loop_constraint->type == urdf::LoopConstraint::PLANAR)
      constraint_xml->SetAttribute("type", "planar");
    else
    {
      CONSOLE_BRIDGE_logError("Constraint [%s] has no known type [%d]", constraint.name.c_str(), loop_constraint->type);
      return false;
    }

    // Origins
    exportPose(loop_constraint->predecessor_to_constraint_origin_transform, predecessor_xml);
    exportPose(loop_constraint->successor_to_constraint_origin_transform, successor_xml);

    // Axis
    tinyxml2::XMLElement* axis_xml = constraint_xml->GetDocument()->NewElement("axis");
    axis_xml->SetAttribute("xyz", urdf_export_helpers::values2str(loop_constraint->axis).c_str());
    constraint_xml->LinkEndChild(axis_xml);
  }
  else if (constraint.class_type == urdf::Constraint::COUPLING)
  {
    CouplingConstraint* coupling_constraint = dynamic_cast<CouplingConstraint*>(&constraint);
    
    // Ratio
    tinyxml2::XMLElement* ratio_xml = constraint_xml->GetDocument()->NewElement("ratio");
    ratio_xml->SetAttribute("value", urdf_export_helpers::values2str(coupling_constraint->ratio).c_str());
    constraint_xml->LinkEndChild(ratio_xml);
  }
  else
  {
    CONSOLE_BRIDGE_logError("Constraint [%s] has no known class type [%d]", constraint.name.c_str(), constraint.class_type);
    return false;
  }

  constraint_xml->LinkEndChild(predecessor_xml);
  constraint_xml->LinkEndChild(successor_xml);

  xml->LinkEndChild(constraint_xml);

  return true;

}


}
