// Generated by gencpp from file rosplan_knowledge_msgs/KnowledgeUpdateServiceArrayRequest.msg
// DO NOT EDIT!


#ifndef ROSPLAN_KNOWLEDGE_MSGS_MESSAGE_KNOWLEDGEUPDATESERVICEARRAYREQUEST_H
#define ROSPLAN_KNOWLEDGE_MSGS_MESSAGE_KNOWLEDGEUPDATESERVICEARRAYREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <rosplan_knowledge_msgs/KnowledgeItem.h>

namespace rosplan_knowledge_msgs
{
template <class ContainerAllocator>
struct KnowledgeUpdateServiceArrayRequest_
{
  typedef KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> Type;

  KnowledgeUpdateServiceArrayRequest_()
    : update_type()
    , knowledge()  {
    }
  KnowledgeUpdateServiceArrayRequest_(const ContainerAllocator& _alloc)
    : update_type(_alloc)
    , knowledge(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _update_type_type;
  _update_type_type update_type;

   typedef std::vector< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >::other >  _knowledge_type;
  _knowledge_type knowledge;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(ADD_KNOWLEDGE)
  #undef ADD_KNOWLEDGE
#endif
#if defined(_WIN32) && defined(ADD_GOAL)
  #undef ADD_GOAL
#endif
#if defined(_WIN32) && defined(REMOVE_KNOWLEDGE)
  #undef REMOVE_KNOWLEDGE
#endif
#if defined(_WIN32) && defined(REMOVE_GOAL)
  #undef REMOVE_GOAL
#endif
#if defined(_WIN32) && defined(ADD_METRIC)
  #undef ADD_METRIC
#endif
#if defined(_WIN32) && defined(REMOVE_METRIC)
  #undef REMOVE_METRIC
#endif

  enum {
    ADD_KNOWLEDGE = 0u,
    ADD_GOAL = 1u,
    REMOVE_KNOWLEDGE = 2u,
    REMOVE_GOAL = 3u,
    ADD_METRIC = 4u,
    REMOVE_METRIC = 5u,
  };


  typedef boost::shared_ptr< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> const> ConstPtr;

}; // struct KnowledgeUpdateServiceArrayRequest_

typedef ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<std::allocator<void> > KnowledgeUpdateServiceArrayRequest;

typedef boost::shared_ptr< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest > KnowledgeUpdateServiceArrayRequestPtr;
typedef boost::shared_ptr< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest const> KnowledgeUpdateServiceArrayRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator1> & lhs, const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator2> & rhs)
{
  return lhs.update_type == rhs.update_type &&
    lhs.knowledge == rhs.knowledge;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator1> & lhs, const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rosplan_knowledge_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca65c6d32619a07d0026fd233b46e020";
  }

  static const char* value(const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca65c6d32619a07dULL;
  static const uint64_t static_value2 = 0x0026fd233b46e020ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosplan_knowledge_msgs/KnowledgeUpdateServiceArrayRequest";
  }

  static const char* value(const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Knowledge building; extra service:\n"
"# Add or remove knowledge from the current state using an array to save time.\n"
"\n"
"uint8 ADD_KNOWLEDGE = 0\n"
"uint8 ADD_GOAL = 1\n"
"uint8 REMOVE_KNOWLEDGE = 2\n"
"uint8 REMOVE_GOAL = 3\n"
"uint8 ADD_METRIC = 4\n"
"uint8 REMOVE_METRIC = 5\n"
"\n"
"uint8[] update_type\n"
"rosplan_knowledge_msgs/KnowledgeItem[] knowledge\n"
"\n"
"================================================================================\n"
"MSG: rosplan_knowledge_msgs/KnowledgeItem\n"
"# A knowledge item used to represent a piece of the state in ROSPlan\n"
"\n"
"uint8 INSTANCE = 0\n"
"uint8 FACT = 1\n"
"uint8 FUNCTION = 2\n"
"uint8 EXPRESSION = 3\n"
"uint8 INEQUALITY = 4\n"
"\n"
"uint8 knowledge_type\n"
"\n"
"# time at which this knowledge becomes true\n"
"time initial_time\n"
"\n"
"# knowledge is explicitly false\n"
"bool is_negative\n"
"\n"
"#---------\n"
"# INSTANCE\n"
"#---------\n"
"\n"
"# instance knowledge_type\n"
"string instance_type\n"
"string instance_name\n"
"\n"
"#----------------------\n"
"# PREDICATE OR FUNCTION\n"
"#----------------------\n"
"\n"
"# attribute knowledge_type\n"
"string attribute_name\n"
"diagnostic_msgs/KeyValue[] values\n"
"\n"
"#---------\n"
"# FUNCTION\n"
"#---------\n"
"\n"
"# function value\n"
"float64 function_value\n"
"\n"
"# assignment operator\n"
"uint8 AP_ASSIGN = 0 \n"
"uint8 AP_INCREASE = 1\n"
"uint8 AP_DECREASE = 2\n"
"uint8 AP_SCALE_UP = 3\n"
"uint8 AP_SCALE_DOWN = 4\n"
"uint8 AP_ASSIGN_CTS = 5\n"
"\n"
"uint8 assign_op\n"
"\n"
"#-----------\n"
"# EXPRESSION\n"
"#-----------\n"
"\n"
"string optimization\n"
"rosplan_knowledge_msgs/ExprComposite expr\n"
"\n"
"#-----------\n"
"# INEQUALITY\n"
"#-----------\n"
"\n"
"rosplan_knowledge_msgs/DomainInequality ineq\n"
"\n"
"================================================================================\n"
"MSG: diagnostic_msgs/KeyValue\n"
"string key # what to label this value when viewing\n"
"string value # a value to track over time\n"
"\n"
"================================================================================\n"
"MSG: rosplan_knowledge_msgs/ExprComposite\n"
"# A message used to represent a numerical expression; composite type (2/2)\n"
"# stores an array of ExprBase as prefix notation\n"
"\n"
"# components\n"
"ExprBase[] tokens\n"
"\n"
"================================================================================\n"
"MSG: rosplan_knowledge_msgs/ExprBase\n"
"# A message used to represent a numerical expression; base types (1/2)\n"
"\n"
"# expression types\n"
"uint8 CONSTANT = 0\n"
"uint8 FUNCTION = 1\n"
"uint8 OPERATOR = 2\n"
"uint8 SPECIAL  = 3\n"
"\n"
"# operators\n"
"uint8 ADD    = 0\n"
"uint8 SUB    = 1\n"
"uint8 MUL    = 2\n"
"uint8 DIV    = 3\n"
"uint8 UMINUS = 4\n"
"\n"
"# special types\n"
"uint8 HASHT      = 0\n"
"uint8 TOTAL_TIME = 1\n"
"uint8 DURATION   = 2\n"
"\n"
"# expression base type\n"
"uint8 expr_type\n"
"\n"
"# constant value\n"
"float64 constant\n"
"\n"
"# function\n"
"rosplan_knowledge_msgs/DomainFormula function\n"
"\n"
"# operator\n"
"uint8 op\n"
"\n"
"# special\n"
"uint8 special_type\n"
"\n"
"================================================================================\n"
"MSG: rosplan_knowledge_msgs/DomainFormula\n"
"# A message used to represent an atomic formula from the domain.\n"
"# typed_parameters matches label -> type\n"
"string name\n"
"diagnostic_msgs/KeyValue[] typed_parameters\n"
"\n"
"================================================================================\n"
"MSG: rosplan_knowledge_msgs/DomainInequality\n"
"# A message used to store the numeric effects of an action\n"
"# Can be grounded or ungrounded\n"
"\n"
"uint8 GREATER   = 0\n"
"uint8 GREATEREQ = 1\n"
"uint8 LESS      = 2\n"
"uint8 LESSEQ    = 3\n"
"uint8 EQUALS    = 4\n"
"\n"
"uint8 comparison_type\n"
"\n"
"rosplan_knowledge_msgs/ExprComposite LHS\n"
"rosplan_knowledge_msgs/ExprComposite RHS\n"
"\n"
"bool grounded\n"
;
  }

  static const char* value(const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.update_type);
      stream.next(m.knowledge);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct KnowledgeUpdateServiceArrayRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest_<ContainerAllocator>& v)
  {
    s << indent << "update_type[]" << std::endl;
    for (size_t i = 0; i < v.update_type.size(); ++i)
    {
      s << indent << "  update_type[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.update_type[i]);
    }
    s << indent << "knowledge[]" << std::endl;
    for (size_t i = 0; i < v.knowledge.size(); ++i)
    {
      s << indent << "  knowledge[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >::stream(s, indent + "    ", v.knowledge[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSPLAN_KNOWLEDGE_MSGS_MESSAGE_KNOWLEDGEUPDATESERVICEARRAYREQUEST_H
