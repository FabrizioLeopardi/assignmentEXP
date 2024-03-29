// Generated by gencpp from file rosplan_knowledge_msgs/KnowledgeItem.msg
// DO NOT EDIT!


#ifndef ROSPLAN_KNOWLEDGE_MSGS_MESSAGE_KNOWLEDGEITEM_H
#define ROSPLAN_KNOWLEDGE_MSGS_MESSAGE_KNOWLEDGEITEM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <diagnostic_msgs/KeyValue.h>
#include <rosplan_knowledge_msgs/ExprComposite.h>
#include <rosplan_knowledge_msgs/DomainInequality.h>

namespace rosplan_knowledge_msgs
{
template <class ContainerAllocator>
struct KnowledgeItem_
{
  typedef KnowledgeItem_<ContainerAllocator> Type;

  KnowledgeItem_()
    : knowledge_type(0)
    , initial_time()
    , is_negative(false)
    , instance_type()
    , instance_name()
    , attribute_name()
    , values()
    , function_value(0.0)
    , assign_op(0)
    , optimization()
    , expr()
    , ineq()  {
    }
  KnowledgeItem_(const ContainerAllocator& _alloc)
    : knowledge_type(0)
    , initial_time()
    , is_negative(false)
    , instance_type(_alloc)
    , instance_name(_alloc)
    , attribute_name(_alloc)
    , values(_alloc)
    , function_value(0.0)
    , assign_op(0)
    , optimization(_alloc)
    , expr(_alloc)
    , ineq(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _knowledge_type_type;
  _knowledge_type_type knowledge_type;

   typedef ros::Time _initial_time_type;
  _initial_time_type initial_time;

   typedef uint8_t _is_negative_type;
  _is_negative_type is_negative;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _instance_type_type;
  _instance_type_type instance_type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _instance_name_type;
  _instance_name_type instance_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _attribute_name_type;
  _attribute_name_type attribute_name;

   typedef std::vector< ::diagnostic_msgs::KeyValue_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::diagnostic_msgs::KeyValue_<ContainerAllocator> >::other >  _values_type;
  _values_type values;

   typedef double _function_value_type;
  _function_value_type function_value;

   typedef uint8_t _assign_op_type;
  _assign_op_type assign_op;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _optimization_type;
  _optimization_type optimization;

   typedef  ::rosplan_knowledge_msgs::ExprComposite_<ContainerAllocator>  _expr_type;
  _expr_type expr;

   typedef  ::rosplan_knowledge_msgs::DomainInequality_<ContainerAllocator>  _ineq_type;
  _ineq_type ineq;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(INSTANCE)
  #undef INSTANCE
#endif
#if defined(_WIN32) && defined(FACT)
  #undef FACT
#endif
#if defined(_WIN32) && defined(FUNCTION)
  #undef FUNCTION
#endif
#if defined(_WIN32) && defined(EXPRESSION)
  #undef EXPRESSION
#endif
#if defined(_WIN32) && defined(INEQUALITY)
  #undef INEQUALITY
#endif
#if defined(_WIN32) && defined(AP_ASSIGN)
  #undef AP_ASSIGN
#endif
#if defined(_WIN32) && defined(AP_INCREASE)
  #undef AP_INCREASE
#endif
#if defined(_WIN32) && defined(AP_DECREASE)
  #undef AP_DECREASE
#endif
#if defined(_WIN32) && defined(AP_SCALE_UP)
  #undef AP_SCALE_UP
#endif
#if defined(_WIN32) && defined(AP_SCALE_DOWN)
  #undef AP_SCALE_DOWN
#endif
#if defined(_WIN32) && defined(AP_ASSIGN_CTS)
  #undef AP_ASSIGN_CTS
#endif

  enum {
    INSTANCE = 0u,
    FACT = 1u,
    FUNCTION = 2u,
    EXPRESSION = 3u,
    INEQUALITY = 4u,
    AP_ASSIGN = 0u,
    AP_INCREASE = 1u,
    AP_DECREASE = 2u,
    AP_SCALE_UP = 3u,
    AP_SCALE_DOWN = 4u,
    AP_ASSIGN_CTS = 5u,
  };


  typedef boost::shared_ptr< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> const> ConstPtr;

}; // struct KnowledgeItem_

typedef ::rosplan_knowledge_msgs::KnowledgeItem_<std::allocator<void> > KnowledgeItem;

typedef boost::shared_ptr< ::rosplan_knowledge_msgs::KnowledgeItem > KnowledgeItemPtr;
typedef boost::shared_ptr< ::rosplan_knowledge_msgs::KnowledgeItem const> KnowledgeItemConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator1> & lhs, const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator2> & rhs)
{
  return lhs.knowledge_type == rhs.knowledge_type &&
    lhs.initial_time == rhs.initial_time &&
    lhs.is_negative == rhs.is_negative &&
    lhs.instance_type == rhs.instance_type &&
    lhs.instance_name == rhs.instance_name &&
    lhs.attribute_name == rhs.attribute_name &&
    lhs.values == rhs.values &&
    lhs.function_value == rhs.function_value &&
    lhs.assign_op == rhs.assign_op &&
    lhs.optimization == rhs.optimization &&
    lhs.expr == rhs.expr &&
    lhs.ineq == rhs.ineq;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator1> & lhs, const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rosplan_knowledge_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eb1f72eba0c2c67ff10276f88d435a64";
  }

  static const char* value(const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeb1f72eba0c2c67fULL;
  static const uint64_t static_value2 = 0xf10276f88d435a64ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosplan_knowledge_msgs/KnowledgeItem";
  }

  static const char* value(const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# A knowledge item used to represent a piece of the state in ROSPlan\n"
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

  static const char* value(const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.knowledge_type);
      stream.next(m.initial_time);
      stream.next(m.is_negative);
      stream.next(m.instance_type);
      stream.next(m.instance_name);
      stream.next(m.attribute_name);
      stream.next(m.values);
      stream.next(m.function_value);
      stream.next(m.assign_op);
      stream.next(m.optimization);
      stream.next(m.expr);
      stream.next(m.ineq);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct KnowledgeItem_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosplan_knowledge_msgs::KnowledgeItem_<ContainerAllocator>& v)
  {
    s << indent << "knowledge_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.knowledge_type);
    s << indent << "initial_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.initial_time);
    s << indent << "is_negative: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_negative);
    s << indent << "instance_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.instance_type);
    s << indent << "instance_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.instance_name);
    s << indent << "attribute_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.attribute_name);
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size(); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::diagnostic_msgs::KeyValue_<ContainerAllocator> >::stream(s, indent + "    ", v.values[i]);
    }
    s << indent << "function_value: ";
    Printer<double>::stream(s, indent + "  ", v.function_value);
    s << indent << "assign_op: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.assign_op);
    s << indent << "optimization: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.optimization);
    s << indent << "expr: ";
    s << std::endl;
    Printer< ::rosplan_knowledge_msgs::ExprComposite_<ContainerAllocator> >::stream(s, indent + "  ", v.expr);
    s << indent << "ineq: ";
    s << std::endl;
    Printer< ::rosplan_knowledge_msgs::DomainInequality_<ContainerAllocator> >::stream(s, indent + "  ", v.ineq);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSPLAN_KNOWLEDGE_MSGS_MESSAGE_KNOWLEDGEITEM_H
