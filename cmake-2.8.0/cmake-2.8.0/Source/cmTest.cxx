/*============================================================================
  CMake - Cross Platform Makefile Generator
  Copyright 2000-2009 Kitware, Inc., Insight Software Consortium

  Distributed under the OSI-approved BSD License (the "License");
  see accompanying file Copyright.txt for details.

  This software is distributed WITHOUT ANY WARRANTY; without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the License for more information.
============================================================================*/
#include "cmTest.h"
#include "cmSystemTools.h"

#include "cmake.h"
#include "cmMakefile.h"

//----------------------------------------------------------------------------
cmTest::cmTest(cmMakefile* mf)
{
  this->Makefile = mf;
  this->OldStyle = true;
  this->Properties.SetCMakeInstance(mf->GetCMakeInstance());
  this->Backtrace = new cmListFileBacktrace;
  this->Makefile->GetBacktrace(*this->Backtrace);
}

//----------------------------------------------------------------------------
cmTest::~cmTest()
{
  delete this->Backtrace;
}

//----------------------------------------------------------------------------
cmListFileBacktrace const& cmTest::GetBacktrace() const
{
  return *this->Backtrace;
}

//----------------------------------------------------------------------------
void cmTest::SetName(const char* name)
{
  if ( !name )
    {
    name = "";
    }
  this->Name = name;
}

//----------------------------------------------------------------------------
void cmTest::SetCommand(std::vector<std::string> const& command)
{
  this->Command = command;
}

//----------------------------------------------------------------------------
const char *cmTest::GetProperty(const char* prop) const
{
  bool chain = false;
  const char *retVal = 
    this->Properties.GetPropertyValue(prop, cmProperty::TEST, chain);
  if (chain)
    {
    return this->Makefile->GetProperty(prop,cmProperty::TEST);
    }
  return retVal;
}

//----------------------------------------------------------------------------
bool cmTest::GetPropertyAsBool(const char* prop) const
{
  return cmSystemTools::IsOn(this->GetProperty(prop));
}

//----------------------------------------------------------------------------
void cmTest::SetProperty(const char* prop, const char* value)
{
  if (!prop)
    {
    return;
    }

  this->Properties.SetProperty(prop, value, cmProperty::TEST);
}

//----------------------------------------------------------------------------
void cmTest::AppendProperty(const char* prop, const char* value)
{
  if (!prop)
    {
    return;
    }
  this->Properties.AppendProperty(prop, value, cmProperty::TEST);
}

//----------------------------------------------------------------------------
void cmTest::DefineProperties(cmake *cm)
{
  cm->DefineProperty
    ("ENVIRONMENT", cmProperty::TEST,
     "Specify environment variables that should be defined for running "
     "a test.",
     "If set to a list of environment variables and values of the form "
     "MYVAR=value those environment variables will be defined while "
     "running the test. The environment is restored to its previous state "
     "after the test is done.");

  cm->DefineProperty
    ("FAIL_REGULAR_EXPRESSION", cmProperty::TEST,
     "If the output matches this regular expression the test will fail.",
     "If set, if the output matches one of "
     "specified regular expressions, the test will fail."
     "For example: PASS_REGULAR_EXPRESSION \"[^a-z]Error;ERROR;Failed\"");

  cm->DefineProperty
    ("LABELS", cmProperty::TEST,
     "Specify a list of text labels associated with a test.",
     "The list is reported in dashboard submissions.");

  cm->DefineProperty
    ("MEASUREMENT", cmProperty::TEST, 
     "Specify a CDASH measurement and value to be reported for a test.",
     "If set to a name then that name will be reported to CDASH as a "
     "named measurement with a value of 1. You may also specify a value "
     "by setting MEASUREMENT to \"measurement=value\".");

  cm->DefineProperty
    ("PASS_REGULAR_EXPRESSION", cmProperty::TEST, 
     "The output must match this regular expression for the test to pass.",
     "If set, the test output will be checked "
     "against the specified regular expressions and at least one of the"
     " regular expressions has to match, otherwise the test will fail.");

  cm->DefineProperty
    ("TIMEOUT", cmProperty::TEST, 
     "How many seconds to allow for this test.",
     "This property if set will limit a test to not take more than "
     "the specified number of seconds to run. If it exceeds that the "
     "test process will be killed and ctest will move to the next test. "
     "This setting takes precedence over "
     "CTEST_TESTING_TIMEOUT.");

  cm->DefineProperty
    ("WILL_FAIL", cmProperty::TEST, 
     "If set to true, this will invert the pass/fail flag of the test.",
     "This property can be used for tests that are expected to fail and "
     "return a non zero return code.");
}