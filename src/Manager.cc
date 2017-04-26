/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <list>
#include <mutex>
#include <numeric>
#include <thread>
#include <condition_variable>

#include <unistd.h>
#include <signal.h>
#include <tinyxml2.h>

#include <sys/wait.h>

#include <ignition/common/Console.hh>

#include "ignition/launch/Config.hh"
#include "ignition/launch/Manager.hh"

using namespace ignition;
using namespace launch;

/// \brief A class to encapsulate an executable (program) to run.
class Executable
{
  /// \brief Default constructor
  public: Executable() = default;

  /// \brief Constructs a executable object with a name, process id, command,
  /// and other options. This class is used primarily for internal
  /// book keeping.
  /// \param[in] _name Name of the executable. This is used for internal
  /// book keeping, it is not (necessarily) the name of the executable that
  /// will be run.
  /// \param[in] _pid The pid in which the executable is run.
  /// \param[in] _cmd The command and arguments that specify how to run the
  /// executable.
  /// \param[in] _autoRestart True if the executable should restart when it
  /// is is killed.
  public: Executable(const std::string &_name, const pid_t _pid,
                     const std::vector<std::string> &_cmd, bool _autoRestart)
          : name(_name), pid(_pid), command(_cmd), autoRestart(_autoRestart)
          {}

  /// \brief Name of the executable
  public: std::string name = "";

  /// \brief Process id in which the executable is run.
  public: pid_t pid = -1;

  /// \brief The command to run.
  public: std::vector<std::string> command;

  /// \brief True will cause the command to restart on kill
  public: bool autoRestart = false;
};

/// \brief Private data variables for the Ignition class.
class ignition::launch::ManagerPrivate
{
  /// \brief Constructor.
  public: ManagerPrivate() {}

  /// \brief Load the system.
  /// \param[in] _args Command line arguments.
  /// \return True on success.
  public: bool Load(const std::map<std::string, std::string> &_args);

  /// \brief Parse a configuration file.
  /// \param[in] _filename Name of the XML file to parse.
  /// \return True on success.
  public: bool ParseConfig(const std::string &_filename);

  /// \brief Fork a new process for a command specified by _exec. This
  /// function will create a new Executable and add it to the executables
  /// list.
  /// \param[in] _exec An executable definition to run.
  /// \return True on success.
  public: bool RunExecutable(const Executable &_exec);

  /// \brief Fork a new process for a command specific by _cmd.
  /// \param[in] _name A unique name given to the command. This name is
  /// used for book keeping and control of the process.
  /// \param[in] _cmd A vector of strings where the first string is
  /// expected to be the name of the executable to run, and subsequent
  /// strings are arguments.
  /// \param[in] _autoRestart True if the executable should auto restart on
  /// death.
  /// \return True on success.
  public: bool RunExecutable(const std::string &_name,
                             const std::vector<std::string> &_cmd,
                             const bool _autoRestart);

  /// \brief Print help information to stdout.
  public: void PrintUsage();

  /// \brief Signal interrupt handler.
  /// \param[in] _v Signint value.
  public: static void SigInt(int _v);

  /// \brief Signal child handler.
  /// \param[in] _v Signchld value.
  public: static void SigChild(int _v);

  /// \brief A list of executables that are running, or have been run.
  public: std::list<Executable> executables;

  /// \brief Mutex to protect the executables list.
  public: std::mutex executablesMutex;

  /// \brief A condition variable used by SIG_INT and Manager::Run.
  public: std::condition_variable runCondition;

  /// \brief A mutex used in conjunction with runCondition.
  public: std::mutex runMutex;

  /// \brief True if running.
  public: bool running = false;

  public: bool master = false;

  /// \brief Pointer to myself. This is used in the signal handlers.
  /// A raw pointer is acceptable here since it is used only internally.
  public: static ManagerPrivate *myself;
};

ManagerPrivate *ManagerPrivate::myself = nullptr;

/////////////////////////////////////////////////
Manager::Manager()
  :dataPtr(new ManagerPrivate)
{
  this->dataPtr->myself = this->dataPtr.get();

  // Make sure to initialize logging.
  ignLogInit("~/.ign-launch", "ignition.log");

  // Register a signal handler to capture CTRL-C events.
  if (signal(SIGINT, ManagerPrivate::SigInt) == SIG_ERR)
    ignerr << "signal(2) failed while setting up for SIGINT" << std::endl;

  // Register a signal handler to capture child process death events.
  if (signal(SIGCHLD, ManagerPrivate::SigChild) == SIG_ERR)
    ignerr << "signal(2) failed while setting up for SIGCHLD" << std::endl;
}

/////////////////////////////////////////////////
Manager::~Manager()
{
}

/////////////////////////////////////////////////
bool Manager::Run(const std::map<std::string, std::string> &_args)
{
  std::unique_lock<std::mutex> lock(this->dataPtr->runMutex);

  this->dataPtr->master = true;

  // Load the system based on command line arguments.
  // The Load command will fork processes for each executable.
  if (!this->dataPtr->Load(_args))
  {
    ignerr << "Failed to load ignition.\n";
    return false;
  }

  // Child processes should exit
  if (!this->dataPtr->master)
    return true;

  this->dataPtr->running = true;

  // Wait for a shutdown event.
  // \todo: In the future, we could add execution models that control plugin
  // updates.
  while (this->dataPtr->running)
  {
    this->dataPtr->runCondition.wait(lock);
  }

  // \todo: Shutdown the plugins, when they are used.

  // Shutdown executables.
  {
    std::lock_guard<std::mutex> mutex(this->dataPtr->executablesMutex);

    // Remove the sigchld signal handler
    signal(SIGCHLD, nullptr);

    // Create a vector of monitor threads that wait for each process to stop.
    std::vector<std::thread> monitors;
    for (auto &exec : this->dataPtr->executables)
      monitors.push_back(std::thread([&] {waitpid(exec.pid, nullptr, 0);}));

    // Shutdown the processes
    for (auto &exec : this->dataPtr->executables)
    {
      igndbg << "Killing the process[" << exec.name
             << "] with PID[" << exec.pid << "]\n";
      kill(exec.pid, SIGINT);
    }

    igndbg << "Waiting for each process to end\n";

    // Wait for all the monitors to stop
    for (auto &m : monitors)
      m.join();
  }

  // If we got here, then everything was hunky-dorry
  return true;
}

/////////////////////////////////////////////////
bool Manager::Stop()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->runMutex);

  if (this->dataPtr->running)
  {
    this->dataPtr->running = false;
    this->dataPtr->runCondition.notify_all();
  }

  return this->dataPtr->running;
}

/////////////////////////////////////////////////
bool ManagerPrivate::Load(const std::map<std::string, std::string> &_args)
{
  // Set the verbose level first.
  if (_args.find("verbose") != _args.end())
  {
    int v = 1;
    try
    {
      v = std::stoi(_args.at("verbose"));
    }
    catch(...)
    {
    }

    ignition::common::Console::SetVerbosity(v);
  }

  if (_args.find("run") != _args.end())
  {
    // Check for empty
    if (_args.at("run").empty())
    {
      ignerr << "Empty --run argument\n";
      return false;
    }

    // Split the provided string, and attempt to run the executable.
    if (!this->RunExecutable("default",
          ignition::common::split(_args.at("run"), " "), false))
    {
      ignerr << "Unable to run command[" << _args.at("run")
        << "], as specified "
        << "with with -run argument. "
        << "Include a valid executable, with or without arguments, enclosed in "
        << "double quotes. For example (on linux systems): \n"
        << "\tignition -run \"echo hello\"\n";
      return false;
    }
    else
      return true;
  }
  else if (_args.find("config") != _args.end())
  {
    // Process the configuration file.
    return this->ParseConfig(_args.at("config"));
  }

  ignerr << "Missing a valid command to process. Nothing to do.\n";

  return false;
}

/////////////////////////////////////////////////
void ManagerPrivate::PrintUsage()
{
  std::cerr << "ignition -- Run the Ignition program and plugin manager.\n\n";
  std::cerr << "`ignition` [options]\n\n";
  std::cerr << "Ignition launches the specified programs and/or plugins.\n\n";
}

/////////////////////////////////////////////////
void ManagerPrivate::SigChild(int)
{
  pid_t p;
  int status;

  // Executable to restart
  Executable restartExec;

  if ((p = waitpid(-1, &status, WNOHANG)) != -1)
  {
    std::lock_guard<std::mutex> mutex(myself->executablesMutex);

    // Find the executable
    for (std::list<Executable>::iterator iter = myself->executables.begin();
         iter != myself->executables.end(); ++iter)
    {
      if (iter->pid == p)
      {
        igndbg << "Death of process[" << p << "] with name["
               << iter->name << "].\n";

        // Restart if autoRestart is enabled
        if (iter->autoRestart)
        {
          restartExec = *iter;
          myself->executables.erase(iter);
        }
        break;
      }
    }
  }

  if (!restartExec.name.empty() && !restartExec.command.empty())
  {
    igndbg << "Restarting process with name[" << restartExec.name << "]\n";
    myself->RunExecutable(restartExec);
  }
}

/////////////////////////////////////////////////
void ManagerPrivate::SigInt(int)
{
  std::lock_guard<std::mutex> lock(myself->runMutex);
  myself->runCondition.notify_all();
}

/////////////////////////////////////////////////
bool ManagerPrivate::ParseConfig(const std::string &_filename)
{
  tinyxml2::XMLDocument xmlDoc;

  // Load the XML configuration file into TinyXML
  if (xmlDoc.LoadFile(_filename.c_str()) != tinyxml2::XML_SUCCESS)
  {
    ignerr << "Unable to load config file[" << _filename << "]\n";
    return false;
  }

  // Get the root element.
  tinyxml2::XMLElement *root = xmlDoc.FirstChildElement("ignition");
  if (!root)
  {
    ignerr << "Invalid config file[" << _filename << "]. "
      << "Missing <ignition> element\n";
    return false;
  }

  // Process all the executables.
  tinyxml2::XMLElement *execElem = root->FirstChildElement("executable");

  // This "i" variable is just used for output messages.
  for (int i = 0; execElem;)
  {
    bool autoRestart = false;
    bool valid = true;
    std::vector<std::string> cmdParts;

    // Get the executable's name
    std::string nameStr = execElem->Attribute("name");
    if (nameStr.empty())
    {
      valid = false;
      ignerr << "Invalid configuration file[" << _filename << "]. "
        << "Missing name attribute for the " << i << " <executable> element."
        << std::endl;
    }

    // Get the command
    tinyxml2::XMLElement *cmdElem = execElem->FirstChildElement("command");
    if (!cmdElem)
    {
      valid = false;
      ignerr << "Invalid configuration file[" << _filename << "]. "
        << " Missing <command> child element "
        << " of <executable name=\"" << nameStr << "\">\n";
    }
    else
    {
      auto parts = ignition::common::split(cmdElem->GetText(), " ");
      std::move(parts.begin(), parts.end(), std::back_inserter(cmdParts));
    }

    // Get the <auto_restart> element. It is okay if the <auto_restart> element
    // is not present
    tinyxml2::XMLElement *restartElem = execElem->FirstChildElement(
        "auto_restart");
    if (restartElem && restartElem->GetText())
    {
      std::string txt = ignition::common::lowercase(restartElem->GetText());
      autoRestart = txt == "true" || txt == "1" || txt == "t";
    }

    // Get the arguments
    tinyxml2::XMLElement *argElem = execElem->FirstChildElement("arg");
    while (argElem)
    {
      std::string arg = argElem->GetText();
      if (arg.empty())
      {
        ignwarn << "Configuration file[" << _filename << "] has an empty "
          << "argument in <element name=\"" << nameStr << "\">\n";
      }
      else
      {
        cmdParts.push_back(argElem->GetText());
      }
      argElem = argElem->NextSiblingElement("arg");
    }

    if (valid)
    {
      if (!this->RunExecutable(nameStr, cmdParts, autoRestart))
      {
        ignerr << "Unable to run executable named[" << nameStr << "] in "
          << "configuration file[" << _filename << "].\n";
      }
    }

    execElem = execElem->NextSiblingElement("executable");
  }

  return true;
}

/////////////////////////////////////////////////
bool ManagerPrivate::RunExecutable(const Executable &_exec)
{
  return this->RunExecutable(_exec.name, _exec.command, _exec.autoRestart);
}

/////////////////////////////////////////////////
bool ManagerPrivate::RunExecutable(const std::string &_name,
    const std::vector<std::string> &_cmd, bool _autoRestart)
{
  // Check for empty
  if (_cmd.empty())
  {
    ignerr << "Empty command.\n";
    return false;
  }

  // Fork a process for the command
  pid_t pid = fork();

  // If parent process...
  if (pid)
  {
    igndbg << "Forked a process for [" << _name << "] command["
      << std::accumulate(_cmd.begin(), _cmd.end(), std::string("")) << "]\n"
      << std::flush;

    std::lock_guard<std::mutex> mutex(this->executablesMutex);
    // Store the PID in the parent process.
    this->executables.push_back(Executable(_name, pid, _cmd, _autoRestart));
  }
  // Else child process...
  else
  {
    // A child is not the master
    this->master = false;

    // Create a vector of char* in the child process
    std::vector<char*> cstrings;
    for (auto const &part : _cmd)
    {
      cstrings.push_back(const_cast<char *>(part.c_str()));
    }

    // Add the nullptr termination.
    cstrings.push_back(nullptr);

    // Remove from foreground process group.
    setpgid(0, 0);

    // Run the command
    if (execvp(cstrings[0], &cstrings[0]) < 0)
    {
      ignerr << "Unable to run command["
        << std::accumulate(_cmd.begin(), _cmd.end(), std::string("")) << "]\n";
    }
  }

  return true;
}
